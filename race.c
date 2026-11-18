/**
  ******************************************************************************
  * @file    race.c
  * @brief   赛道状态机实现（巡线 / 十字 / 环岛 / 160 cm 断路 / 停车）
  *
  *          三个"安全兜底"贯穿全文：
  *            1) 丢线：先沿用上一帧偏差，再用陀螺保航向，最后停车报警；
  *            2) 元素：进入有条件（标志持续 + 距离间隔），退出有里程/转角判据，
  *               并且都有最大里程/超时保护，绝不会"一个状态卡死冲出去"；
  *            3) 停车：目标归零后还要实测轮速降下来才算停稳。
  ******************************************************************************
  */
#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "alarm.h"
#include "board_io.h"
#include "chassis.h"
#include "comm.h"
#include "encoder.h"
#include "imu.h"
#include "line_follow.h"
#include "race.h"
#include "wheel_speed.h"

typedef enum
{
    RING_PHASE_ENTER = 0,
    RING_PHASE_FOLLOW,
    RING_PHASE_EXIT
} RingPhase;

static RaceState s_state = RACE_STATE_INIT;
static uint32_t  s_state_ms = 0;
static uint32_t  s_race_start_ms = 0;
static uint8_t   s_lap = 0;

static float     s_cmd_v = 0.0f;
static float     s_cmd_w = 0.0f;

/* --- 元素上下文 --- */
static float     s_elem_yaw = 0.0f;
static float     s_elem_odom = 0.0f;
static float     s_elem_odom_exit = 0.0f;
static float     s_last_element_odom = 0.0f;
static float     s_ring_omega_sign = 1.0f;
static RingPhase s_ring_phase = RING_PHASE_ENTER;
static uint32_t  s_ring_phase_ms = 0;
/* 环岛累计转角：必须用"逐拍增量累加"，不能用 wrap_pi(当前-入环) 直接相减，
 * 否则转过 180° 之后差值会被卷回负值，永远到不了 320°（本仓库回归测试抓到过）。 */
static float     s_ring_progress = 0.0f;
static float     s_prev_heading = 0.0f;

/* --- 视觉丢线处理 --- */
static float     s_hold_dev = 0.0f;
static uint32_t  s_vision_last_ok_ms = 0;
static float     s_hold_yaw = 0.0f;
static uint8_t   s_hold_yaw_valid = 0;

/* --- 标志确认定时器 --- */
static uint32_t  s_flag_cross_since = 0;
static uint32_t  s_flag_ring_since = 0;
static uint32_t  s_flag_gap_since = 0;
static uint32_t  s_flag_relock_since = 0;

/* --- 航向（无陀螺时的轮速差估计） --- */
static float     s_odom_yaw = 0.0f;

/* --- 停车 --- */
static uint32_t  s_stop_stable_since = 0;
static uint32_t  s_lap1_ms = 0;
static uint8_t   s_stop_request = 0;

/* --- 可在线调整的参数 --- */
static float     s_v_base = V_LINE_CM_S;

/* ==========================================================================
 * 小工具
 * ======================================================================== */
static float wrap_pi(float a)
{
    while (a > BOARD_PI_F) {
        a -= BOARD_TWO_PI_F;
    }
    while (a < -BOARD_PI_F) {
        a += BOARD_TWO_PI_F;
    }
    return a;
}

static const char *k_state_names[] =
{
    "INIT", "WAIT", "LINE", "CROSS", "RING", "GAP", "STOP", "FINISH"
};

const char *Race_StateName(RaceState state)
{
    if ((uint8_t)state > (uint8_t)RACE_STATE_FINISH) {
        return "?";
    }
    return k_state_names[(uint8_t)state];
}

static void beep(uint16_t ms)
{
    BoardIO_Buzzer(ms);
}

/* 当前航向：有陀螺用陀螺，没有就用轮速差 */
static float heading(void)
{
    if (IMU_IsReady()) {
        return IMU_GetYawRad();
    }
    return s_odom_yaw;
}

static float yaw_rate(void)
{
    if (IMU_IsReady()) {
        return IMU_GetYawRate();
    }
    return (Encoder_GetSpeedR() - Encoder_GetSpeedL()) / TRACK_WIDTH_CM;
}

static uint32_t vision_flags(void)
{
    return (uint32_t)Comm_GetVision()->flags;
}

static uint8_t vision_ok(void)
{
    if (!Comm_VisionIsFresh(HAL_GetTick())) {
        return 0;
    }
    return ((vision_flags() & VISION_FLAG_VALID) != 0u) ? 1u : 0u;
}

/* 把跨回合的残留清掉 */
static void clear_flag_timers(void)
{
    s_flag_cross_since  = 0;
    s_flag_ring_since   = 0;
    s_flag_gap_since    = 0;
    s_flag_relock_since = 0;
}

/* 标志连续保持 need_ms 才认可 */
static uint8_t confirm_flag(uint32_t *since, uint8_t present,
                            uint32_t need_ms, uint32_t now)
{
    if (!present) {
        *since = 0;
        return 0;
    }
    if (*since == 0u) {
        *since = (now == 0u) ? 1u : now;
        return 0;
    }
    return ((now - *since) >= need_ms) ? 1u : 0u;
}

/* 弯道降速 */
static float line_speed_for_dev(float dev)
{
    float a;
    float v_curve;

    /* 弯道速度按基础速度等比缩放，改基础速度时弯道跟着变 */
    v_curve = s_v_base * (V_CURVE_CM_S / V_LINE_CM_S);

    a = BOARD_ABS(dev);
    if (a >= LINE_CURVE_DEV) {
        return v_curve;
    }
    if (a >= LINE_CURVE_DEV * 0.5f) {
        return (s_v_base + v_curve) * 0.5f;
    }
    return s_v_base;
}

/* 下发目标（唯一的输出口） */
static void publish(float v, float omega, float dt)
{
    float left;
    float right;

    s_cmd_v = v;
    s_cmd_w = omega;
    Chassis_Apply(v, omega, dt, &left, &right);
    WheelSpeed_SetTargets(left, right);
}

/* ==========================================================================
 * 状态进入：所有"切状态要做的初始化"集中在这里，避免散落各处
 * ======================================================================== */
static void state_enter(RaceState next)
{
    uint32_t now;

    now = HAL_GetTick();
    s_state = next;
    s_state_ms = now;
    s_stop_stable_since = 0;
    clear_flag_timers();

    switch (next) {
    case RACE_STATE_CROSS:
        s_elem_yaw  = heading();
        s_elem_odom = Encoder_GetDistAvg();
        LineFeed_Reset();          /* 十字是直行，外环积分清零再回来 */
        break;

    case RACE_STATE_RING:
        s_elem_yaw  = heading();
        s_elem_odom = Encoder_GetDistAvg();
        /* 进环方向：dev>0 说明赛道中心在右，需要右转 ⇒ 顺时针环 */
        s_ring_omega_sign = (s_hold_dev >= 0.0f) ? -1.0f : 1.0f;
        s_ring_phase = RING_PHASE_ENTER;
        s_ring_phase_ms = now;
        s_ring_progress = 0.0f;
        s_prev_heading = heading();
        LineFeed_Reset();
        break;

    case RACE_STATE_GAP:
        s_elem_yaw  = heading();
        s_elem_odom = Encoder_GetDistAvg();
        LineFeed_Reset();
        break;

    case RACE_STATE_STOP:
        WheelSpeed_SetTargets(0.0f, 0.0f);
        break;

    case RACE_STATE_FINISH:
        WheelSpeed_SetTargets(0.0f, 0.0f);
        beep(300);
        break;

    default:
        break;
    }
}

/* ==========================================================================
 * 各状态实现
 * ======================================================================== */
static void race_wait_start(float dt)
{
    uint32_t now;

    now = HAL_GetTick();

    publish(0.0f, 0.0f, dt);

    /* 两个灯：LED1 心跳，LED2 视觉在线 */
    BoardIO_LedSet(BOARD_LED_1, (uint8_t)(((now / 250u) & 1u) != 0u));
    BoardIO_LedSet(BOARD_LED_2, vision_ok());

    if (BoardIO_WasPressed(BOARD_KEY_START)) {
        if ((RACE_REQUIRE_VISION == 0) || vision_ok()) {
            Encoder_ResetOdom();
            Chassis_Init();
            LineFeed_Reset();
            WheelSpeed_Reset();
            s_odom_yaw = 0.0f;
            s_hold_dev = 0.0f;
            s_hold_yaw_valid = 0;
            s_vision_last_ok_ms = now;
            s_race_start_ms = now;
            s_lap = 1;
            s_lap1_ms = 0;
            s_last_element_odom = 0.0f;
            s_stop_request = 0;
            Alarm_Clear();
            IMU_SetYawRad(0.0f);
            beep(80);
            state_enter(RACE_STATE_LINE);
        } else {
            beep(400);   /* 视觉没就绪，不让发车 */
        }
    }
}

static float vision_dev(void)
{
    return Comm_GetVision()->dev;
}

static void race_line(float dt)
{
    uint32_t now;
    uint32_t age;
    float    dev;
    float    v;
    float    w;
    float    dist;
    float    travelled;
    uint8_t  fresh;
    uint8_t  gap_present;
    uint8_t  cross_present;
    uint8_t  ring_present;

    now   = HAL_GetTick();
    fresh = vision_ok();
    dist  = Encoder_GetDistAvg();

    if (s_stop_request || Alarm_IsActive()) {
        state_enter(RACE_STATE_STOP);
        return;
    }

    if (fresh) {
        dev = vision_dev();
        s_hold_dev = dev;
        s_vision_last_ok_ms = now;
        s_hold_yaw_valid = 0;
    }
    age = now - s_vision_last_ok_ms;

    if (fresh) {
        v = line_speed_for_dev(dev);
        w = LineFeed_Update(dev, v, dt);
    } else if (age <= VISION_HOLD_MS) {
        /* 短时丢线：沿用上一帧偏差，降速 */
        v = V_RECOVER_CM_S;
        w = LineFeed_Update(s_hold_dev, v, dt);
    } else if (age <= VISION_LOST_STOP_MS) {
        /* 长时间丢线：保最后一次有视觉时的航向，低速找线 */
        if (!s_hold_yaw_valid) {
            s_hold_yaw = heading();
            s_hold_yaw_valid = 1;
        }
        v = V_RECOVER_CM_S;
        w = LineFeed_YawHold(wrap_pi(s_hold_yaw - heading()), yaw_rate(), dt);
    } else {
        Alarm_Raise(ALARM_VISION_LOST);
        state_enter(RACE_STATE_STOP);
        return;
    }

    /* ---------------- 元素识别 ---------------- */
    travelled = dist - s_last_element_odom;

    gap_present   = ((vision_flags() & VISION_FLAG_GAP) != 0u) ? 1u : 0u;
    cross_present = ((vision_flags() & VISION_FLAG_CROSS) != 0u) ? 1u : 0u;
    ring_present  = ((vision_flags() & VISION_FLAG_RING_IN) != 0u) ? 1u : 0u;

#if (GAP_BY_ODOM_ENABLE == 1)
    if (dist >= GAP_ODOM_START_CM) {
        gap_present = 1u;
    }
#endif

    /* 视觉标志只有在新鲜时才可信 */
    if (!fresh) {
        gap_present   = 0u;
        cross_present = 0u;
        ring_present  = 0u;
    }

    /* 三个标志都要先跑够距离，防止同一处抖动反复触发 */
    if (travelled > ELEMENT_MIN_GAP_CM) {
        /* 断路优先级最高：一旦进了断路，别的标志都不看 */
        if (confirm_flag(&s_flag_gap_since, gap_present, GAP_CONFIRM_MS, now)) {
            beep(40);
            state_enter(RACE_STATE_GAP);
            publish(v, w, dt);
            return;
        }
        if (confirm_flag(&s_flag_cross_since, cross_present, CROSS_CONFIRM_MS, now)) {
            beep(40);
            state_enter(RACE_STATE_CROSS);
            publish(v, w, dt);
            return;
        }
        if (confirm_flag(&s_flag_ring_since, ring_present, RING_CONFIRM_MS, now)) {
            beep(40);
            state_enter(RACE_STATE_RING);
            publish(v, w, dt);
            return;
        }
    }

    /* ---------------- 圈数 / 停车标志 ---------------- */
    if ((vision_flags() & VISION_FLAG_STOP) != 0u) {
        s_lap1_ms = now - s_race_start_ms;
        state_enter(RACE_STATE_STOP);
        return;
    }
    if (dist >= LAP_LENGTH_CM) {
        s_lap1_ms = now - s_race_start_ms;
        state_enter(RACE_STATE_STOP);
        return;
    }

    publish(v, w, dt);
}

static void race_cross(float dt)
{
    uint32_t now;
    float    v;
    float    w;
    float    travelled;
    uint8_t  relock;
    uint32_t flags;

    now = HAL_GetTick();
    if (s_stop_request || Alarm_IsActive()) {
        state_enter(RACE_STATE_STOP);
        return;
    }

    travelled = Encoder_GetDistAvg() - s_elem_odom;
    v = V_CROSS_CM_S;

    /* 陀螺（或轮速差）保直行：十字走错就是"打舵了"，所以这里不跟视觉偏差 */
    w = LineFeed_YawHold(wrap_pi(s_elem_yaw - heading()), yaw_rate(), dt);

    flags  = vision_flags();
    relock = (vision_ok() &&
              ((flags & VISION_FLAG_LINE) != 0u) &&
              ((flags & VISION_FLAG_CROSS) == 0u)) ? 1u : 0u;

    /* 超过最大里程：不管看不看到线都必须退出，避免一路直冲出赛道 */
    if (travelled >= CROSS_MAX_CM) {
        s_last_element_odom = Encoder_GetDistAvg();
        state_enter(RACE_STATE_LINE);
        return;
    }

    if ((travelled >= CROSS_MIN_CM) &&
        confirm_flag(&s_flag_relock_since, relock, CROSS_RELOCK_MS, now)) {
        s_last_element_odom = Encoder_GetDistAvg();
        state_enter(RACE_STATE_LINE);
        return;
    }

    publish(v, w, dt);
}

static void race_ring(float dt)
{
    uint32_t now;
    float    dist;
    float    travelled;
    float    v;
    float    w;
    float    progress;
    uint32_t flags;
    uint8_t  fresh;

    now = HAL_GetTick();
    if (s_stop_request || Alarm_IsActive()) {
        state_enter(RACE_STATE_STOP);
        return;
    }

    dist      = Encoder_GetDistAvg();
    travelled = dist - s_elem_odom;
    fresh     = vision_ok();
    flags     = vision_flags();
    v         = V_RING_CM_S;

    /* 超时兜底：环里最多待 RING_MAX_TIME_MS，宁可退出也不能一直转 */
    if ((now - s_ring_phase_ms) > RING_MAX_TIME_MS) {
        s_last_element_odom = dist;
        beep(120);
        state_enter(RACE_STATE_LINE);
        return;
    }

    switch (s_ring_phase) {
    case RING_PHASE_ENTER:
        /* 进环：视觉还看得到入口就跟着偏差走，看不到就按环半径给角速度 */
        if (fresh) {
            w = LineFeed_UpdateScaled(vision_dev(), v, dt, 1.0f);
        } else {
            w = s_ring_omega_sign * (v / RING_RADIUS_CM);
        }
        if (travelled >= RING_ENTER_CM) {
            s_ring_phase = RING_PHASE_FOLLOW;
            s_ring_phase_ms = now;
        }
        break;

    case RING_PHASE_FOLLOW:
        /* 沿环：视觉可用就跟视觉，否则按 v/R 做圆周前馈 */
        if (fresh) {
            w = LineFeed_UpdateScaled(vision_dev(), v, dt, 0.85f);
        } else {
            w = s_ring_omega_sign * (v / RING_RADIUS_CM);
        }

        /* 累计转角：由 Race_Update 逐拍累加（见文件末尾），这里直接读 */
        progress = s_ring_progress;

        if ((progress >= (RING_EXIT_YAW_DEG * BOARD_DEG2RAD)) &&
            fresh && ((flags & VISION_FLAG_LINE) != 0u)) {
            s_ring_phase = RING_PHASE_EXIT;
            s_ring_phase_ms = now;
            s_elem_odom_exit = dist;
        }
        break;

    case RING_PHASE_EXIT:
    default:
        /* 出环：反向给一点舵把车头掰回赛道方向 */
        w = -s_ring_omega_sign * 0.5f * (v / RING_RADIUS_CM);
        if ((dist - s_elem_odom_exit) >= RING_EXIT_CM) {
            s_last_element_odom = dist;
            state_enter(RACE_STATE_LINE);
            return;
        }
        break;
    }

    publish(v, w, dt);
}

static void race_gap(float dt)
{
    float   v;
    float   w;
    float   travelled;
    uint32_t flags;
    uint8_t back;

    if (s_stop_request || Alarm_IsActive()) {
        state_enter(RACE_STATE_STOP);
        return;
    }

    travelled = Encoder_GetDistAvg() - s_elem_odom;
    v = V_GAP_CM_S;

    /* 断路里没有赛道，全靠入段航向保直行 */
    w = LineFeed_YawHold(wrap_pi(s_elem_yaw - heading()), yaw_rate(), dt);

    if (travelled >= GAP_MAX_CM) {
        /* 跑太远了还没看到赛道：停车报警，不许盲冲 */
        Alarm_Raise(ALARM_GAP_TIMEOUT);
        state_enter(RACE_STATE_STOP);
        return;
    }

    if (travelled >= GAP_TRAVEL_LEN_CM) {
        flags = vision_flags();
        back  = (vision_ok() && ((flags & VISION_FLAG_LINE) != 0u)) ? 1u : 0u;
        if (back) {
            s_last_element_odom = Encoder_GetDistAvg();
            state_enter(RACE_STATE_LINE);
            return;
        }
        /* 已经跑够长度但还没看到线：低速往前摸，等视觉恢复 */
        v = V_RECOVER_CM_S;
    }

    publish(v, w, dt);
}

static void race_stop(float dt)
{
    uint32_t now;

    now = HAL_GetTick();

    publish(0.0f, 0.0f, dt);

    /* 有故障就停在 STOP 里等人工处理，不往下走到 FINISH */
    if (Alarm_IsActive()) {
        return;
    }

    if ((BOARD_ABS(Encoder_GetSpeedL()) < SPEED_STOP_CM_S) &&
        (BOARD_ABS(Encoder_GetSpeedR()) < SPEED_STOP_CM_S)) {
        if (s_stop_stable_since == 0u) {
            s_stop_stable_since = (now == 0u) ? 1u : now;
        }
        if ((now - s_stop_stable_since) >= RACE_STOP_DWELL_MS) {
#if (RACE_RUN_SECOND_LAP == 1)
            if ((s_lap == 1u) && (s_lap1_ms != 0u)) {
                /* 第二圈：低速重跑（盲盒/数独/红障停 5 s 由后续版本实现） */
                s_lap = 2u;
                Encoder_ResetOdom();
                s_last_element_odom = 0.0f;
                s_stop_stable_since = 0;
                beep(100);
                state_enter(RACE_STATE_LINE);
                return;
            }
#endif
            state_enter(RACE_STATE_FINISH);
        }
    } else {
        s_stop_stable_since = 0;
    }
}

static void race_finish(float dt)
{
    publish(0.0f, 0.0f, dt);

    BoardIO_LedSet(BOARD_LED_3, 0);
    BoardIO_LedSet(BOARD_LED_1, 1);

    /* 再次按发车键：复位，准备下一轮 */
    if (BoardIO_WasPressed(BOARD_KEY_START)) {
        Race_Init();
    }
}

/* ==========================================================================
 * 对外接口
 * ======================================================================== */
void Race_Init(void)
{
    s_state = RACE_STATE_INIT;
    s_state_ms = HAL_GetTick();
    s_race_start_ms = 0;
    s_lap = 0;
    s_cmd_v = 0.0f;
    s_cmd_w = 0.0f;
    s_elem_yaw = 0.0f;
    s_elem_odom = 0.0f;
    s_elem_odom_exit = 0.0f;
    s_last_element_odom = 0.0f;
    s_ring_omega_sign = 1.0f;
    s_ring_phase = RING_PHASE_ENTER;
    s_ring_phase_ms = 0;
    s_ring_progress = 0.0f;
    s_prev_heading = 0.0f;
    s_hold_dev = 0.0f;
    s_vision_last_ok_ms = HAL_GetTick();
    s_hold_yaw = 0.0f;
    s_hold_yaw_valid = 0;
    s_odom_yaw = 0.0f;
    s_stop_stable_since = 0;
    s_lap1_ms = 0;
    s_stop_request = 0;
    clear_flag_timers();

    LineFeed_Init();
    WheelSpeed_SetTargets(0.0f, 0.0f);
    state_enter(RACE_STATE_WAIT_START);
}

void Race_Update(float dt)
{
    /* 没有陀螺时用轮速差积分一个航向出来 */
    s_odom_yaw += ((Encoder_GetSpeedR() - Encoder_GetSpeedL()) / TRACK_WIDTH_CM) * dt;
    s_odom_yaw = wrap_pi(s_odom_yaw);

    /* 环岛累计转角：用相邻两拍的航向差累加（差值很小，不会被 ±π 卷绕坑到） */
    {
        float h_now = heading();
        float d_yaw = wrap_pi(h_now - s_prev_heading);
        s_prev_heading = h_now;
        if (s_state == RACE_STATE_RING) {
            s_ring_progress += d_yaw * s_ring_omega_sign;
        }
    }

    /* 行驶中 LED3 常亮 */
    BoardIO_LedSet(BOARD_LED_3,
                   (s_state == RACE_STATE_LINE) || (s_state == RACE_STATE_CROSS) ||
                   (s_state == RACE_STATE_RING) || (s_state == RACE_STATE_GAP));

    /* 随时可用的急停键 */
    if (BoardIO_WasPressed(BOARD_KEY_AUX1)) {
        s_stop_request = 1;
    }

    switch (s_state) {
    case RACE_STATE_INIT:
        state_enter(RACE_STATE_WAIT_START);
        break;
    case RACE_STATE_WAIT_START:
        race_wait_start(dt);
        break;
    case RACE_STATE_LINE:
        race_line(dt);
        break;
    case RACE_STATE_CROSS:
        race_cross(dt);
        break;
    case RACE_STATE_RING:
        race_ring(dt);
        break;
    case RACE_STATE_GAP:
        race_gap(dt);
        break;
    case RACE_STATE_STOP:
        race_stop(dt);
        break;
    case RACE_STATE_FINISH:
        race_finish(dt);
        break;
    default:
        state_enter(RACE_STATE_STOP);
        break;
    }
}

void Race_RequestStop(void)
{
    s_stop_request = 1;
}

void Race_SetBaseSpeed(float cm_s)
{
    s_v_base = BOARD_CLAMP(cm_s, 20.0f, SPEED_MAX_CM_S);
}

RaceState Race_GetState(void)
{
    return s_state;
}

uint8_t Race_IsRunning(void)
{
    return ((s_state == RACE_STATE_LINE) || (s_state == RACE_STATE_CROSS) ||
            (s_state == RACE_STATE_RING) || (s_state == RACE_STATE_GAP)) ? 1u : 0u;
}

float Race_GetDistanceCm(void)
{
    return Encoder_GetDistAvg();
}

float Race_GetElementDistanceCm(void)
{
    return Encoder_GetDistAvg() - s_elem_odom;
}

uint32_t Race_GetLapTimeMs(void)
{
    if (s_lap1_ms != 0u) {
        return s_lap1_ms;
    }
    if (Race_IsRunning()) {
        return HAL_GetTick() - s_race_start_ms;
    }
    return 0u;
}

uint32_t Race_GetStateTimeMs(void)
{
    return HAL_GetTick() - s_state_ms;
}

uint8_t Race_GetLap(void)
{
    return s_lap;
}

float Race_GetCmdSpeed(void)
{
    return s_cmd_v;
}

float Race_GetCmdOmega(void)
{
    return s_cmd_w;
}
