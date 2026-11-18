/**
  ******************************************************************************
  * @file    wheel_speed.c
  * @brief   轮速闭环实现（串级内环，基于 NEUQ_PID 的 IncrementalPID）
  *
  *          直接用库里为"直道速度环"准备好的两个全局增量式 PID：
  *              stra_speed_wheel_1  → 左轮
  *              stra_speed_wheel_2  → 右轮
  *          以及库函数：
  *              Ipid_parameters_setting()   写增益、清误差
  *              IncrementalPID_Update()     返回本拍 PWM 增量
  *
  *          结构：
  *            Δpwm = Kp*(e_k - e_{k-1}) + Ki*e_k          ← 库 IncrementalPID_Update()
  *            pwm  = clamp(pwm + Δpwm, -10000, +10000)
  *
  *          库没管、但工程上必须处理的两点，放在本文件：
  *            1) 积分饱和：输出顶到限幅、误差又推同一个方向时，本拍不再累加；
  *            2) 停车抖动：目标为 0 且轮子已经很慢时直接清零并复位 PI，
  *               否则增量式 PI 会在 0 附近来回抖，车"咯噔咯噔"。
  ******************************************************************************
  */
#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "encoder.h"
#include "motor.h"
#include "wheel_speed.h"

#define WHEEL_BRAKE_PWM_MAX     3000.0f   /* 目标为 0 时的反向制动上限 */
#define WHEEL_TARGET_DEADBAND   0.5f      /* 目标绝对值小于它视为 0 */
#define WHEEL_STOP_SPEED        2.0f      /* 轮速小于它视为已经停住 */

static volatile float s_target_l = 0.0f;
static volatile float s_target_r = 0.0f;
static float s_pwm_l = 0.0f;
static float s_pwm_r = 0.0f;

/* 指向库里的两个全局增量式 PID（左右各一个） */
static IncrementalPID *const s_pid_l = &stra_speed_wheel_1;
static IncrementalPID *const s_pid_r = &stra_speed_wheel_2;

/* --------------------------------------------------------------------------
 * 成对写入目标（主循环 → 中断）
 * ------------------------------------------------------------------------ */
void WheelSpeed_SetTargets(float left_cm_s, float right_cm_s)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    s_target_l = left_cm_s;
    s_target_r = right_cm_s;
    if (primask == 0u) {
        __enable_irq();
    }
}

void WheelSpeed_GetTargets(float *left_cm_s, float *right_cm_s)
{
    if (left_cm_s != 0) {
        *left_cm_s = s_target_l;
    }
    if (right_cm_s != 0) {
        *right_cm_s = s_target_r;
    }
}

/* --------------------------------------------------------------------------
 * 用库函数初始化，再把库写死的 max_delta_pwm(6000) 改成本车的节拍限幅
 * ------------------------------------------------------------------------ */
static void pid_setup(IncrementalPID *pid, float kp, float ki, float kd)
{
    Ipid_parameters_setting(pid, kp, ki, kd);   /* 库：写增益 + 清 error[]，max_delta=6000 */
    pid->max_delta_pwm = WHEEL_DELTA_MAX;       /* 按 10 ms 节拍改成自己的限幅 */
}

void WheelSpeed_SetGains(float kp, float ki, float kd)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    pid_setup(s_pid_l, kp, ki, kd);
    pid_setup(s_pid_r, kp, ki, kd);
    if (primask == 0u) {
        __enable_irq();
    }
}

void WheelSpeed_Init(void)
{
    pid_setup(s_pid_l, WHEEL_KP, WHEEL_KI, WHEEL_KD);
    pid_setup(s_pid_r, WHEEL_KP, WHEEL_KI, WHEEL_KD);
    s_pwm_l = 0.0f;
    s_pwm_r = 0.0f;
    s_target_l = 0.0f;
    s_target_r = 0.0f;
    Motor_Stop();
}

void WheelSpeed_Reset(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    s_pid_l->error[0] = 0.0f;
    s_pid_l->error[1] = 0.0f;
    s_pid_l->error[2] = 0.0f;
    s_pid_r->error[0] = 0.0f;
    s_pid_r->error[1] = 0.0f;
    s_pid_r->error[2] = 0.0f;
    s_pwm_l = 0.0f;
    s_pwm_r = 0.0f;
    if (primask == 0u) {
        __enable_irq();
    }
}

void WheelSpeed_Stop(void)
{
    WheelSpeed_SetTargets(0.0f, 0.0f);
    WheelSpeed_Reset();
    Motor_Stop();
}

/* --------------------------------------------------------------------------
 * 单侧：库的增量 PI + 限幅 + 停机处理
 * ------------------------------------------------------------------------ */
static float wheel_update_one(IncrementalPID *pid, float *pwm,
                              float target, float speed)
{
    float delta;
    float next;
    float abs_target;

    abs_target = BOARD_ABS(target);
    if (abs_target < WHEEL_TARGET_DEADBAND) {
        /* 目标为 0：已经停住就直接清干净，否则允许有限的反向制动 */
        if (BOARD_ABS(speed) < WHEEL_STOP_SPEED) {
            pid->error[0] = 0.0f;
            pid->error[1] = 0.0f;
            pid->error[2] = 0.0f;
            *pwm = 0.0f;
            return 0.0f;
        }
        delta = IncrementalPID_Update(pid, 0.0f, speed);
        next  = *pwm + delta;
        if (next < -WHEEL_BRAKE_PWM_MAX) {
            next = -WHEEL_BRAKE_PWM_MAX;
        }
        if (next > WHEEL_BRAKE_PWM_MAX) {
            next = WHEEL_BRAKE_PWM_MAX;
        }
        *pwm = next;
        return next;
    }

    delta = IncrementalPID_Update(pid, target, speed);
    next  = *pwm + delta;

    if (next > (float)PWM_OUT_MAX) {
        next = (float)PWM_OUT_MAX;
        /* 顶到上限且还想往上加：丢掉本拍增量，避免积分继续膨胀 */
        if (delta > 0.0f) {
            *pwm = next;
            return next;
        }
    } else if (next < (float)PWM_OUT_MIN) {
        next = (float)PWM_OUT_MIN;
        if (delta < 0.0f) {
            *pwm = next;
            return next;
        }
    } else {
        /* 正常区间 */
    }

    *pwm = next;
    return next;
}

void WheelSpeed_Update(void)
{
    float target_l;
    float target_r;
    float pwm_l;
    float pwm_r;

    /* 目标成对读出，避免本拍用的是"一新一旧" */
    target_l = s_target_l;
    target_r = s_target_r;

#if (WHEEL_LOOP_ENABLE == 0)
    /* 开环调试模式：目标直接当占空比用（标定方向、死区、速度-占空比曲线） */
    Motor_SetLeft((int16_t)BOARD_CLAMP(target_l * 50.0f, PWM_OUT_MIN, PWM_OUT_MAX));
    Motor_SetRight((int16_t)BOARD_CLAMP(target_r * 50.0f, PWM_OUT_MIN, PWM_OUT_MAX));
    return;
#else
    pwm_l = wheel_update_one(s_pid_l, &s_pwm_l, target_l, Encoder_GetSpeedL());
    pwm_r = wheel_update_one(s_pid_r, &s_pwm_r, target_r, Encoder_GetSpeedR());

    Motor_SetLeft((int16_t)pwm_l);
    Motor_SetRight((int16_t)pwm_r);
#endif
}

float WheelSpeed_GetPwmL(void)
{
    return s_pwm_l;
}

float WheelSpeed_GetPwmR(void)
{
    return s_pwm_r;
}
