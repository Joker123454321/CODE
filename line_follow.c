/**
  ******************************************************************************
  * @file    line_follow.c
  * @brief   巡线外环实现（基于 NEUQ_PID 的 pid_value + Unclipped_Positional_PID）
  *
  *          库里已经给了三个全局 pid_value，直接用它们，不再自己造 PID：
  *              yaw_angle_wheel       ← 方向环：视觉偏差 → 目标角速度 ω
  *              yaw_anglespeed_wheel  ← 航向环：yaw 误差 → ω（十字/断路直行用）
  *              yaw_speed_wheel       ← 预留（本版未用）
  *
  *          用法要点（也是库的口径）：
  *            OUT = Kp*Ek + Ki*Ek_total + Kd*(Ek - Ek1)
  *            Ek_total 逐拍累加（不乘 dt），D 用逐拍差分（不除 dt）。
  *            所以 Kd_库 = Kd_每秒 × 100、Ki_库 = Ki_每秒 × 0.01（见 board_config.h）。
  *
  *          小技巧：把"目标 = 0、测量 = dev"喂给库函数，输出天然是负号关系，
  *          即 dev>0（车偏左）→ OUT<0（右转），代码里不用到处写负号。
  ******************************************************************************
  */
#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "line_follow.h"

/* 库里的全局 pid_value */
static pid_value *const s_dir_pid = &yaw_angle_wheel;        /* 外环：偏差 → ω */
static pid_value *const s_yaw_pid = &yaw_anglespeed_wheel;   /* 直行保航向 */

static float   s_dev_filt   = 0.0f;
static float   s_omega_last = 0.0f;
static uint8_t s_dev_invert = VISION_DEV_INVERT;
static float   s_kp_scale   = 1.0f;   /* 现场调参倍数 */
static float   s_ki_scale   = 1.0f;
static float   s_kd_scale   = 1.0f;

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

/* 清一个库 pid 的状态（库只提供 pid_parameters_setting，会把增益一起改掉，
 * 所以这里只清状态量，不动增益） */
static void pid_clear_state(pid_value *pid)
{
    pid->Ek = 0.0f;
    pid->Ek1 = 0.0f;
    pid->Ek2 = 0.0f;
    pid->Ek_total = 0.0f;
    pid->OUT = 0.0f;
}

void LineFeed_Init(void)
{
    /* 库函数：写 Kp/Ki/Kd 并清状态 */
    pid_parameters_setting(s_dir_pid, LINE_KP_BASE, LINE_KI_BASE, LINE_KD_BASE);
    pid_parameters_setting(s_yaw_pid, YAW_KP, 0.0f, YAW_KD);
    LineFeed_Reset();
}

void LineFeed_Reset(void)
{
    pid_clear_state(s_dir_pid);
    pid_clear_state(s_yaw_pid);
    s_dev_filt   = 0.0f;
    s_omega_last = 0.0f;
}

void LineFeed_SetDevInvert(uint8_t invert)
{
    s_dev_invert = (invert != 0) ? 1 : 0;
}

void LineFeed_SetGainScale(float kp_scale, float ki_scale, float kd_scale)
{
    s_kp_scale = BOARD_CLAMP(kp_scale, 0.1f, 5.0f);
    s_ki_scale = BOARD_CLAMP(ki_scale, 0.0f, 5.0f);
    s_kd_scale = BOARD_CLAMP(kd_scale, 0.0f, 5.0f);
}

float LineFeed_Update(float dev, float v_cm_s, float dt)
{
    return LineFeed_UpdateScaled(dev, v_cm_s, dt, 1.0f);
}

float LineFeed_UpdateScaled(float dev, float v_cm_s, float dt, float gain_scale)
{
    float scale;
    float omega;
    float omega_max;
    float speed_for_limit;
    float max_i;

    if (dt <= 0.0f) {
        dt = CONTROL_DT_S;
    }
    if (s_dev_invert) {
        dev = -dev;
    }
    dev = BOARD_CLAMP(dev, -VISION_DEV_MAX, VISION_DEV_MAX);

    /* 偏差一阶低通：库的 D 是逐拍差分、没有滤波，噪声全靠这一级压住 */
    s_dev_filt += LINE_DEV_LPF_ALPHA * (dev - s_dev_filt);

    /* 速度增益调度：Kp/Ki/Kd ∝ v/v_ref（直接改库结构体的三个增益字段，
     * 不调用 pid_parameters_setting，否则会把积分一起清掉） */
    scale = v_cm_s / LINE_V_REF_CM_S;
    scale = BOARD_CLAMP(scale, LINE_GAIN_MIN_SCALE, LINE_GAIN_MAX_SCALE);
    scale *= BOARD_CLAMP(gain_scale, 0.0f, 2.0f);

    s_dir_pid->Kp = LINE_KP_BASE * scale * s_kp_scale;
    s_dir_pid->Ki = LINE_KI_BASE * scale * s_ki_scale;
    s_dir_pid->Kd = LINE_KD_BASE * scale * s_kd_scale;

    /* 库的 Max_I 限的是"误差累加和"，而我们要限的是"积分项输出 rad/s"，
     * 所以 Max_I = 允许的积分输出 / Ki */
    max_i = (s_dir_pid->Ki > 1e-6f) ? (LINE_I_OUT_MAX / s_dir_pid->Ki) : 0.0f;

    /* 目标 0、测量 dev：dev>0 → Ek<0 → ω<0（右转） */
    omega = Unclipped_Positional_ PID(s_dir_pid, 0.0f, s_dev_filt, max_i);

    /* ω 限幅：低速受 OMEGA_MAX，高速受侧向加速度约束 */
    speed_for_limit = (BOARD_ABS(v_cm_s) > 10.0f) ? BOARD_ABS(v_cm_s) : 10.0f;
    omega_max = LAT_ACC_MAX_CM_S2 / speed_for_limit;
    if (omega_max > OMEGA_MAX_RAD_S) {
        omega_max = OMEGA_MAX_RAD_S;
    }
    if (omega_max < 1.0f) {
        omega_max = 1.0f;
    }
    omega = BOARD_CLAMP(omega, -omega_max, omega_max);

    s_omega_last = omega;
    return omega;
}

float LineFeed_YawHold(float yaw_err_rad, float yaw_rate, float dt)
{
    float omega;

    (void)dt;
    (void)yaw_rate;   /* 阻尼由库的 D 项完成，见下面的注释 */

    yaw_err_rad = wrap_pi(yaw_err_rad);

    s_yaw_pid->Kp = YAW_KP;
    s_yaw_pid->Ki = 0.0f;
    s_yaw_pid->Kd = YAW_KD;

    /* 把 -yaw_err 当"测量值"传给库函数：
     *   Ek      = 0 - (-yaw_err) = yaw_err
     *   D 项    = Kd*(Ek-Ek1) = Kd*Δyaw_err = -Kd*dt*角速度  ← 正好是阻尼
     * 所以这里不需要再单独乘 yaw_rate，YAW_KD 已经等效为"每秒 0.35"的阻尼系数。 */
    omega = Unclipped_Positional_PID(s_yaw_pid, 0.0f, -yaw_err_rad, 0.0f);
    omega = BOARD_CLAMP(omega, -YAW_OMEGA_MAX, YAW_OMEGA_MAX);

    s_omega_last = omega;
    return omega;
}

float LineFeed_GetDevFiltered(void)
{
    return s_dev_filt;
}

float LineFeed_GetOmega(void)
{
    return s_omega_last;
}

void LineFeed_GetTerms(float *p_term, float *i_term, float *d_term)
{
    if (p_term != 0) {
        *p_term = s_dir_pid->Kp * s_dir_pid->Ek;
    }
    if (i_term != 0) {
        *i_term = s_dir_pid->Ki * s_dir_pid->Ek_total;
    }
    if (d_term != 0) {
        *d_term = s_dir_pid->Kd * (s_dir_pid->Ek - s_dir_pid->Ek1);
    }
}
