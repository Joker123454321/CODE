/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   差速解算 + 加速度限幅
  ******************************************************************************
  */
#include "board_config.h"
#include "chassis.h"

static float s_lim_l = 0.0f;
static float s_lim_r = 0.0f;

void Chassis_Init(void)
{
    s_lim_l = 0.0f;
    s_lim_r = 0.0f;
}

void Chassis_Split(float v_cm_s, float omega_rad_s,
                   float *left_cm_s, float *right_cm_s)
{
    float half_b;

    /* 轮距用 cm，ω 用 rad/s，乘出来正好是 cm/s */
    half_b = TRACK_WIDTH_CM * 0.5f;

    if (left_cm_s != 0) {
        *left_cm_s = v_cm_s - omega_rad_s * half_b;
    }
    if (right_cm_s != 0) {
        *right_cm_s = v_cm_s + omega_rad_s * half_b;
    }
}

/* --------------------------------------------------------------------------
 * 单侧斜率限幅
 * ------------------------------------------------------------------------ */
static float limit_one(float target, float *state, float step)
{
    float delta;
    float out;

    target = BOARD_CLAMP(target, -SPEED_MAX_CM_S, SPEED_MAX_CM_S);
    out    = *state;
    delta  = target - out;

    if (delta > step) {
        out += step;
    } else if (delta < -step) {
        out -= step;
    } else {
        out = target;
    }

    *state = out;
    return out;
}

void Chassis_Apply(float v_cm_s, float omega_rad_s, float dt,
                   float *left_cm_s, float *right_cm_s)
{
    float left;
    float right;
    float step;

    if (dt <= 0.0f) {
        dt = CONTROL_DT_S;
    }
    step = ACC_LIMIT_CM_S2 * dt;

    Chassis_Split(v_cm_s, omega_rad_s, &left, &right);

    left  = limit_one(left,  &s_lim_l, step);
    right = limit_one(right, &s_lim_r, step);

    if (left_cm_s != 0) {
        *left_cm_s = left;
    }
    if (right_cm_s != 0) {
        *right_cm_s = right;
    }
}

void Chassis_GetLimited(float *left_cm_s, float *right_cm_s)
{
    if (left_cm_s != 0) {
        *left_cm_s = s_lim_l;
    }
    if (right_cm_s != 0) {
        *right_cm_s = s_lim_r;
    }
}
