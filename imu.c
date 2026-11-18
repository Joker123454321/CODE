/**
  ******************************************************************************
  * @file    imu.c
  * @brief   MPU6050 航向估计实现
  *
  *          注意：I2C 读一次 14 字节是**阻塞**的（HAL 100 ms 超时），
  *          所以只在主循环里读，绝不放进 TIM6 中断。
  ******************************************************************************
  */
#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "encoder.h"
#include "imu.h"

static uint8_t  s_ready      = 0;
static uint8_t  s_calib_req  = 0;
static float    s_bias_dps   = 0.0f;   /* 陀螺零偏（°/s） */
static float    s_rate_dps   = 0.0f;
static float    s_rate_rad_s = 0.0f;
static float    s_yaw_rad    = 0.0f;
static uint32_t s_last_ms    = 0;

/* --------------------------------------------------------------------------
 * 静止零偏标定
 * ------------------------------------------------------------------------ */
static void imu_calibrate(void)
{
    float    sum;
    uint32_t i;

    sum = 0.0f;
    for (i = 0; i < IMU_CALIB_SAMPLES; i++) {
        NEUQ_MPU6050_read_data();
        sum += MPU6050_gyro_transition(mpu6050_gyro_z);
        NEUQ_Delay_ms(2);
    }
    s_bias_dps = sum / (float)IMU_CALIB_SAMPLES;
    s_yaw_rad  = 0.0f;
    s_last_ms  = HAL_GetTick();
}

void IMU_Init(void)
{
    s_ready = 0;
    s_yaw_rad = 0.0f;
    s_rate_dps = 0.0f;
    s_rate_rad_s = 0.0f;

    if (NEUQ_MPU6050_init() == 0u) {
        /* 没接陀螺也能跑：race.c 会改用轮速差估计航向 */
        s_ready = 0;
        return;
    }
    s_ready = 1;
    imu_calibrate();
}

void IMU_RequestBiasCalib(void)
{
    s_calib_req = 1;
}

void IMU_Poll(void)
{
    uint32_t now;
    uint32_t dt_ms;
    float    raw_dps;
    float    dt_s;

    if (!s_ready) {
        return;
    }

    now = HAL_GetTick();

    if (s_calib_req) {
        s_calib_req = 0;
        imu_calibrate();
        return;
    }

    if ((now - s_last_ms) < 5u) {
        return;                 /* 限到 200 Hz，省 I2C 带宽 */
    }
    dt_ms = now - s_last_ms;
    s_last_ms = now;
    dt_s = (float)dt_ms * 0.001f;

    NEUQ_MPU6050_read_data();
    raw_dps = MPU6050_gyro_transition(mpu6050_gyro_z) * IMU_RATE_SIGN;

    /* 停车静止时缓慢跟踪零偏，抑制温漂；有速度时绝不动零偏 */
    if ((BOARD_ABS(raw_dps - s_bias_dps) < 1.2f) &&
        (BOARD_ABS(Encoder_GetSpeedL()) < 2.0f) &&
        (BOARD_ABS(Encoder_GetSpeedR()) < 2.0f)) {
        s_bias_dps += 0.002f * (raw_dps - s_bias_dps);
    }

    s_rate_dps   = raw_dps - s_bias_dps;
    s_rate_rad_s = s_rate_dps * BOARD_DEG2RAD;
    s_yaw_rad   += s_rate_rad_s * dt_s;

    /* 归一化到 ±π */
    while (s_yaw_rad > BOARD_PI_F) {
        s_yaw_rad -= BOARD_TWO_PI_F;
    }
    while (s_yaw_rad < -BOARD_PI_F) {
        s_yaw_rad += BOARD_TWO_PI_F;
    }

}

uint8_t IMU_IsReady(void)
{
    return s_ready;
}

float IMU_GetYawRad(void)
{
    return s_yaw_rad;
}

float IMU_GetYawRate(void)
{
    return s_rate_rad_s;
}

void IMU_SetYawRad(float rad)
{
    s_yaw_rad = rad;
}
