/**
  ******************************************************************************
  * @file    motor.c
  * @brief   左右电机驱动（基于 NEUQ_PWM + NEUQ_GPIO）
  *
  *          左：TIM8_CH1 / PC6 + DIR / PC7
  *          右：TIM2_CH3 / PA2 + DIR / PA5
  *
  *          全部走库的封装：
  *            NEUQ_PWM_init(PWM_Port, freq, duty)   初始化 PWM，duty 直接是 0~10000
  *            NEUQ_PWM_set_duty(PWM_Port, duty)     改占空比
  *            NEUQ_GPIO_init(Port, 模式, 电平)       方向脚（库会自动开 GPIO 时钟）
  *            NEUQ_GPIO_write(Port, 电平)            改方向
  *          所以本文件不再碰 HAL/寄存器，只做"有符号量 → 方向 + 占空比"的换算。
  ******************************************************************************
  */
#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "motor.h"

static int16_t s_cmd_left  = 0;
static int16_t s_cmd_right = 0;

/* --------------------------------------------------------------------------
 * 单侧输出：符号决定方向，幅值做死区补偿
 * ------------------------------------------------------------------------ */
static void motor_output(PWM_Ports_ENUM pwm_port, GPIO_Ports_ENUM dir_port,
                         GPIO_Level dir_fwd_level, int16_t cmd)
{
    int32_t duty = cmd;
    GPIO_Level level;

    if (duty > PWM_OUT_MAX) {
        duty = PWM_OUT_MAX;
    } else if (duty < PWM_OUT_MIN) {
        duty = PWM_OUT_MIN;
    }

    if (duty > 0) {
        level = dir_fwd_level;
        if (duty < MOTOR_DEADZONE) {
            duty = MOTOR_DEADZONE;
        }
    } else if (duty < 0) {
        level = (dir_fwd_level == GPIO_High) ? GPIO_Low : GPIO_High;
        duty = -duty;
        if (duty < MOTOR_DEADZONE) {
            duty = MOTOR_DEADZONE;
        }
    } else {
        level = dir_fwd_level;
        duty  = 0;
    }

    NEUQ_GPIO_write(dir_port, level);
    NEUQ_PWM_set_duty(pwm_port, (uint32_t)duty);
}

/* ==========================================================================
 * 对外接口
 * ======================================================================== */
void Motor_Init(void)
{
    /* 方向脚先给"前进"电平，再开 PWM，避免初始化瞬间乱转 */
    NEUQ_GPIO_init(MOTOR_L_DIR_PORT, GPIO_Mode_Output_PushPull, MOTOR_L_DIR_FWD);
    NEUQ_GPIO_init(MOTOR_R_DIR_PORT, GPIO_Mode_Output_PushPull, MOTOR_R_DIR_FWD);

    NEUQ_PWM_init(MOTOR_L_PWM_PORT, MOTOR_PWM_FREQ_HZ, 0);
    NEUQ_PWM_init(MOTOR_R_PWM_PORT, MOTOR_PWM_FREQ_HZ, 0);

    Motor_Stop();
}

void Motor_SetLeft(int16_t left)
{
    if (left > PWM_OUT_MAX) {
        left = PWM_OUT_MAX;
    } else if (left < PWM_OUT_MIN) {
        left = PWM_OUT_MIN;
    }
    s_cmd_left = left;
    motor_output(MOTOR_L_PWM_PORT, MOTOR_L_DIR_PORT, MOTOR_L_DIR_FWD, left);
}

void Motor_SetRight(int16_t right)
{
    if (right > PWM_OUT_MAX) {
        right = PWM_OUT_MAX;
    } else if (right < PWM_OUT_MIN) {
        right = PWM_OUT_MIN;
    }
    s_cmd_right = right;
    motor_output(MOTOR_R_PWM_PORT, MOTOR_R_DIR_PORT, MOTOR_R_DIR_FWD, right);
}

void Motor_Set(int16_t left, int16_t right)
{
    Motor_SetLeft(left);
    Motor_SetRight(right);
}

void Motor_Stop(void)
{
    Motor_Set(0, 0);
}

int16_t Motor_GetLeftCmd(void)
{
    return s_cmd_left;
}

int16_t Motor_GetRightCmd(void)
{
    return s_cmd_right;
}
