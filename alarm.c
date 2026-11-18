/**
  ******************************************************************************
  * @file    alarm.c
  * @brief   故障报警实现
  ******************************************************************************
  */
#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "alarm.h"
#include "board_io.h"

static AlarmCode s_code = ALARM_NONE;
static uint32_t  s_raise_ms = 0;
static uint32_t  s_last_beep_ms = 0;

static const char *k_alarm_names[] =
{
    "NONE", "VISION_LOST", "GAP_TIMEOUT", "SPEED_OVER",
    "MOTOR_STALL", "BATTERY_LOW"
};

void Alarm_Init(void)
{
    s_code = ALARM_NONE;
    s_raise_ms = HAL_GetTick();
    s_last_beep_ms = 0;
    BoardIO_LedSet(BOARD_LED_4, 0);
}

void Alarm_Raise(AlarmCode code)
{
    if ((code == ALARM_NONE) || (s_code != ALARM_NONE)) {
        return;   /* 只保留第一个 */
    }
    s_code = code;
    s_raise_ms = HAL_GetTick();
    s_last_beep_ms = 0;
}

void Alarm_Clear(void)
{
    s_code = ALARM_NONE;
    BoardIO_LedSet(BOARD_LED_4, 0);
    BoardIO_BuzzerOff();
}

uint8_t Alarm_IsActive(void)
{
    return (s_code != ALARM_NONE) ? 1u : 0u;
}

AlarmCode Alarm_GetCode(void)
{
    return s_code;
}

const char *Alarm_Name(AlarmCode code)
{
    if ((uint8_t)code >= (uint8_t)ALARM_COUNT) {
        return "?";
    }
    return k_alarm_names[(uint8_t)code];
}

void Alarm_Poll(void)
{
    uint32_t now;

    now = HAL_GetTick();

    if (s_code == ALARM_NONE) {
        BoardIO_LedSet(BOARD_LED_4, 0);
        return;
    }

    /* LED4：刚报故障时 5 Hz 快闪 2 s，之后 2 Hz 慢闪；同时每秒一次短鸣 */
    if ((now - s_raise_ms) < 2000u) {
        BoardIO_LedSet(BOARD_LED_4, (uint8_t)(((now / 100u) & 1u) != 0u));
    } else {
        BoardIO_LedSet(BOARD_LED_4, (uint8_t)(((now / 250u) & 1u) != 0u));
    }
    if ((now - s_last_beep_ms) >= 1000u) {
        s_last_beep_ms = now;
        BoardIO_Buzzer(80);
    }
}
