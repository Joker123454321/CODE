/**
  ******************************************************************************
  * @file    board_io.c
  * @brief   按键 / 灯 / 蜂鸣器 / 电池采样（基于 NEUQ_GPIO + NEUQ_PWM + NEUQ_ADC）
  *
  *          全部走库封装，本文件只负责"消抖、闪烁节奏、电压换算"这些应用逻辑：
  *            NEUQ_GPIO_init/write/read/toggle   按键、状态灯、方向脚同源
  *            NEUQ_PWM_init/set_duty             蜂鸣器（TIM1_CH3 / PE13）
  *            adc_init/adc_mean_filter_convert   电池电压（ADC1_IN3 / PA3）
  ******************************************************************************
  */
#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "board_io.h"

#define KEY_SCAN_PERIOD_MS      10u
#define KEY_STABLE_CNT          2u      /* 连续 2 次(20 ms)一致才认 */
#define BATTERY_PERIOD_MS       500u

static const GPIO_Ports_ENUM s_key_ports[BOARD_KEY_COUNT] =
{
    KEY_START_PORT,
    KEY_AUX1_PORT,
    KEY_AUX2_PORT,
    KEY_AUX3_PORT,
};

static const GPIO_Ports_ENUM s_led_ports[BOARD_LED_COUNT] =
{
    LED_1_PORT,
    LED_2_PORT,
    LED_3_PORT,
    LED_4_PORT,
};

static uint8_t  s_key_down[BOARD_KEY_COUNT];
static uint8_t  s_key_stable[BOARD_KEY_COUNT];
static uint8_t  s_key_event[BOARD_KEY_COUNT];
static uint32_t s_key_last_ms = 0;

static uint32_t s_buzzer_until_ms = 0;
static uint8_t  s_buzzer_on = 0;

static float    s_battery_v = 0.0f;
static uint32_t s_battery_last_ms = 0;

/* ==========================================================================
 * 初始化
 * ======================================================================== */
void BoardIO_Init(void)
{
    uint8_t i;

    /* 按键：上拉输入，低电平按下 */
    for (i = 0; i < BOARD_KEY_COUNT; i++) {
        NEUQ_GPIO_init(s_key_ports[i], GPIO_Mode_Input_PullUp, GPIO_High);
        s_key_down[i]   = 0;
        s_key_stable[i] = 0;
        s_key_event[i]  = 0;
    }

    /* 状态灯：推挽输出，先全灭 */
    for (i = 0; i < BOARD_LED_COUNT; i++) {
        NEUQ_GPIO_init(s_led_ports[i], GPIO_Mode_Output_PushPull, LED_OFF_LEVEL);
    }

    /* 蜂鸣器：库函数初始化 TIM1_CH3，占空比先给 0 */
    NEUQ_PWM_init(BUZZER_PWM_PORT, BUZZER_FREQ_HZ, 0);

#if (BATTERY_CHECK_ENABLE == 1)
    adc_init(BATTERY_ADC_CH, ADC_12BIT);
#endif

    s_key_last_ms = HAL_GetTick();
    s_buzzer_until_ms = 0;
    s_buzzer_on = 0;
}

/* --------------------------------------------------------------------------
 * 轮询：10 ms 扫按键、管蜂鸣器、500 ms 采电池
 * ------------------------------------------------------------------------ */
void BoardIO_Poll(void)
{
    uint32_t now;
    uint8_t  i;
    uint8_t  pressed;

    now = HAL_GetTick();

    /* --- 按键消抖（读库的 GPIO） --- */
    if ((now - s_key_last_ms) >= KEY_SCAN_PERIOD_MS) {
        s_key_last_ms = now;
        for (i = 0; i < BOARD_KEY_COUNT; i++) {
            pressed = (NEUQ_GPIO_read(s_key_ports[i]) == KEY_PRESSED_LEVEL) ? 1u : 0u;

            if (pressed != s_key_down[i]) {
                s_key_down[i] = pressed;
                s_key_stable[i] = 0;
            } else if (s_key_stable[i] < KEY_STABLE_CNT) {
                s_key_stable[i]++;
                if ((s_key_stable[i] >= KEY_STABLE_CNT) && s_key_down[i]) {
                    s_key_event[i] = 1;      /* 记录按下沿 */
                }
            } else {
                /* 稳定 */
            }
        }
    }

    /* --- 蜂鸣器自动关闭 --- */
    if (s_buzzer_on && (now >= s_buzzer_until_ms)) {
        BoardIO_BuzzerOff();
    }

    /* --- 电池（库的 ADC：8 次均值滤波） --- */
#if (BATTERY_CHECK_ENABLE == 1)
    if ((now - s_battery_last_ms) >= BATTERY_PERIOD_MS) {
        uint16_t raw;
        s_battery_last_ms = now;
        raw = adc_mean_filter_convert(BATTERY_ADC_CH, 8);
        s_battery_v = ((float)raw / BATTERY_ADC_FULL_SCALE) * BATTERY_ADC_VREF * BATTERY_DIV_RATIO;
    }
#endif
}

uint8_t BoardIO_WasPressed(BoardKey key)
{
    uint8_t ev;

    if ((uint8_t)key >= (uint8_t)BOARD_KEY_COUNT) {
        return 0;
    }
    ev = s_key_event[(uint8_t)key];
    s_key_event[(uint8_t)key] = 0;
    return ev;
}

uint8_t BoardIO_IsDown(BoardKey key)
{
    if ((uint8_t)key >= (uint8_t)BOARD_KEY_COUNT) {
        return 0;
    }
    return s_key_down[(uint8_t)key];
}

void BoardIO_LedSet(BoardLed led, uint8_t on)
{
    if ((uint8_t)led >= (uint8_t)BOARD_LED_COUNT) {
        return;
    }
    NEUQ_GPIO_write(s_led_ports[(uint8_t)led],
                    (on != 0) ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

void BoardIO_LedToggle(BoardLed led)
{
    if ((uint8_t)led >= (uint8_t)BOARD_LED_COUNT) {
        return;
    }
    NEUQ_GPIO_toggle(s_led_ports[(uint8_t)led]);
}

void BoardIO_Buzzer(uint16_t on_ms)
{
    if (on_ms == 0u) {
        BoardIO_BuzzerOff();
        return;
    }
    NEUQ_PWM_set_duty(BUZZER_PWM_PORT, 5000);
    s_buzzer_on = 1;
    s_buzzer_until_ms = HAL_GetTick() + (uint32_t)on_ms;
}

void BoardIO_BuzzerOff(void)
{
    NEUQ_PWM_set_duty(BUZZER_PWM_PORT, 0);
    s_buzzer_on = 0;
}

float BoardIO_BatteryVolt(void)
{
#if (BATTERY_CHECK_ENABLE == 1)
    return s_battery_v;
#else
    return 0.0f;
#endif
}
