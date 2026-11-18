/**
  ******************************************************************************
  * @file    board_io.h
  * @brief   板载人机接口：按键、状态灯、蜂鸣器、电池电压
  *
  *          发车键 SW3 = PE7（低电平按下，软件消抖 20 ms）
  *          备用键 SW8/SW5/SW9 = PE8/PE9/PE10
  *          状态灯 LED3~LED6 = PD8 / PD10 / PD12 / PD14
  *          蜂鸣器 = PE13 / TIM1_CH3，约 2.7 kHz
  *          电池   = PA3 / ADC1_IN3（电阻分压）
  ******************************************************************************
  */
#ifndef _BOARD_IO_H_
#define _BOARD_IO_H_

#include <stdint.h>

typedef enum
{
    BOARD_KEY_START = 0,   /* SW3 / PE7：发车 */
    BOARD_KEY_AUX1,        /* SW8 / PE8 */
    BOARD_KEY_AUX2,        /* SW5 / PE9 */
    BOARD_KEY_AUX3,        /* SW9 / PE10 */
    BOARD_KEY_COUNT
} BoardKey;

typedef enum
{
    BOARD_LED_1 = 0,       /* PD8  ：心跳/就绪 */
    BOARD_LED_2,           /* PD10 ：视觉有效 */
    BOARD_LED_3,           /* PD12 ：行驶中 */
    BOARD_LED_4,           /* PD14 ：故障 */
    BOARD_LED_COUNT
} BoardLed;

void    BoardIO_Init(void);
void    BoardIO_Poll(void);

/** 返回"按下沿"事件（读后自动清除），主循环用 */
uint8_t BoardIO_WasPressed(BoardKey key);
/** 当前是否按住 */
uint8_t BoardIO_IsDown(BoardKey key);

void    BoardIO_LedSet(BoardLed led, uint8_t on);
void    BoardIO_LedToggle(BoardLed led);

/** 响 ms 毫秒（非阻塞，BoardIO_Poll 负责关掉） */
void    BoardIO_Buzzer(uint16_t on_ms);
void    BoardIO_BuzzerOff(void);

/** 电池电压（V）；未使能 ADC 时返回 0 */
float   BoardIO_BatteryVolt(void);

#endif /* _BOARD_IO_H_ */
