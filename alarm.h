/**
  ******************************************************************************
  * @file    alarm.h
  * @brief   故障报警：记录首个故障码，闪烁 + 蜂鸣提示，并让状态机停车
  *
  *          设计原则：只记录**第一个**故障（后面的往往是它引起的连锁反应），
  *          故障码可以通过无线串口发出去，也可以直接看 LED4 闪烁节奏。
  ******************************************************************************
  */
#ifndef _ALARM_H_
#define _ALARM_H_

#include <stdint.h>

typedef enum
{
    ALARM_NONE = 0,
    ALARM_VISION_LOST,     /* 丢线超时 */
    ALARM_GAP_TIMEOUT,     /* 160cm 断路里跑超里程还没回赛道 */
    ALARM_SPEED_OVER,      /* 轮速超限（打滑/编码器故障） */
    ALARM_MOTOR_STALL,     /* 给足 PWM 却不动（堵转/断线） */
    ALARM_BATTERY_LOW,     /* 电池低压 */
    ALARM_COUNT
} AlarmCode;

void       Alarm_Init(void);
void       Alarm_Raise(AlarmCode code);
void       Alarm_Clear(void);
uint8_t    Alarm_IsActive(void);
AlarmCode  Alarm_GetCode(void);
const char *Alarm_Name(AlarmCode code);

/** 主循环调用：负责 LED4 闪烁与间歇蜂鸣 */
void       Alarm_Poll(void);

#endif /* _ALARM_H_ */
