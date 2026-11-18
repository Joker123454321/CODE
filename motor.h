/**
  ******************************************************************************
  * @file    motor.h
  * @brief   左右电机驱动（PWM + 方向脚）
  *
  *          逻辑量程 0~10000（10000 = 100%），有符号：正 = 前进，负 = 后退。
  *          与 NEUQ 库约定一致（库的 PWM_MAX 也是 10000），但本模块自己初始化
  *          定时器，原因是库的 NEUQ_PWM_init() 把"预分频"和"频率"绑在一起算，
  *          在 20 kHz 这种常用频率下会算出 0 甚至溢出，这里按定时器实际时钟
  *          重新计算，保证 20 kHz、CCR 满量程。
  ******************************************************************************
  */
#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

/** 初始化两路 PWM + 两个方向脚（上电后先输出 0） */
void Motor_Init(void);

/** 同时设置左右（有符号，正 = 前进） */
void Motor_Set(int16_t left, int16_t right);

/** 单侧设置 */
void Motor_SetLeft(int16_t left);
void Motor_SetRight(int16_t right);

/** 两轮输出 0（滑行，不主动刹车） */
void Motor_Stop(void);

/** 读取最近一次下发的占空比（调试用） */
int16_t Motor_GetLeftCmd(void);
int16_t Motor_GetRightCmd(void);

#endif /* _MOTOR_H_ */
