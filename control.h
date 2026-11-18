/**
  ******************************************************************************
  * @file    control.h
  * @brief   顶层调度：初始化顺序、10 ms 中断、主循环
  *
  *          时间分工（与引脚分工文档 04/05 页一致）：
  *            TIM6 10 ms 中断 : Encoder_Update() + WheelSpeed_Update()
  *                              —— 只做"读反馈 + 算 PWM"，快进快出
  *            主循环          : 串口解析 / 状态机 Race_Update() / IMU 读取 /
  *                              灯与按键 / 报警 / VOFA 波形
  ******************************************************************************
  */
#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <stdint.h>

/** 上电初始化：串口 → IO → 电机 → 编码器 → 内环 → 外环 → 报警 → IMU → 状态机 → 启动 10 ms 定时器 */
void Control_Init(void);

/** TIM6 中断里调用（10 ms）：测速 + 轮速闭环 */
void Control_TickIsr(void);

/** 主循环里反复调用（非阻塞） */
void Control_Loop(void);

/** 取走一次"中断节拍"标志（读后清除） */
uint8_t Control_TakeTick(void);

/** 中断累计节拍数，用来算实际控制频率 */
uint32_t Control_GetTickCount(void);

#endif /* _CONTROL_H_ */
