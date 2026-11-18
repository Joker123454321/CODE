/**
  ******************************************************************************
  * @file    wheel_speed.h
  * @brief   轮速闭环（串级 PID 的内环）：目标轮速 → PWM
  *
  *          分工文档要求的接口都在这里：
  *            WheelSpeed_Update()        在 TIM6 中断里跑
  *            WheelSpeed_SetTargets()    主循环把左右目标"成对"交接给中断
  *
  *          实现方式：两侧各一个增量式 PI，IncrementalPID_Update() 给出 PWM
  *          增量，本侧累加后限幅 —— 与引脚分工文档 04/05 页一致。
  ******************************************************************************
  */
#ifndef _WHEEL_SPEED_H_
#define _WHEEL_SPEED_H_

/** 初始化（写增益、清状态） */
void WheelSpeed_Init(void);

/** 主循环发布目标轮速（cm/s，前进为正）。内部用极短临界区成对写入 */
void WheelSpeed_SetTargets(float left_cm_s, float right_cm_s);

/** 读回当前目标（调试/显示用） */
void WheelSpeed_GetTargets(float *left_cm_s, float *right_cm_s);

/** 内环单拍：读反馈 → PI → 累加 → 限幅 → 输出 PWM。
  * 只允许在 TIM6 中断（100 Hz）里调用。 */
void WheelSpeed_Update(void);

/** 立刻停机：PWM 归零并复位 PI 状态（报警 / 发车前 / 停车用） */
void WheelSpeed_Stop(void);

/** 复位 PI 状态（不清目标） */
void WheelSpeed_Reset(void);

/** 在线改增益（VOFA 调参用） */
void WheelSpeed_SetGains(float kp, float ki, float kd);

/** 读取当前 PWM 输出（调试用） */
float WheelSpeed_GetPwmL(void);
float WheelSpeed_GetPwmR(void);

#endif /* _WHEEL_SPEED_H_ */
