/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   差速解算 + 目标加速度限幅（串级 PID 的"分配层"）
  *
  *          外环输出的是车速 v(cm/s) 与角速度 ω(rad/s)，内环要的是两个轮速：
  *             左轮 = v - B·ω/2
  *             右轮 = v + B·ω/2      （B = 轮距 cm，ω 左转为正）
  *          这正是引脚分工文档 04 页给出的公式。
  *
  *          限幅放在这里而不是放在内环，是为了让"目标"本身就是可执行的：
  *          内环只管跟踪，不做规划，两个环的职责不混。
  ******************************************************************************
  */
#ifndef _CHASSIS_H_
#define _CHASSIS_H_

/** 清空斜率限幅状态（停车 / 发车前调用） */
void Chassis_Init(void);

/** 纯解算：车速 + 角速度 → 左右轮速目标（cm/s），不带任何限幅 */
void Chassis_Split(float v_cm_s, float omega_rad_s,
                   float *left_cm_s, float *right_cm_s);

/** 解算 + 加速度限幅：把 v/ω 变成真正下发的一对轮速目标。
  * dt 传 CONTROL_DT_S。 */
void Chassis_Apply(float v_cm_s, float omega_rad_s, float dt,
                   float *left_cm_s, float *right_cm_s);

/** 读取限幅器内部状态（调试用） */
void Chassis_GetLimited(float *left_cm_s, float *right_cm_s);

#endif /* _CHASSIS_H_ */
