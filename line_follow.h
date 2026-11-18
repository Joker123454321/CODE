/**
  ******************************************************************************
  * @file    line_follow.h
  * @brief   巡线外环（串级 PID 的外环）：视觉偏差 → 目标角速度 ω
  *
  *          串级结构总览：
  *            视觉偏差 dev  ──[本模块 PID]──►  ω (rad/s)
  *                                              │
  *                              chassis.c 差速解算 (v, ω) → (vL, vR)
  *                                              │
  *                             wheel_speed.c 轮速 PID（内环，10 ms）
  *                                              │
  *                                         电机 PWM
  *
  *          为什么外环用"位置式 PID + 速度增益调度"：
  *            - 偏差量纲是图像组给的归一化值，随速度提高同样的横向误差需要更大 ω，
  *              所以 Kp/Ki/Kd 都乘 v/v_ref；
  *            - ω 还要受侧向加速度限制 ω_max = a_max / v，高速时不许猛打舵。
  ******************************************************************************
  */
#ifndef _LINE_FOLLOW_H_
#define _LINE_FOLLOW_H_

#include <stdint.h>

/** 初始化 / 复位（复位会清积分与微分状态，切换控制模式时必须调用） */
void LineFeed_Init(void);
void LineFeed_Reset(void);

/** 单拍更新
  *   dev     : 归一化视觉偏差 -100~100，**车偏左为正**（需要向右打舵）
  *   v_cm_s  : 当前规划车速，用于增益调度与 ω 限幅
  *   dt      : 控制周期（s）
  *   返回    : 目标角速度 ω (rad/s)，左转为正
  */
float LineFeed_Update(float dev, float v_cm_s, float dt);

/** 带增益缩放的版本（环岛内用小增益，避免绕着环画龙） */
float LineFeed_UpdateScaled(float dev, float v_cm_s, float dt, float gain_scale);

/** 直行保航向（十字 / 160 cm 断路用）
  *   yaw_err_rad : 目标航向 - 当前航向（已归一化到 ±π）
  *   yaw_rate    : 当前角速度（rad/s），用作阻尼
  *   返回        : 目标角速度 ω (rad/s)
  */
float LineFeed_YawHold(float yaw_err_rad, float yaw_rate, float dt);

/** 视觉偏差取反开关（图像组给的方向反了才用） */
void  LineFeed_SetDevInvert(uint8_t invert);

/** 外环增益倍数（现场调参，默认 1.0/1.0/1.0） */
void  LineFeed_SetGainScale(float kp_scale, float ki_scale, float kd_scale);

/** 调试：滤波后的偏差、最近一次 ω、PID 三相输出 */
float LineFeed_GetDevFiltered(void);
float LineFeed_GetOmega(void);
void  LineFeed_GetTerms(float *p_term, float *i_term, float *d_term);

#endif /* _LINE_FOLLOW_H_ */
