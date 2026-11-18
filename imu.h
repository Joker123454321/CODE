/**
  ******************************************************************************
  * @file    imu.h
  * @brief   航向估计（MPU6050 陀螺 Z 轴积分 + 静止零偏自校准）
  *
  *          为什么要它：160 cm 断路那段没有白色赛道，视觉完全失效，
  *          只能靠"入段时的航向"保直行；十字路口也是同理。
  *          没有陀螺时，race.c 会退化用轮速差做航向估计（也能用，只是会漂）。
  *
  *          约定：yaw 逆时针为正（左转为正），单位 rad。
  ******************************************************************************
  */
#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>

/** 陀螺 Z 轴方向修正：若"原地左转"读数为负，改成 -1.0f */
#define IMU_RATE_SIGN       (+1.0f)
/** 零偏标定的静止采样次数 */
#define IMU_CALIB_SAMPLES   150u

/** 初始化（含静止零偏标定，约 0.3 s；失败时 IMU_IsReady() 返回 0） */
void IMU_Init(void);

/** 主循环轮询：内部限速 200 Hz，读取陀螺并积分航向 */
void IMU_Poll(void);

uint8_t IMU_IsReady(void);
float   IMU_GetYawRad(void);       /* 航向角，逆时针为正 */
float   IMU_GetYawRate(void);      /* 角速度 rad/s，逆时针为正 */
void    IMU_SetYawRad(float rad);  /* 强制设置航向（入弯/入断路时对齐用） */
/** 请求重新标定零偏（车必须静止，下一次 IMU_Poll 生效） */
void    IMU_RequestBiasCalib(void);

#endif /* _IMU_H_ */
