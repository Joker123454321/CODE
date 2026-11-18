/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   正交编码器测速 / 里程（TIM4 左，TIM3 右，四倍频）
  *
  *          约定：向前为正，速度 cm/s，里程 cm。
  *          计数差用 int16_t 回卷相减，所以 10 ms 拍内打死也不会溢出。
  ******************************************************************************
  */
#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <stdint.h>

/** 初始化两路编码器（TIM3/TIM4 编码器模式，四倍频） */
void Encoder_Init(void);

/** 清零里程与总计数（发车前调用） */
void Encoder_ResetOdom(void);

/** 每拍调用（TIM6 中断里）：读计数差 → 轮速 / 累计里程 */
void Encoder_Update(float dt);

float Encoder_GetSpeedL(void);      /* cm/s，一阶低通后 */
float Encoder_GetSpeedR(void);
float Encoder_GetSpeedRawL(void);   /* 未滤波，标定用 */
float Encoder_GetSpeedRawR(void);
float Encoder_GetDistL(void);       /* cm */
float Encoder_GetDistR(void);
float Encoder_GetDistAvg(void);     /* (L+R)/2，元素里程用 */
int32_t Encoder_GetCntL(void);      /* 上电以来累计计数，标定线数用 */
int32_t Encoder_GetCntR(void);

#endif /* _ENCODER_H_ */
