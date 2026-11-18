/**
  ******************************************************************************
  * @file    race.h
  * @brief   赛道状态机：发车 → 巡线 → 十字 → 环岛 → 160 cm 断路 → 停车
  *
  *          状态迁移（视觉标志 + 里程 + 航向三者共同判定）：
  *
  *            WAIT_START ──按发车键──► LINE
  *            LINE ──十字标志稳定40ms──► CROSS ──跑够45cm且重新看到线──► LINE
  *            LINE ──环岛标志稳定40ms──► RING(进环/沿环/出环) ──转角≥320°──► LINE
  *            LINE ──断路标志/里程──► GAP ──跑够192cm且视觉恢复──► LINE
  *            LINE ──总里程≥55.06m──► STOP ──停稳──► FINISH
  *            任意状态 ──AUX1 键/报警──► STOP
  *
  * @note    本版按用户要求"暂不做路障/数独/软件盲盒"：
  *          - 三色路障：`race.h` 未实现绕行与第二圈红障停车，位置预留 TODO，
  *            现版本按"直接通过"处理（绿障规则一致；黄障会被判罚 10 s，属已知取舍）；
  *          - 6×6 数独、软件盲盒、真正停车点均未实现，只保留 STOP_MARK 视觉标志接口。
  ******************************************************************************
  */
#ifndef _RACE_H_
#define _RACE_H_

#include <stdint.h>

typedef enum
{
    RACE_STATE_INIT = 0,
    RACE_STATE_WAIT_START,
    RACE_STATE_LINE,
    RACE_STATE_CROSS,
    RACE_STATE_RING,
    RACE_STATE_GAP,
    RACE_STATE_STOP,
    RACE_STATE_FINISH
} RaceState;

/** 初始化（清状态，进 WAIT_START） */
void Race_Init(void);

/** 100 Hz 主循环调用：状态推进 + 下发左右轮目标 */
void Race_Update(float dt);

/** 外部强制停车（报警 / 无线命令） */
void Race_RequestStop(void);

/** 在线改巡线基础速度（cm/s，弯道速度按比例跟着变） */
void Race_SetBaseSpeed(float cm_s);

RaceState    Race_GetState(void);
const char  *Race_StateName(RaceState state);
uint8_t      Race_IsRunning(void);
float        Race_GetDistanceCm(void);
float        Race_GetElementDistanceCm(void);
uint32_t     Race_GetLapTimeMs(void);
uint32_t     Race_GetStateTimeMs(void);
uint8_t      Race_GetLap(void);
float        Race_GetCmdSpeed(void);
float        Race_GetCmdOmega(void);

#endif /* _RACE_H_ */
