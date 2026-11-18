/**
  ******************************************************************************
  * @file    control.c
  * @brief   顶层调度实现
  *
  *          初始化顺序是有讲究的：
  *            1) NEUQ_Debug_init()  先有 printf，库里的驱动才敢用 printf 调试
  *            2) NEUQ_Delay_init()  库里的延时（IMU 初始化要用）
  *            3) 串口 / IO / 电机 / 编码器  底层外设
  *            4) 内环 / 外环 / 报警 / IMU / 状态机
  *            5) NEUQ_PIT_init_ms() 最后开 10 ms 中断，避免中断里访问还没初始化好的外设
  ******************************************************************************
  */
#include <stdio.h>

#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "alarm.h"
#include "board_io.h"
#include "chassis.h"
#include "comm.h"
#include "control.h"
#include "encoder.h"
#include "imu.h"
#include "line_follow.h"
#include "motor.h"
#include "race.h"
#include "wheel_speed.h"

static volatile uint32_t s_tick_count = 0;
static volatile uint8_t  s_tick_flag = 0;
static uint32_t s_debug_last_ms = 0;

/* 现场可调的三个外环倍数 */
static float s_kp_scale = 1.0f;
static float s_ki_scale = 1.0f;
static float s_kd_scale = 1.0f;

/* 堵转检测 */
static uint16_t s_stall_cnt_l = 0;
static uint16_t s_stall_cnt_r = 0;

/* ==========================================================================
 * 健康检查：超速 / 堵转 / 低压
 * ======================================================================== */
static void control_health_check(void)
{
    float sl;
    float sr;
    float tl;
    float tr;
    float pwml;
    float pwmr;

    sl = Encoder_GetSpeedL();
    sr = Encoder_GetSpeedR();
    WheelSpeed_GetTargets(&tl, &tr);
    pwml = WheelSpeed_GetPwmL();
    pwmr = WheelSpeed_GetPwmR();

    if ((BOARD_ABS(sl) > SPEED_MAX_CM_S * 1.2f) ||
        (BOARD_ABS(sr) > SPEED_MAX_CM_S * 1.2f)) {
        Alarm_Raise(ALARM_SPEED_OVER);
    }

    /* 给了很大 PWM、目标也不为 0，但轮子基本不动 → 堵转/断线 */
    if ((BOARD_ABS(tl) > 10.0f) && (BOARD_ABS(pwml) > 6000.0f) && (BOARD_ABS(sl) < 2.0f)) {
        if (s_stall_cnt_l < 0xFFFFu) {
            s_stall_cnt_l++;
        }
    } else {
        s_stall_cnt_l = 0;
    }

    if ((BOARD_ABS(tr) > 10.0f) && (BOARD_ABS(pwmr) > 6000.0f) && (BOARD_ABS(sr) < 2.0f)) {
        if (s_stall_cnt_r < 0xFFFFu) {
            s_stall_cnt_r++;
        }
    } else {
        s_stall_cnt_r = 0;
    }

    if ((s_stall_cnt_l > 50u) || (s_stall_cnt_r > 50u)) {
        Alarm_Raise(ALARM_MOTOR_STALL);
    }

#if (BATTERY_CHECK_ENABLE == 1)
    {
        float vbat = BoardIO_BatteryVolt();
        if ((vbat > 1.0f) && (vbat < BATTERY_LOW_V)) {
            Alarm_Raise(ALARM_BATTERY_LOW);
        }
    }
#endif
}

/* ==========================================================================
 * 无线调试：调参 / 状态 / 复位
 * ======================================================================== */
static void control_handle_serial(void)
{
    char  key;
    float value;
    char  cmd;
    char  buf[112];

    while (Comm_TakeTune(&key, &value)) {
        switch (key) {
        case 'v':
            Race_SetBaseSpeed(value);
            break;
        case 'p':
            s_kp_scale = value;
            LineFeed_SetGainScale(s_kp_scale, s_ki_scale, s_kd_scale);
            break;
        case 'i':
            s_ki_scale = value;
            LineFeed_SetGainScale(s_kp_scale, s_ki_scale, s_kd_scale);
            break;
        case 'd':
            s_kd_scale = value;
            LineFeed_SetGainScale(s_kp_scale, s_ki_scale, s_kd_scale);
            break;
        default:
            break;
        }
    }

    cmd = Comm_TakeCommand();
    if (cmd == 'S') {
        (void)snprintf(buf, sizeof(buf),
                       "ST,%s,L%d,D%d,T%d,V%d,A%s\r\n",
                       Race_StateName(Race_GetState()),
                       (int)Race_GetLap(),
                       (int)Race_GetDistanceCm(),
                       (int)Race_GetLapTimeMs(),
                       (int)Comm_GetVision()->frames,
                       Alarm_Name(Alarm_GetCode()));
        Comm_SendText(buf);
    } else if (cmd == 'R') {
        Alarm_Clear();
        Race_Init();
        Chassis_Init();
        WheelSpeed_Reset();
        BoardIO_Buzzer(60);
    } else {
        /* 没有命令 */
    }
}

/* ==========================================================================
 * VOFA+ 波形：8 通道 JustFloat
 *   ch0/1 目标轮速 / 实测轮速（左）
 *   ch2/3 目标轮速 / 实测轮速（右）
 *   ch4   视觉偏差    ch5 外环 ω
 *   ch6   航向(rad)   ch7 状态机状态
 * ======================================================================== */
static void control_debug_stream(void)
{
    uint32_t now;
    float    ch[DEBUG_STREAM_CHANNELS];
    float    tl;
    float    tr;

    now = HAL_GetTick();
    if ((now - s_debug_last_ms) < DEBUG_STREAM_PERIOD_MS) {
        return;
    }
    s_debug_last_ms = now;

    WheelSpeed_GetTargets(&tl, &tr);

    ch[0] = tl;
    ch[1] = Encoder_GetSpeedL();
    ch[2] = tr;
    ch[3] = Encoder_GetSpeedR();
    ch[4] = Comm_GetVision()->dev;
    ch[5] = LineFeed_GetOmega();
    ch[6] = IMU_GetYawRad();
    ch[7] = (float)Race_GetState();

    Comm_SendJustFloat(ch, DEBUG_STREAM_CHANNELS);
}

/* ==========================================================================
 * 对外接口
 * ======================================================================== */
void Control_Init(void)
{
    NEUQ_Debug_init();
    NEUQ_Delay_init();

    Comm_Init();

    BoardIO_Init();
    Motor_Init();
    Encoder_Init();

    WheelSpeed_Init();
    Chassis_Init();
    LineFeed_Init();
    Alarm_Init();

    IMU_Init();          /* 没有陀螺也不影响发车，race.c 会退化成轮速差航向 */

    Race_Init();

    BoardIO_Buzzer(80);

    /* 最后开节拍中断：10 ms，100 Hz */
    NEUQ_PIT_init_ms(PIT_CH1, 10);
}

void Control_TickIsr(void)
{
    Encoder_Update(CONTROL_DT_S);   /* 先测速 */
    WheelSpeed_Update();            /* 再跑内环，用本拍最新反馈 */

    s_tick_count++;
    s_tick_flag = 1;
}

uint8_t Control_TakeTick(void)
{
    if (s_tick_flag == 0u) {
        return 0;
    }
    s_tick_flag = 0;
    return 1;
}

uint32_t Control_GetTickCount(void)
{
    return s_tick_count;
}

void Control_Loop(void)
{
    Comm_Poll();        /* 串口解析：视觉帧 / 无线命令 */
    BoardIO_Poll();     /* 按键消抖、蜂鸣器计时、电池采样 */
    IMU_Poll();         /* 陀螺读取 + 航向积分（内部 200 Hz 限速） */
    Alarm_Poll();       /* 故障闪烁与蜂鸣 */
    control_handle_serial();

    if (Control_TakeTick()) {
        Race_Update(CONTROL_DT_S);   /* 状态机推进 + 下发轮速目标 */
        control_health_check();
        control_debug_stream();
    }
}
