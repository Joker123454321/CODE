/**
  ******************************************************************************
  * @file    board_config.h
  * @brief   校赛三轮差速车 —— 控制组统一配置（引脚 / 几何 / 单位 / 参数）
  *
  *          本文件是控制组唯一的"接线与参数真相"：
  *            1) 所有引脚、定时器、通道只在这里定义，其它模块不许再写死引脚；
  *            2) 所有需要上车标定的物理量（轮径、轮距、编码器线数、PID）都在这里；
  *            3) 单位约定：速度 cm/s，角速度 rad/s，角度 rad（逆时针为正），
  *               里程 cm；PWM 量程 0~10000（库的 PWM_MAX，10000 = 100%）。
  *
  *          **全部外设都以 NEUQ_lib 的封装为准**：
  *            NEUQ_PWM      → 电机两路 PWM、蜂鸣器
  *            NEUQ_Encoder  → 左右正交编码器（TIM4 / TIM3）
  *            NEUQ_GPIO     → 电机方向脚、按键、状态灯
  *            NEUQ_ADC      → 电池电压采样
  *            NEUQ_PIT      → TIM6 10 ms 节拍
  *            NEUQ_Uart     → 视觉（UART_3）与无线（UART_0）
  *            NEUQ_PID      → 外环 pid_value / 内环 IncrementalPID
  *            NEUQ_MPU6050  → 航向（I2C3）
  *          库没有的部分（速度/里程换算、差速解算、赛道状态机、协议解析）才自己写。
  *
  *          引脚依据：校赛控制组_引脚配置与模块分工.pdf（2025-10-24 原理图）
  *            ├─ 左电机  PWM: TIM8_CH1 / PC6      DIR: PC7
  *            ├─ 右电机  PWM: TIM2_CH3 / PA2      DIR: PA5
  *            ├─ 左编码器 TIM4  A/B: PB6(CH1)/PB7(CH2)
  *            ├─ 右编码器 TIM3  A/B: PA6(CH1)/PA7(CH2)
  *            ├─ 视觉香橙派 UART4  PC10(TX)/PC11(RX)   -> NEUQ UART_3
  *            ├─ 无线/VOFA  USART1 PA9(TX)/PA10(RX)     -> NEUQ UART_0
  *            ├─ MPU6050    I2C3   PA8(SCL)/PC9(SDA)    -> Hard_IIC_3
  *            ├─ 发车键/备用键 PE7 / PE8,PE9,PE10（低电平按下）
  *            ├─ 蜂鸣器 TIM1_CH3 / PE13（QMB-09B-03，约 2.7 kHz）
  *            ├─ 状态灯 PD8 / PD10 / PD12 / PD14
  *            └─ 电池采样 ADC1_IN3 / PA3（分压）
  *
  * @note    方向约定
  *            - 轮速：向前为正。
  *            - 角速度 ω：左转（逆时针）为正 ⇒ 左轮 = v - Bω/2，右轮 = v + Bω/2。
  *            - 视觉偏差 dev：**车相对赛道中心偏左时为正**，即 dev>0 需要向右打舵。
  *              若图像组给的是反向，把 VISION_DEV_INVERT 改成 1 即可，不用改算法。
  ******************************************************************************
  */
#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

/* 引脚/通道全部使用库的枚举，所以这里直接引库的汇总头 */
#include "NEUQ_lib_include.h"

/*==============================================================================
 * 0. 版本
 *============================================================================*/
#define BOARD_FW_VERSION            "ctrl-1.1-library"

/*==============================================================================
 * 1. 电机：PWM + 方向（全部走 NEUQ_PWM / NEUQ_GPIO）
 *============================================================================*/
#define MOTOR_L_PWM_PORT            TIM8_CH1_PC6   /* PWM_Ports_ENUM */
#define MOTOR_R_PWM_PORT            TIM2_CH3_PA2

#define MOTOR_L_DIR_PORT            PC7            /* GPIO_Ports_ENUM */
#define MOTOR_R_DIR_PORT            PA5

/** 逻辑"前进"时方向脚输出什么电平（GPIO_High / GPIO_Low）。
  * 首次上电架空试转，若某轮前进方向反了，把该侧的宏对调即可。 */
#define MOTOR_L_DIR_FWD             GPIO_High
#define MOTOR_R_DIR_FWD             GPIO_High

/** PWM 频率：**8.4 kHz**，这是 NEUQ_PWM_init 能给出的最高常用频率。
  * 原因：库的预分频 = 84 MHz /(freq × PWM_MAX(10000))，TIM1/TIM8 再 ×2-1：
  *   freq = 8400 → 84e6/(8400*10000) = 1 ⇒ TIM2 预分频 0、TIM8 预分频 1，
  *   两边都是 84 MHz 计数 / 10000 = 8.4 kHz，正确。
  *   freq = 20000 → 整除得 0，再减 1 变成 0xFFFF，实际输出只剩几 Hz，**不能用**。
  * 8.4 kHz 对直流电机完全够用（略有人耳可闻的啸叫，属正常）。 */
#define MOTOR_PWM_FREQ_HZ           8400U
#define PWM_OUT_MAX                 10000      /* 库的 PWM_MAX，10000 = 100% */
#define PWM_OUT_MIN                (-10000)

/** 电机启动死区补偿：|占空比| 不为 0 时至少给这么多，避免"给了 PWM 不动"。
  * 各车不同，用 VOFA 看占空比与轮速曲线标定；填 0 表示不补偿。 */
#define MOTOR_DEADZONE              260

/*==============================================================================
 * 2. 编码器：NEUQ_Encoder（TIM4 左 / TIM3 右，库内部已配四倍频 TI12）
 *============================================================================*/
#define ENC_L_DEV                   Encoder_TIM4
#define ENC_L_PHASE_A               Encoder_TIM4_PhaseA_PB6
#define ENC_L_PHASE_B               Encoder_TIM4_PhaseB_PB7

#define ENC_R_DEV                   Encoder_TIM3
#define ENC_R_PHASE_A               Encoder_TIM3_PhaseA_PA6
#define ENC_R_PHASE_B               Encoder_TIM3_PhaseB_PA7

/** 计数方向修正：若某轮"车往前推"读数却在减小，把对应 SIGN 改成 -1 */
#define ENC_L_SIGN                  (+1)
#define ENC_R_SIGN                  (+1)

/*==============================================================================
 * 3. 底盘几何与标定量（必须实测标定，否则里程与速度全部成比例偏）
 *============================================================================*/
/** 轮胎直径（cm）—— 赛规不允许改轮径，用卡尺量外径 */
#define WHEEL_DIAMETER_CM           6.5f
/** 两驱动轮轮距（cm）—— 差速解算用，量两轮接地点中心距 */
#define TRACK_WIDTH_CM              15.0f

/** 每转编码器计数（含四倍频与减速比）= 线数 × 4 × 减速比。
  * 默认按 13 线 × 4 × 30:1 = 1560；实测方法见 README：
  * 架空后手转轮子整 10 圈，读累计计数，除以 10 即为该值。 */
#define ENC_L_COUNTS_PER_REV        1560.0f
#define ENC_R_COUNTS_PER_REV        1560.0f

/*==============================================================================
 * 4. 控制节拍
 *============================================================================*/
#define CONTROL_FREQ_HZ             100U
#define CONTROL_DT_S                0.01f        /* NEUQ_PIT TIM6 10 ms，内环周期 */

/*==============================================================================
 * 5. 速度规划（cm/s）—— 先低速跑通再逐级提速
 *============================================================================*/
#define SPEED_MAX_CM_S              180.0f       /* 安全限速 */
#define SPEED_STOP_CM_S             1.5f         /* 判停阈值 */
#define V_LINE_CM_S                 110.0f       /* 直道巡线基础速度 */
#define V_CURVE_CM_S                75.0f        /* 大偏差/连续弯降速 */
#define V_CROSS_CM_S                90.0f        /* 十字通过速度 */
#define V_RING_CM_S                 70.0f        /* 环岛通过速度 */
#define V_GAP_CM_S                  120.0f       /* 160 cm 断路通过速度 */
#define V_RECOVER_CM_S              60.0f        /* 丢线/重新找线时的速度 */
#define ACC_LIMIT_CM_S2             380.0f       /* 目标轮速斜率限幅 */
#define OMEGA_MAX_RAD_S             6.0f         /* 角速度限幅 */
#define LAT_ACC_MAX_CM_S2           320.0f       /* 侧向加速度限幅：ω_max = a/v */

/*==============================================================================
 * 6. 串级 PID 参数
 *    外环（方向环）：视觉偏差 -> 目标角速度 ω   —— line_follow.c，用库的 pid_value
 *    内环（轮速环）：目标轮速 -> PWM 增量      —— wheel_speed.c，用库的 IncrementalPID
 *
 *    ⚠ 库里的 pid_value 是"逐拍"口径，不是"每秒"口径：
 *        Ek_total += Ek        （不乘 dt）
 *        D = Kd × (Ek - Ek1)   （不除 dt）
 *      10 ms 节拍下换算：Ki_库 = Ki_每秒 × 0.01，Kd_库 = Kd_每秒 × 100。
 *      下面数值已是库的口径，注释标了等效"每秒"值。
 *============================================================================*/
/* --- 外环（库 pid_value） --- */
#define LINE_KP_BASE                2.6f         /* 归一化偏差 → rad/s */
#define LINE_KI_BASE                0.009f       /* ≙ 每秒 0.9 */
#define LINE_KD_BASE                6.0f         /* ≙ 每秒 0.06 */
#define LINE_V_REF_CM_S             100.0f       /* 增益按速度归一的参考速度 */
#define LINE_GAIN_MIN_SCALE         0.6f         /* 速度增益调度的上下限 */
#define LINE_GAIN_MAX_SCALE         1.6f
#define LINE_I_OUT_MAX              0.30f        /* 积分项输出限幅 rad/s（换算成库的 Max_I） */
#define LINE_DEV_LPF_ALPHA          0.55f        /* 偏差一阶低通（库的 D 没有滤波，靠它压噪声） */
#define LINE_CURVE_DEV               45.0f       /* |dev| 超过该值认为在弯里，降速 */

/* --- 内环（库 IncrementalPID） --- */
#define WHEEL_KP                    8.0f         /* Δpwm = Kp*Δe + Ki*e */
#define WHEEL_KI                    2.5f
#define WHEEL_KD                    0.0f
#define WHEEL_DELTA_MAX             1200.0f      /* 单拍 PWM 增量限幅 */
#define WHEEL_SPEED_LPF_ALPHA       0.55f        /* 轮速反馈一阶低通 */
#define WHEEL_LOOP_ENABLE           1            /* 0 = 开环调电机方向/占空比 */

/* --- 直行保航向（断路/十字用，同样走库 pid_value）：yaw 误差 → ω --- */
#define YAW_KP                      2.4f         /* rad/s per rad */
#define YAW_KD                      35.0f        /* ≙ 每秒 0.35，等效"角速度阻尼" */
#define YAW_OMEGA_MAX               3.5f

/*==============================================================================
 * 7. 视觉偏差输入（UART4 / 香橙派）
 *============================================================================*/
#define VISION_DEV_MAX              100.0f       /* 协议约定归一化偏差 ±100 */
#define VISION_DEV_INVERT           0            /* 1 = 取反（图像组给反了才改） */
#define VISION_TIMEOUT_MS           120          /* 超过该时间没收到帧视为丢线 */
#define VISION_HOLD_MS              180          /* 丢线后沿用上一帧偏差的最长时间 */
#define VISION_LOST_STOP_MS         1200         /* 丢线超过该时间：停车报警 */

/*==============================================================================
 * 8. 赛道与元素参数（依据赛题说明初版）
 *============================================================================*/
#define TRACK_WIDTH_PVC_CM          45.0f        /* PVC 赛道宽度，仅供换算参考 */
#define LAP_LENGTH_CM               5506.0f      /* 赛道中心总长 55.061 m */

/* --- 十字 --- */
#define CROSS_CONFIRM_MS            40           /* 视觉十字标志持续多久才认可 */
#define CROSS_MIN_CM                45.0f        /* 通过最小里程：不许提前退出 */
#define CROSS_MAX_CM                130.0f       /* 超过该里程无论是否看到线都退出 */
#define CROSS_RELOCK_MS             40           /* 重新看到正常赛道的确认时间 */

/* --- 环岛 --- */
#define RING_CONFIRM_MS             40
#define RING_RADIUS_CM              55.0f        /* 环岛中心线半径（估），仅兜底用 */
#define RING_ENTER_CM               18.0f        /* 进环段：强制入弯里程 */
#define RING_EXIT_YAW_DEG           320.0f       /* 环内累计转角，达到后允许出环 */
#define RING_EXIT_CM                22.0f        /* 出环段：回正里程 */
#define RING_MAX_TIME_MS            12000U       /* 环内超时保护 */

/* --- 160 cm 断路（固定在右下角直道） --- */
#define GAP_LENGTH_CM               160.0f       /* 官方断路长度 */
#define GAP_CONFIRM_MS              30
#define GAP_TRAVEL_LEN_CM           192.0f       /* 160 + 32 余量，跑完这段才算过完 */
#define GAP_MAX_CM                  240.0f       /* 断路内最大允许里程，超了报警停车 */
#define GAP_RELOCK_MS               30           /* 重新看到赛道确认时间 */
#define GAP_BY_ODOM_ENABLE          0            /* 1 = 用里程触发断路（视觉缺该标志时的兜底） */
#define GAP_ODOM_START_CM           0.0f         /* 配 GAP_BY_ODOM_ENABLE：从起点算的触发里程 */

/* --- 元素重入间隔：两个元素之间至少跑这么多，防止同一次抖动重复触发 --- */
#define ELEMENT_MIN_GAP_CM          40.0f

/*==============================================================================
 * 9. 通信（NEUQ_Uart）
 *============================================================================*/
#define COMM_VISION_UART            UART_3       /* UART4：香橙派 */
#define COMM_DEBUG_UART             UART_0       /* USART1：无线 / VOFA */
#define COMM_VISION_BAUD            UART_BAUD_115200
#define COMM_DEBUG_BAUD             UART_BAUD_115200
#define COMM_RX_BUF_SIZE            256U

/** 调试波形（VOFA+ JustFloat）发送周期与通道数 */
#define DEBUG_STREAM_PERIOD_MS      50U
#define DEBUG_STREAM_CHANNELS       8

/*==============================================================================
 * 10. 按键 / 灯 / 蜂鸣器 / 电池（NEUQ_GPIO / NEUQ_PWM / NEUQ_ADC）
 *============================================================================*/
#define KEY_START_PORT              PE7          /* SW3，低电平按下 */
#define KEY_AUX1_PORT               PE8          /* SW8，本工程作急停 */
#define KEY_AUX2_PORT               PE9          /* SW5 */
#define KEY_AUX3_PORT               PE10         /* SW9 */
#define KEY_PRESSED_LEVEL           GPIO_Low
#define KEY_DEBOUNCE_MS             20U

#define LED_1_PORT                  PD8          /* 心跳/就绪 */
#define LED_2_PORT                  PD10         /* 视觉在线 */
#define LED_3_PORT                  PD12         /* 行驶中 */
#define LED_4_PORT                  PD14         /* 故障 */
#define LED_ON_LEVEL                GPIO_High
#define LED_OFF_LEVEL               GPIO_Low

#define BUZZER_PWM_PORT             TIM1_CH3_PE13
#define BUZZER_FREQ_HZ              2700U

#define BATTERY_ADC_CH              ADC1_CH3_PA3 /* PA3 / ADC1_IN3 */
#define BATTERY_ADC_FULL_SCALE      4095.0f      /* 12 bit */
#define BATTERY_DIV_RATIO           11.0f        /* 分压比 = (R上+R下)/R下，按实物核对 */
#define BATTERY_ADC_VREF            3.3f
#define BATTERY_LOW_V               6.8f         /* 低压报警阈值 */
#define BATTERY_CHECK_ENABLE        1

/*==============================================================================
 * 11. 运行策略开关
 *============================================================================*/
#define RACE_REQUIRE_VISION         1   /* 1 = 没有视觉数据不允许发车（纯底盘调车时改 0） */
#define RACE_RUN_SECOND_LAP         0   /* 1 = 第一圈结束后继续跑第二圈（盲盒/数独用） */
#define RACE_STOP_DWELL_MS          500U /* 停稳确认时间 */

/*==============================================================================
 * 12. 小工具
 *============================================================================*/
#define BOARD_PI_F                  3.1415926f
#define BOARD_TWO_PI_F              6.2831853f
#define BOARD_DEG2RAD               (BOARD_PI_F / 180.0f)
#define BOARD_RAD2DEG               (180.0f / BOARD_PI_F)

/** 限幅 / 取符号 */
#define BOARD_CLAMP(x, lo, hi)      (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))
#define BOARD_CLAMPF(x, lim)        BOARD_CLAMP((x), -(lim), (lim))
#define BOARD_SIGN(x)               (((x) > 0.0f) ? 1.0f : (((x) < 0.0f) ? -1.0f : 0.0f))
#define BOARD_ABS(x)                (((x) >= 0.0f) ? (x) : -(x))

#endif /* _BOARD_CONFIG_H_ */
