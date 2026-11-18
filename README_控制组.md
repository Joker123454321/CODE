# 校赛三轮差速车 —— 控制组代码说明

> 适用平台：NEUQ STM32F407VET6 开源库（EIDE + ARMCC5 / AC5）
> 依据文档：《校赛控制组_引脚配置与模块分工》（2025-10-24 原理图）、《NEUQ智能车实验室_校赛赛题说明（初版）》
> 本版范围：**串级 PID + 电机驱动 + 编码器测速 + 轮速闭环 + 基础巡线 + 十字 + 环岛 + 160 cm 断路**
> 明确不做：三色路障、6×6 数独、软件盲盒（接口已留，见第 8 节）
>
> **设计原则：外设一律用 `NEUQ_lib` 的封装，库没提供的（速度里程换算、差速解算、赛道状态机、协议解析）才自己写。**

---

## 1. 库模块 ↔ 本工程对应关系

| 用到库的哪个模块 | 库函数 / 全局量 | 本工程谁在用 | 干什么 |
|------------------|-----------------|--------------|--------|
| `NEUQ_PWM` | `NEUQ_PWM_init / NEUQ_PWM_set_duty` | `motor.c`、`board_io.c` | 左右电机 PWM（TIM8_CH1/PC6、TIM2_CH3/PA2）、蜂鸣器（TIM1_CH3/PE13） |
| `NEUQ_Encoder` | `NEUQ_Encoder_init / Read / Clear` | `encoder.c` | TIM4(左)/TIM3(右) 正交编码器，四倍频 |
| `NEUQ_GPIO` | `NEUQ_GPIO_init / write / read / toggle` | `motor.c`（方向脚）、`board_io.c`（按键、灯） | 所有普通 IO |
| `NEUQ_ADC` | `adc_init / adc_mean_filter_convert` | `board_io.c` | 电池采样 PA3 / ADC1_IN3 |
| `NEUQ_PIT` | `NEUQ_PIT_init_ms(PIT_CH1, 10)` | `control.c` | TIM6 10 ms 控制节拍 |
| `NEUQ_Uart` | `NEUQ_Uart_init / write_* / Enable_RX_Interrupt` | `comm.c` | 视觉 UART_3、无线 UART_0 |
| `NEUQ_PID` | `pid_value` + `Unclipped_Positional_PID` | `line_follow.c`（外环、航向环） | 偏差→ω、yaw 误差→ω |
| `NEUQ_PID` | `IncrementalPID` + `IncrementalPID_Update`，全局 `stra_speed_wheel_1/2` | `wheel_speed.c`（内环） | 左右轮速 PI → PWM 增量 |
| `NEUQ_MPU6050` | `NEUQ_MPU6050_init / read_data / gyro_transition` | `imu.c` | 航向积分（I2C3） |
| `NEUQ_Debug / Delay` | `NEUQ_Debug_init / NEUQ_Delay_*` | `control.c`、`imu.c` | printf 重定向、延时 |

**库没有、由本工程实现的部分**（这些是赛题逻辑，库里本来就没有对应模块）：

| 文件 | 为什么必须自己写 |
|------|------------------|
| `encoder.c` 的速度/里程换算 | 库只给"位置"（相对 0x7FFF），没有 cm/s 和里程 |
| `chassis.c` | 差速解算 `vL = v - Bω/2`、加速度限幅 |
| `race.c` | 赛道状态机（十字/环岛/断路判据） |
| `comm.c` 的行解析 | 库只做字节收发，没有协议帧解析 |
| `line_follow.c` 的增益调度/限幅 | 库给的是定参数 PID，速度调度、ω 限幅是赛道逻辑 |
| `alarm.c` / `control.c` | 故障码、调度顺序 |

### 文件清单

| 文件 | 职责 | 关键接口 |
|------|------|----------|
| `board_config.h` | **唯一的引脚/几何/参数真相**，引脚全部用库的枚举 | — |
| `motor.c/h` | `NEUQ_PWM`+`NEUQ_GPIO`：有符号量 → 方向 + 占空比 | `Motor_Init/Motor_Set` |
| `encoder.c/h` | `NEUQ_Encoder` + 位置差分 → 轮速/里程 | `Encoder_Update(dt)/Encoder_GetSpeedL()` |
| `wheel_speed.c/h` | **内环**：库 `IncrementalPID` 两侧独立 PI | `WheelSpeed_Update()/WheelSpeed_SetTargets()` |
| `chassis.c/h` | 差速解算 + 目标加速度限幅 | `Chassis_Split/Chassis_Apply` |
| `line_follow.c/h` | **外环**：库 `pid_value`，偏差→ω；直行保航向 | `LineFeed_Update/LineFeed_YawHold` |
| `comm.c/h` | 视觉帧解析（UART_3）+ 无线调参（UART_0）+ VOFA 波形 | `Comm_Poll/Comm_GetVision` |
| `imu.c/h` | 陀螺航向积分 + 静止零偏自校准 | `IMU_GetYawRad/IMU_GetYawRate` |
| `race.c/h` | **赛道状态机**：巡线/十字/环岛/断路/停车 | `Race_Update(dt)` |
| `board_io.c/h` | `NEUQ_GPIO`+`NEUQ_PWM`+`NEUQ_ADC`：按键、灯、蜂鸣器、电池 | `BoardIO_WasPressed` |
| `alarm.c/h` | 故障码、闪烁与蜂鸣、触发停车 | `Alarm_Raise/Alarm_IsActive` |
| `control.c/h` | 顶层调度：初始化顺序、10 ms 中断、主循环 | `Control_Init/Control_TickIsr/Control_Loop` |

接入点（已改好）：
- `User/NEUQ_Main.c` → `User_Main()` 只调 `Control_Init()` + `Control_Loop()`
- `User/NEUQ_Interrupts.c` → `NEUQ_PIT_1_CALLBACK()` 跑 10 ms 轮速闭环；`TS_UART_0/3_RX_CALLBACK()` 只收字节

---

## 2. 引脚与资源（与硬件组核对用）

| 功能 | STM32 引脚 | 库的枚举写法 | 代码位置 |
|------|-----------|--------------|----------|
| 左电机 PWM | **PC6** | `TIM8_CH1_PC6`（`PWM_Ports_ENUM`） | `MOTOR_L_PWM_PORT` |
| 左电机方向 | **PC7** | `PC7`（`GPIO_Ports_ENUM`） | `MOTOR_L_DIR_PORT` |
| 右电机 PWM | **PA2** | `TIM2_CH3_PA2` | `MOTOR_R_PWM_PORT` |
| 右电机方向 | **PA5** | `PA5` | `MOTOR_R_DIR_PORT` |
| 左编码器 A/B | **PB6 / PB7** | `Encoder_TIM4_PhaseA_PB6 / PhaseB_PB7` | `ENC_L_*` |
| 右编码器 A/B | **PA6 / PA7** | `Encoder_TIM3_PhaseA_PA6 / PhaseB_PA7` | `ENC_R_*` |
| 视觉（香橙派） | **PC10 TX / PC11 RX** | `UART_3`（UART4），115200 | `COMM_VISION_UART` |
| 无线 / VOFA | **PA9 / PA10** | `UART_0`（USART1），115200 | `COMM_DEBUG_UART` |
| MPU6050 | **PA8 SCL / PC9 SDA** | `Hard_IIC_3` | `imu.c` |
| 发车键 SW3 | **PE7** | `PE7` | `KEY_START_PORT` |
| 备用键 SW8/SW5/SW9 | **PE8 / PE9 / PE10** | `PE8/PE9/PE10`（AUX1 作急停） | `KEY_AUX*_PORT` |
| 蜂鸣器 | **PE13** | `TIM1_CH3_PE13` | `BUZZER_PWM_PORT` |
| 状态灯 LED3~6 | **PD8 / PD10 / PD12 / PD14** | `PD8/PD10/PD12/PD14` | `LED_*_PORT` |
| 电池采样 | **PA3** | `ADC1_CH3_PA3`（12 bit） | `BATTERY_ADC_CH` |
| 控制节拍 | 芯片内部 | `PIT_CH1`（TIM6） | `NEUQ_PIT_init_ms` |

> **重要**：CubeMX 模板把 PA5/PA6/PA7 配给了 SPI1、PB6/PB7 配给了 I2C1，但生成的 `main.c` 里
> **没有任何 `MX_xxx_Init()` 调用**，所以运行期这几个外设并没有被初始化，库函数重新配置 GPIO 不会冲突。
> 如果以后有人把 CubeMX 生成的 `MX_SPI1_Init()` 之类加回来，会与本工程抢引脚，**必须删掉**。

---

## 3. 控制结构：串级 PID（两级都用库的 PID）

```
                    ┌──── 外环 100 Hz（主循环）───────────────────────┐
  视觉偏差 dev ──► 低通 ──► 库 pid_value + Unclipped_Positional_PID ──► ω
                    │        （增益随车速调度，ω 再受侧向加速度限幅）    │
                    └────────────────────────────────────────────────────┘
                                     │
                        chassis.c  差速解算
                        vL = v - B·ω/2 ,  vR = v + B·ω/2
                                     │
                    ┌──── 内环 100 Hz（TIM6 中断）────────────────────┐
                    │ 库 IncrementalPID_Update(stra_speed_wheel_1, …) │
                    │   左：Δpwm = Kp·Δe + Ki·e → pwm += Δpwm → 限幅  │
                    │   右：同上（两路完全独立）                       │
                    └────────────────────────────────────────────────┘
                                     │
                          motor.c → NEUQ_PWM_set_duty + 方向脚
                                     ▲
                     encoder.c ← NEUQ_Encoder_Read 位置差分
```

**为什么是两级**
- 外环管"往哪走"：偏差 → 角速度，响应慢（视觉 40~100 Hz），带宽低，允许积分。
- 内环管"走多快"：轮速 → 占空比，响应快（10 ms），把电池电压下降、地面摩擦变化、左右电机差异全部吃掉。

**外环（`line_follow.c`，库 `pid_value`）**
- 用库的全局 `yaw_angle_wheel`（方向环）和 `yaw_anglespeed_wheel`（航向环），不另造 PID 结构。
- 巧用符号：把 `目标=0`、`测量=dev` 喂给库函数，`dev>0`（车偏左、要右打舵）自动得到 `ω<0`。
- 增益随速度调度：`Kp = LINE_KP_BASE × (v/V_REF)`，限幅在 `[0.6, 1.6]` 倍；直接改库结构体的
  `Kp/Ki/Kd` 三个字段（不调用 `pid_parameters_setting`，否则会把积分一起清掉）。
- 积分限幅换算：库的 `Max_I` 限的是"误差累加和"，所以传 `Max_I = LINE_I_OUT_MAX / Ki`，
  这样积分项输出正好被限制在 `LINE_I_OUT_MAX`(rad/s)。
- ω 限幅：`ω_max = min(OMEGA_MAX_RAD_S, LAT_ACC_MAX_CM_S2 / v)`，高速时不许猛打舵。

**内环（`wheel_speed.c`，库 `IncrementalPID`）**
- 直接用库里为直道速度环准备的全局 `stra_speed_wheel_1`（左）/ `stra_speed_wheel_2`（右），
  用 `Ipid_parameters_setting()` 初始化，再把库写死的 `max_delta_pwm(6000)` 改成本车的 1200。
- 库没管、本文件补上的两点：**输出顶到限幅就丢掉同向增量**（抗饱和）、**目标为 0 且已停住时清状态**（防抖）。
- 目标由主循环"成对"写入，`__disable_irq()` 极短临界区，避免中断读到"一新一旧"的一对目标。

### ⚠ 库 PID 的"逐拍"口径（调参必读）

库的 `pid_value` 不是标准的每秒口径：

```
Ek_total += Ek            ← 不乘 dt
D 项     = Kd × (Ek-Ek1)  ← 不除 dt
```

在 10 ms 节拍下换算关系是：

| 你想要的口径 | 填进库的值 |
|--------------|-----------|
| `Ki = 0.9`（每秒） | `LINE_KI_BASE = 0.009` |
| `Kd = 0.06`（每秒） | `LINE_KD_BASE = 6.0` |
| `Kd = 0.35`（每秒，航向阻尼） | `YAW_KD = 35.0` |

`board_config.h` 里给的都是**换算后的库口径**，注释标了等效"每秒"值，照着注释理解就行。

---

## 4. 上电自检与首次联调顺序（务必按这个顺序）

1. **不要装车**：只上电，听蜂鸣器"滴"一声 → `Control_Init()` 跑通。
2. **看灯**：LED1 心跳闪（250 ms）、LED2 亮表示收到视觉帧。
3. **架空试转**：把 `board_config.h` 的 `WHEEL_LOOP_ENABLE` 临时改 0（开环），
   在 VOFA 里看 `Motor_GetLeftCmd/RightCmd`，确认：给正数 → 两轮都**向前**转。
   - 某轮反转 → 对调该侧 `MOTOR_x_DIR_FWD` 的 `GPIO_High/GPIO_Low`；
   - 编码器读数与转向相反 → 把该侧 `ENC_x_SIGN` 改成 `-1`。
4. **编码器标定**：架空，手转轮子整 10 圈，读 `Encoder_GetCntL()`，`ENC_L_COUNTS_PER_REV = 计数/10`。
5. **内环闭环**：`WHEEL_LOOP_ENABLE` 改回 1，`WHEEL_KP=0, WHEEL_KI=0.8` 起步，
   VOFA 看"目标轮速/实测轮速"两条曲线同速跟随，再逐步加 `WHEEL_KP`（先把 Ki 调到 2~3，再加 Kp）。
6. **底盘验收四组指令**：两轮同速直行、只给左轮、只给右轮、全部归零 —— 看是否跑偏、停得住。
7. **模拟视觉**：用串口助手按第 5 节协议发 `V,0,33,1` 之类的假帧，
   验证巡线方向（dev 给正，车应该往右打舵）以及十字/环岛/断路的状态切换。
8. **装摄像头**后再定增益、帧率、触发距离。

---

## 5. 与图像组的接口（**这是硬约定，改协议要双方同意**）

物理层：UART4，PC10=STM32_TX，PC11=STM32_RX，**115200 8N1**，共地。

```
V,<dev>,<flags>,<seq>\n
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `dev` | float | 归一化横向偏差，`-100.0 ~ +100.0`。**车相对赛道中心偏左为正**，即 `dev>0` 时要向右打舵。`|dev|=100` ≈ 半个赛道宽（22.5 cm） |
| `flags` | int | 位标志，见下表 |
| `seq` | int | 0~255 循环，主控只用来统计丢帧 |

`flags` 位定义：

| 位 | 值 | 名称 | 含义 |
|----|-----|------|------|
| 0 | 0x01 | VALID | 本帧有效，**必须置位**，否则整帧忽略 |
| 1 | 0x02 | CROSS | 十字路口 |
| 2 | 0x04 | RING_IN | 即将进环岛（进环方向由 `dev` 符号给出） |
| 3 | 0x08 | RING_OUT | 已到环岛出口 |
| 4 | 0x10 | GAP | 160 cm 断路（无白色赛道） |
| 5 | 0x20 | LINE | 正常赛道可信 |
| 6 | 0x40 | STOP_MARK | 停车标志 |
| 7 | 0x80 | 保留 | |

例：`V,-12.5,33,145` → 偏右 12.5%，flags=0x21（有效 + 正常赛道）。

两条容错规则（主控侧已实现）：
- 超过 `VISION_TIMEOUT_MS`（120 ms）没新帧 → 判丢线；
- 置 `VALID` 为 0 但保留元素标志 → 只更新标志，偏差沿用上一帧（用于"看到十字但中心线不可信"）。

**如果图像组给的方向反了**：把 `board_config.h` 里的 `VISION_DEV_INVERT` 改成 1，不要改算法。
**如果图像组暂时给不出十字/环岛/断路标志**：断路可以用 `GAP_BY_ODOM_ENABLE=1` + `GAP_ODOM_START_CM`
按里程触发兜底；十字/环岛没有里程兜底，必须靠视觉（赛题里位置固定但本版未标定具体坐标）。

---

## 6. 状态机与各元素的判据

| 状态 | 进入条件 | 行为 | 退出条件 |
|------|----------|------|----------|
| `WAIT_START` | 上电 / `Race_Init()` | 目标 0，LED1 心跳、LED2=视觉在线 | 按发车键（且视觉已就绪，除非 `RACE_REQUIRE_VISION=0`） |
| `LINE` | 发车 | 外环跟 `dev`，弯道自动降速 | 见下面三个元素 / 里程到 `LAP_LENGTH_CM` / 丢线超时 / STOP 标志 |
| `CROSS` | CROSS 标志持续 40 ms 且距上个元素 > 40 cm | **不跟视觉**，用航向保直行，`V_CROSS_CM_S` | 跑够 `CROSS_MIN_CM` 且重新看到 LINE 并持续 40 ms；或跑满 `CROSS_MAX_CM` 强制退出 |
| `RING` | RING_IN 标志持续 40 ms | 三段：进环 18 cm → 沿环 → 出环 22 cm | 沿行进方向累计转角 ≥ 320° 且看到 LINE → 出环段 → `LINE`；12 s 超时兜底 |
| `GAP` | GAP 标志持续 30 ms（或里程兜底） | **航向保直行**，`V_GAP_CM_S` | 跑够 `GAP_TRAVEL_LEN_CM`=192 cm 且视觉恢复 LINE → `LINE`；超过 240 cm → 报警停车 |
| `STOP` | 里程到一圈 / STOP 标志 / 急停 / 报警 | 目标 0，等实测轮速 < 1.5 cm/s 持续 500 ms | → `FINISH`（有报警时停在 STOP 等人工） |
| `FINISH` | 停稳 | 目标 0，LED1 常亮，蜂鸣 300 ms | 再按发车键 → 复位回 `WAIT_START` |

**丢线三级处理**（`LINE` 状态内）：
1. `age ≤ 180 ms`：沿用上一帧偏差继续跑，降速到 `V_RECOVER_CM_S`；
2. `180 ms < age ≤ 1200 ms`：锁定丢失瞬间的航向，用 `LineFeed_YawHold()` 保直行低速找线；
3. `age > 1200 ms`：`ALARM_VISION_LOST` + 停车。

**为什么十字必须"盲过"**：十字处视觉会同时看到两条白线，偏差必然跳变，这时打舵就是"十字走错"，
所以十字段一律锁航向直行，靠里程 + "重新看到正常赛道"两个条件退出。

**环岛方向**：`race.c` 用进环瞬间 `dev` 的符号决定方向（`dev>0` → 右转/顺时针）。
累计转角用**逐拍累加**（`s_ring_progress`），不能用 `wrap_pi(当前-入环)` 直接相减——
转过 180° 后差值会被卷回负值，永远到不了 320°（这个坑是回归测试抓出来的）。
如果实测方向反了，把 `state_enter(RACE_STATE_RING)` 里 `s_ring_omega_sign` 的取值反过来即可。

---

## 7. 调参与调试手段

### 7.1 无线命令（USART1，115200）

| 命令 | 作用 |
|------|------|
| `P,v,140` | 巡线基础速度改 140 cm/s（弯道速度按比例跟着变） |
| `P,p,1.3` | 外环 Kp 倍数 |
| `P,i,0.8` | 外环 Ki 倍数 |
| `P,d,1.5` | 外环 Kd 倍数 |
| `S` | 回传一行状态：`ST,<状态>,L<圈>,D<里程>,T<圈时>,V<视觉帧数>,A<故障码>` |
| `R` | 复位到等待发车（清报警） |

### 7.2 VOFA+ JustFloat（USART1）

`Comm_SendJustFloat()` 每 50 ms 发一帧 8 通道，VOFA+ 选 **JustFloat** 协议，通道顺序：

| ch | 含义 |
|----|------|
| 0 / 1 | 左轮 目标轮速 / 实测轮速（cm/s） |
| 2 / 3 | 右轮 目标轮速 / 实测轮速（cm/s） |
| 4 | 视觉偏差 dev |
| 5 | 外环输出 ω（rad/s） |
| 6 | 航向 yaw（rad） |
| 7 | 状态机状态（0~7 对应 `Race_StateName`） |

### 7.3 灯与蜂鸣器

| 现象 | 含义 |
|------|------|
| LED1 250 ms 闪 | `WAIT_START`，等发车 |
| LED1 常亮 | `FINISH`，跑完停稳 |
| LED2 亮 | 视觉帧正在刷新 |
| LED3 亮 | 正在跑（LINE/CROSS/RING/GAP） |
| LED4 快闪 2 s 后慢闪 | 有故障码，蜂鸣器每秒短鸣 |
| 短鸣 40 ms | 进入元素（十字/环岛/断路） |
| 长鸣 400 ms | 发车被拒（视觉未就绪） |

### 7.4 参数在哪改

全部在 `board_config.h`，按小节组织：

- **几何**：`WHEEL_DIAMETER_CM`、`TRACK_WIDTH_CM`、`ENC_x_COUNTS_PER_REV`
- **速度**：`V_LINE/V_CURVE/V_CROSS/V_RING/V_GAP_CM_S`
- **外环**：`LINE_KP_BASE`、`LINE_KI_BASE`、`LINE_KD_BASE`、`LINE_V_REF_CM_S`
- **内环**：`WHEEL_KP`、`WHEEL_KI`、`WHEEL_DELTA_MAX`
- **元素**：`CROSS_MIN_CM/CROSS_MAX_CM`、`RING_*`、`GAP_*`

---

## 8. 已知取舍 / TODO

1. **PWM 频率 8.4 kHz（不是 20 kHz）**：这是 `NEUQ_PWM_init` 能给出的最高常用频率。
   库的预分频 = `84MHz/(freq×10000)`，`freq=20000` 时整除得 0，再 `−1` 会变成 `0xFFFF`，
   实际输出只剩几 Hz。`freq=8400` 时 TIM2 预分频 0、TIM8 预分频 1，两边都正好 84 MHz/10000。
   8.4 kHz 对直流电机完全够用，若以后想上更高频率，得先改库的 `NEUQ_PWM_init`。
2. **三色路障（未做）**：现版本对路障一律"直接通过"。
   - 绿障：规则一致；黄障：会被判 10 s 罚时（规则要求绕行）；红障：第一圈忽略，第二圈停 5 s 未做。
   - 扩展点：`race.c` 增加 `RACE_STATE_BUMP`，颜色可复用 flags 的 bit7 或扩协议。
3. **6×6 数独 / 软件盲盒（未做）**：`RACE_STATE_STOP` 之后留了 `#if RACE_RUN_SECOND_LAP` 的钩子。
4. **环岛进环方向判据**基于 `dev` 符号，第一次实测必须盯着看，反了改一个符号（第 6 节）。
5. **电池分压比** `BATTERY_DIV_RATIO=11.0` 是估计值，按实物电阻改；不确定就把 `BATTERY_CHECK_ENABLE` 置 0。
6. **底盘几何默认值**（轮径 6.5 cm、轮距 15 cm、编码器 1560 计数/圈）都是**待标定占位值**。
7. **对库做了一处最小修改**：`NEUQ_lib_APP/NEUQ_PID.c` 补了 `#include <math.h>`。
   原因：该文件用了 `fmaxf/fminf` 但没有包含 `math.h`，C99 模式下会报"隐式声明"。
   逻辑一行未改，只加了一条 include（已在该文件里注明）。

---

## 9. 常见问题排查

| 现象 | 排查顺序 |
|------|----------|
| 上电电机自转 | 先看 `MOTOR_x_DIR_FWD`；再确认 `Motor_Init()` 末尾 `Motor_Stop()` 生效；PWM 引脚是否被其它外设占用 |
| 有 PWM 但轮子不转 | 死区补偿 `MOTOR_DEADZONE` 调大；电池电压；驱动板使能脚（本版未接 `nFAULT`/`MOTOR_ARM`） |
| 电机啸叫 | 8.4 kHz 的开关声，属正常；要更安静得改库的 PWM 实现 |
| 轮速反馈一直是 0 | 编码器 5V 供电、A/B 相是否接反、`ENC_x_SIGN`、示波器看 TIM3/TIM4 计数器是否变化 |
| 轮速正负不对 | 推车向前，`Encoder_GetSpeedL/R` 应为正；否则改 `ENC_x_SIGN` |
| 速度上不去/来回抖 | 先降 `WHEEL_KP` 再降 `WHEEL_KI`；看 VOFA 目标/实测曲线是不是发散 |
| 巡线画龙 | 降 `LINE_KP_BASE`、加 `LINE_KD_BASE`（注意是库口径，×100）、把 `LINE_DEV_LPF_ALPHA` 调小 |
| 弯道切出去 | 提高 `LINE_KP_BASE`、降 `V_LINE_CM_S`、检查 `LAT_ACC_MAX_CM_S2` 是否太小 |
| 十字后找不到线 | `CROSS_MAX_CM` 太小，或视觉 `LINE` 标志恢复太慢 |
| 断路跑偏 | 陀螺没校准（`IMU_IsReady()` 是否为 1）、`IMU_RATE_SIGN`、`YAW_KP` |
| 视觉帧收不到 | 共地、TX/RX 交叉、波特率、PA9/PA10 是否被无线模块占用、`Comm_VisionIsFresh()` |

---

## 10. 编译

EIDE（VS Code）打开工程，目标 `NEUQ_STM32F407VET6_Opensource_Library`，工具链 AC5，直接 Build。
代码为 C99 子集、不含 VLA、不用 `%f`（microlib），源码为 **UTF-8** 编码；
若 Keil 里中文注释显示乱码，把 `Edit → Configuration → Encoding` 设为 `UTF-8` 即可，不影响编译。

> `CODE/` 已经在 `.eide/eide.yml` 的 `srcDirs` 里，**新增 .c 不需要改工程文件**。
> 仓库自带的 `MDK-ARM/*.uvprojx` 是 CubeMX 生成的旧工程，里面没有 NEUQ 库和 User 文件，不要用它编译。

---

## 11. 逻辑回归测试（没有硬件也能验证）

`tests/` 目录里有一套在 PC 上跑的测试，用假视觉 + 假底盘验证状态机与协议，
并且**链接的是库里真实的 `NEUQ_PID.c`**（不是替身）：

```bat
:: 任意 C 编译器都行，下面用 tcc（解压即用，几百 KB）
set LIB=Library\NEUQ_lib
tcc -run -I tests\host -I CODE -I %LIB%\NEUQ_lib_Driver -I %LIB%\NEUQ_lib_APP -I %LIB%\NEUQ_lib_Device -I %LIB%\NEUQ_lib_Debug tests\host\all.c
tcc -run -I tests\host -I CODE -I %LIB%\NEUQ_lib_Driver -I %LIB%\NEUQ_lib_APP -I %LIB%\NEUQ_lib_Device -I %LIB%\NEUQ_lib_Debug tests\host\all_comm.c
```

- `all.c`：34 项，状态机 + 外环（十字/环岛/断路/停车/丢线/方向符号）
- `all_comm.c`：24 项，视觉帧与无线协议解析

`tests/` 不在 EIDE 的 `srcDirs` 里，不会被编进固件。细节见 `tests/README.md`。

本版交付前已经跑过：

- `CODE/` 下 11 个 .c **全部对着真实库头文件编译通过**（`-Wall` 无告警）——
  也就是说 `NEUQ_PWM/Encoder/GPIO/ADC/PID/UART/PIT/MPU6050` 的每个调用签名都被编译器检查过；
- 上面两套回归共 **58 项断言全部通过**。
  其中"环岛累计转角"一项曾经用 `wrap_pi(当前-入环)` 直接相减，转过 180° 后差值被卷回负值，
  导致永远到不了 320°、只能靠超时退出，是这套测试抓出来并修掉的。
