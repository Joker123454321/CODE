/**
  ******************************************************************************
  * @file    comm.h
  * @brief   串口收发：视觉帧（UART4）+ 无线调试（USART1）
  *
  * ============================ 视觉协议（给图像组） ============================
  *  物理层：UART4，PC10=STM32 TX，PC11=STM32 RX，115200 8N1，共地。
  *  帧格式：ASCII 行，每条以 '\n' 结尾（'\r' 可有可无），字段用英文逗号分隔。
  *
  *      V,<dev>,<flags>,<seq>\n
  *
  *      dev   : 归一化横向偏差，浮点，-100.0 ~ +100.0
  *              **车相对赛道中心偏左为正**（需要向右打舵时为正）。
  *              |dev|=100 约等于半个赛道宽度（45 cm 的 1/2）＝ 22.5 cm。
  *      flags : 无符号整数（0~255），位定义见下面的 VISION_FLAG_*
  *      seq   : 0~255 循环递增，本端只统计丢帧，不做重传
  *
  *      例：V,-12.5,33,145\n     → 车偏右 12.5%，flags=0x21(有效+正常赛道)
  *
  *  flags 位定义：
  *      bit0 0x01  VALID     本帧有效（必须置位，不置位整帧丢弃）
  *      bit1 0x02  CROSS     前方/当前是十字路口
  *      bit2 0x04  RING_IN   即将进环岛（进环方向由 dev 的符号给出）
  *      bit3 0x08  RING_OUT  已到环岛出口，能看到正常赛道
  *      bit4 0x10  GAP       前方是 160 cm 断路（无白色赛道面）
  *      bit5 0x20  LINE      正常赛道可信（巡线用）
  *      bit6 0x40  STOP_MARK 到达停车标志（第二圈 / 终点）
  *      bit7 0x80  保留
  *
  *  约定：
  *    - 视觉帧率建议 ≥ 40 Hz；本端超过 VISION_TIMEOUT_MS(120 ms) 没收到新帧
  *      就按"丢线"处理（减速并保航向）。
  *    - 只有 flags.VALID 置位的帧才会更新偏差；置 VALID 但 dev 不可信时，
  *      请把 VALID 清零、只保留元素标志，本端会沿用上一帧偏差。
  *
  * ============================ 无线调试协议（可选） ============================
  *  USART1，115200 8N1。用于现场调参、看状态，不影响比赛流程。
  *
  *      P,<key>,<value>\n   调参：key 为单字符，value 为浮点
  *      S\n                 回传一行文本状态
  *      R\n                 复位到"等待发车"
  *
  *  已注册的 key（control.c 里实现，可自行增删）：
  *      'v' 巡线基础速度 cm/s      'p' 外环 Kp 倍数
  *      'i' 外环 Ki 倍数           'd' 外环 Kd 倍数
  ******************************************************************************
  */
#ifndef _COMM_H_
#define _COMM_H_

#include <stdint.h>

/* ---- flags 位定义 ---- */
#define VISION_FLAG_VALID       0x01u
#define VISION_FLAG_CROSS       0x02u
#define VISION_FLAG_RING_IN     0x04u
#define VISION_FLAG_RING_OUT    0x08u
#define VISION_FLAG_GAP         0x10u
#define VISION_FLAG_LINE        0x20u
#define VISION_FLAG_STOP        0x40u

typedef struct
{
    float    dev;        /* 归一化偏差，正 = 车偏左 */
    uint8_t  flags;      /* VISION_FLAG_* */
    uint8_t  seq;        /* 帧序号 */
    uint32_t stamp_ms;   /* 收到时刻 HAL_GetTick() */
    uint32_t frames;     /* 累计收到有效帧 */
    uint32_t drops;      /* 序号跳变次数 */
} VisionFrame;

/** 初始化两个串口与接收中断 */
void Comm_Init(void);

/** 串口中断里调用（NEUQ_Interrupts.c 的两个回调） */
void Comm_OnByteVision(uint8_t byte);
void Comm_OnByteDebug(uint8_t byte);

/** 主循环解析（非阻塞，每次把缓冲区里的字节处理完） */
void Comm_Poll(void);

/** 视觉帧是否新鲜 */
uint8_t Comm_VisionIsFresh(uint32_t now_ms);
/** 距上一帧的毫秒数 */
uint32_t Comm_VisionAgeMs(uint32_t now_ms);
/** 取最近一帧（指针常驻，不拷贝） */
const VisionFrame *Comm_GetVision(void);

/** 向无线口发一行文本（microlib 下不支持 %f，只用 %d/%s） */
void Comm_SendText(const char *text);
/** 向无线口发 VOFA+ JustFloat 帧（ch 为浮点数组，尾部自动加 0x00 0x00 0x80 0x7f） */
void Comm_SendJustFloat(const float *ch, uint8_t count);

/** 取一条待处理的调参命令（返回 1 表示取到） */
uint8_t Comm_TakeTune(char *key, float *value);
/** 取一条待处理的命令（'S' 状态 / 'R' 复位），返回 0 表示没有 */
char Comm_TakeCommand(void);

#endif /* _COMM_H_ */
