/**
  ******************************************************************************
  * @file    comm.c
  * @brief   串口收发实现
  *
  *          - 中断里只做一件事：把字节塞进环形缓冲，绝不在中断里解析；
  *          - 主循环 Comm_Poll() 逐字节拼行，再按协议解析；
  *          - 环形缓冲是单生产（中断）/单消费（主循环）模型，head/tail 用
  *            volatile uint16_t，Cortex-M4 上读写是原子的，不需要关中断。
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>

#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "comm.h"

/* NEUQ_Uart.c 里有这个实现，但头文件漏了声明，这里补一条 */
extern void NEUQ_Uart_Enable_RX_Interrupt(UART_DEV_ENUM DEV_select, uint8_t priority);

#define COMM_LINE_MAX       64u
#define COMM_JUSTFLOAT_MAX  12u

typedef struct
{
    uint8_t buf[COMM_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} Comm_Ring;

static Comm_Ring s_ring_vision;
static Comm_Ring s_ring_debug;

static char  s_line_vision[COMM_LINE_MAX];
static uint16_t s_line_vision_len;
static char  s_line_debug[COMM_LINE_MAX];
static uint16_t s_line_debug_len;

static VisionFrame s_vision;
static uint8_t  s_have_frame;
static char     s_tune_key;
static float    s_tune_value;
static uint8_t  s_tune_pending;
static char     s_command;

/* --------------------------------------------------------------------------
 * 环形缓冲
 * ------------------------------------------------------------------------ */
static void ring_reset(Comm_Ring *r)
{
    r->head = 0;
    r->tail = 0;
}

static void ring_put(Comm_Ring *r, uint8_t byte)
{
    uint16_t next;

    next = (uint16_t)((r->head + 1u) % COMM_RX_BUF_SIZE);
    if (next == r->tail) {
        return;   /* 满了就丢新字节，比赛数据宁可丢也不能乱序 */
    }
    r->buf[r->head] = byte;
    r->head = next;
}

static uint8_t ring_get(Comm_Ring *r, uint8_t *byte)
{
    if (r->head == r->tail) {
        return 0;
    }
    *byte = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) % COMM_RX_BUF_SIZE);
    return 1;
}

/* --------------------------------------------------------------------------
 * 轻量数字解析（不依赖 sscanf，省 flash 也快）
 * ------------------------------------------------------------------------ */
static uint8_t parse_float(const char **pp, float *out)
{
    const char *p;
    float sign;
    float val;
    float frac;
    uint8_t has;

    p    = *pp;
    sign = 1.0f;
    val  = 0.0f;
    frac = 0.1f;
    has  = 0;

    while (*p == ' ') {
        p++;
    }
    if (*p == '-') {
        sign = -1.0f;
        p++;
    } else if (*p == '+') {
        p++;
    }
    while (*p >= '0' && *p <= '9') {
        val = val * 10.0f + (float)(*p - '0');
        p++;
        has = 1;
    }
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            val += (float)(*p - '0') * frac;
            frac *= 0.1f;
            p++;
            has = 1;
        }
    }
    if (!has) {
        return 0;
    }
    *out = sign * val;
    *pp  = p;
    return 1;
}

static uint8_t parse_uint(const char **pp, uint32_t *out)
{
    const char *p;
    uint32_t val;
    uint8_t has;

    p   = *pp;
    val = 0;
    has = 0;

    while (*p == ' ') {
        p++;
    }
    while (*p >= '0' && *p <= '9') {
        val = val * 10u + (uint32_t)(*p - '0');
        p++;
        has = 1;
    }
    if (!has) {
        return 0;
    }
    *out = val;
    *pp  = p;
    return 1;
}

/* 跳过一个字段（到下一个逗号或行尾） */
static void skip_field(const char **pp)
{
    const char *p;

    p = *pp;
    while ((*p != '\0') && (*p != ',')) {
        p++;
    }
    if (*p == ',') {
        p++;
    }
    *pp = p;
}

/* --------------------------------------------------------------------------
 * 视觉帧解析： V,dev,flags,seq
 * ------------------------------------------------------------------------ */
static void handle_vision_line(void)
{
    const char *p;
    float   dev;
    uint32_t flags;
    uint32_t seq;
    uint8_t  seq_ok;

    p = s_line_vision;

    while (*p == ' ') {
        p++;
    }
    if ((p[0] != 'V') || (p[1] != ',')) {
        return;   /* 不认识的帧直接丢弃 */
    }
    p += 2;

    if (!parse_float(&p, &dev)) {
        return;
    }
    skip_field(&p);

    flags = 0u;
    (void)parse_uint(&p, &flags);
    skip_field(&p);

    seq    = 0u;
    seq_ok = parse_uint(&p, &seq);

    if ((flags & VISION_FLAG_VALID) == 0u) {
        /* 元素标志可能有效但偏差不可信：只更新标志与时间戳，偏差沿用上一帧 */
        s_vision.flags    = (uint8_t)flags;
        s_vision.stamp_ms = HAL_GetTick();
        return;
    }

    if (dev > VISION_DEV_MAX) {
        dev = VISION_DEV_MAX;
    } else if (dev < -VISION_DEV_MAX) {
        dev = -VISION_DEV_MAX;
    }

    if (s_have_frame && seq_ok) {
        uint8_t expect = (uint8_t)((s_vision.seq + 1u) & 0xFFu);
        if ((uint8_t)seq != expect) {
            s_vision.drops++;
        }
    }

    s_vision.dev      = dev;
    s_vision.flags    = (uint8_t)flags;
    s_vision.seq      = (uint8_t)seq;
    s_vision.stamp_ms = HAL_GetTick();
    s_vision.frames++;
    s_have_frame = 1;
}

/* --------------------------------------------------------------------------
 * 无线命令解析： P,key,value / S / R
 * ------------------------------------------------------------------------ */
static void handle_debug_line(void)
{
    const char *p;
    char key;
    float value;

    p = s_line_debug;

    while (*p == ' ') {
        p++;
    }

    if ((p[0] == 'P' || p[0] == 'p') && (p[1] == ',')) {
        p += 2;
        key = *p;
        if (key == '\0') {
            return;
        }
        p++;
        if (*p == ',') {
            p++;
        }
        if (!parse_float(&p, &value)) {
            return;
        }
        s_tune_key     = key;
        s_tune_value   = value;
        s_tune_pending = 1;
        return;
    }

    if ((p[0] == 'S' || p[0] == 's') && (p[1] == '\0' || p[1] == '\r')) {
        s_command = 'S';
        return;
    }
    if ((p[0] == 'R' || p[0] == 'r') && (p[1] == '\0' || p[1] == '\r')) {
        s_command = 'R';
        return;
    }
}

/* --------------------------------------------------------------------------
 * 把环形缓冲里的字节拼成行：返回 1 表示拼好了一条完整行（已补 '\0'）
 * ------------------------------------------------------------------------ */
static uint8_t poll_ring(Comm_Ring *ring, char *line, uint16_t *len, uint16_t max_len)
{
    uint8_t byte;

    while (ring_get(ring, &byte)) {
        if (byte == '\n') {
            line[*len] = '\0';
            return 1u;
        }
        if (byte == '\r') {
            continue;
        }
        if (*len < (uint16_t)(max_len - 1u)) {
            line[*len] = (char)byte;
            (*len)++;
        } else {
            *len = 0;   /* 超长行：丢掉重来，防止脏数据把缓冲区搞乱 */
        }
    }
    return 0u;
}

/* ==========================================================================
 * 对外接口
 * ======================================================================== */
void Comm_Init(void)
{
    ring_reset(&s_ring_vision);
    ring_reset(&s_ring_debug);
    s_line_vision_len = 0;
    s_line_debug_len  = 0;
    s_have_frame = 0;
    s_tune_pending = 0;
    s_command = 0;

    s_vision.dev      = 0.0f;
    s_vision.flags    = 0u;
    s_vision.seq      = 0u;
    s_vision.stamp_ms = 0u;
    s_vision.frames   = 0u;
    s_vision.drops    = 0u;

    NEUQ_Uart_init(COMM_VISION_UART, COMM_VISION_BAUD);
    NEUQ_Uart_init(COMM_DEBUG_UART,  COMM_DEBUG_BAUD);

    NEUQ_Uart_Enable_RX_Interrupt(COMM_VISION_UART, 2);
    NEUQ_Uart_Enable_RX_Interrupt(COMM_DEBUG_UART,  3);
}

void Comm_OnByteVision(uint8_t byte)
{
    ring_put(&s_ring_vision, byte);
}

void Comm_OnByteDebug(uint8_t byte)
{
    ring_put(&s_ring_debug, byte);
}

void Comm_Poll(void)
{
    while (poll_ring(&s_ring_vision, s_line_vision, &s_line_vision_len, COMM_LINE_MAX)) {
        handle_vision_line();
        s_line_vision_len = 0;
    }
    while (poll_ring(&s_ring_debug, s_line_debug, &s_line_debug_len, COMM_LINE_MAX)) {
        handle_debug_line();
        s_line_debug_len = 0;
    }
}

uint8_t Comm_VisionIsFresh(uint32_t now_ms)
{
    if (!s_have_frame) {
        return 0;
    }
    if ((now_ms - s_vision.stamp_ms) > VISION_TIMEOUT_MS) {
        return 0;
    }
    return 1;
}

uint32_t Comm_VisionAgeMs(uint32_t now_ms)
{
    if (!s_have_frame) {
        return 0xFFFFFFFFu;
    }
    return now_ms - s_vision.stamp_ms;
}

const VisionFrame *Comm_GetVision(void)
{
    return &s_vision;
}

void Comm_SendText(const char *text)
{
    if (text == 0) {
        return;
    }
    NEUQ_Uart_write_string(COMM_DEBUG_UART, (uint8_t *)text);
}

void Comm_SendJustFloat(const float *ch, uint8_t count)
{
    uint8_t buf[COMM_JUSTFLOAT_MAX * 4u + 4u];
    uint8_t i;
    uint32_t bits;

    if (ch == 0) {
        return;
    }
    if (count > COMM_JUSTFLOAT_MAX) {
        count = COMM_JUSTFLOAT_MAX;
    }

    for (i = 0; i < count; i++) {
        memcpy(&bits, &ch[i], 4);
        buf[i * 4u + 0u] = (uint8_t)(bits & 0xFFu);
        buf[i * 4u + 1u] = (uint8_t)((bits >> 8) & 0xFFu);
        buf[i * 4u + 2u] = (uint8_t)((bits >> 16) & 0xFFu);
        buf[i * 4u + 3u] = (uint8_t)((bits >> 24) & 0xFFu);
    }
    buf[count * 4u] = 0x00u;
    buf[count * 4u + 1u] = 0x00u;
    buf[count * 4u + 2u] = 0x80u;
    buf[count * 4u + 3u] = 0x7Fu;

    NEUQ_Uart_write_data(COMM_DEBUG_UART, buf, (uint32_t)(count * 4u + 4u));
}

uint8_t Comm_TakeTune(char *key, float *value)
{
    if (!s_tune_pending) {
        return 0;
    }
    if (key != 0) {
        *key = s_tune_key;
    }
    if (value != 0) {
        *value = s_tune_value;
    }
    s_tune_pending = 0;
    return 1;
}

char Comm_TakeCommand(void)
{
    char cmd;

    cmd = s_command;
    s_command = 0;
    return cmd;
}
