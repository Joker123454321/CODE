/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   编码器测速 / 里程（基于 NEUQ_Encoder）
  *
  *          NEUQ_Encoder_init() 内部已经把 TIM3/TIM4 配成编码器模式 TI12（四倍频）、
  *          ARR = 0xFFFF、预分频 0，并把计数器清到 0x7FFF；
  *          NEUQ_Encoder_Read() 返回"相对 0x7FFF 的位置"，不是增量。
  *
  *          所以本文件只做库没提供的那部分：
  *            位置差分（int16 回卷相减）→ 速度 cm/s → 里程 cm → 一阶低通
  ******************************************************************************
  */
#include "NEUQ_lib_include.h"

#include "board_config.h"
#include "encoder.h"

typedef struct
{
    Encoder_DEV_ENUM dev;
    float    cm_per_count;
    int16_t  sign;
    int16_t  last_cnt;      /* 上一拍 NEUQ_Encoder_Read 的位置 */
    int32_t  total_cnt;
    float    dist_cm;
    float    speed_cm_s;
    float    speed_raw_cm_s;
} Encoder_Dev;

static Encoder_Dev s_enc_l;
static Encoder_Dev s_enc_r;

/* --------------------------------------------------------------------------
 * 读库的编码器位置（16 位有符号，基准 0x7FFF）
 * ------------------------------------------------------------------------ */
static int16_t enc_read(Encoder_DEV_ENUM dev)
{
    return (int16_t)NEUQ_Encoder_Read(dev);
}

/* --------------------------------------------------------------------------
 * 单路更新
 * ------------------------------------------------------------------------ */
static void enc_update_one(Encoder_Dev *e, float dt)
{
    int16_t cnt;
    int16_t delta;
    float   delta_cm;

    cnt   = enc_read(e->dev);
    delta = (int16_t)(cnt - e->last_cnt);   /* 回卷相减，天然正确 */
    e->last_cnt = cnt;

    delta = (int16_t)(delta * e->sign);
    e->total_cnt += delta;

    delta_cm   = (float)delta * e->cm_per_count;
    e->dist_cm += delta_cm;

    e->speed_raw_cm_s = delta_cm / dt;
    e->speed_cm_s += WHEEL_SPEED_LPF_ALPHA * (e->speed_raw_cm_s - e->speed_cm_s);
}

/* ==========================================================================
 * 对外接口
 * ======================================================================== */
void Encoder_Init(void)
{
    float cpr_l;
    float cpr_r;

    /* 库函数：DEV / PhaseA(CH1) / PhaseB(CH2)
     * 引脚表里 A 相是 CH2、B 相是 CH1，但库的 PhaseA 参数指 CH1，
     * 所以这里按"PB6/PB7、PA6/PA7"的顺序传，正方向再用 ENC_x_SIGN 纠正。 */
    NEUQ_Encoder_init(ENC_L_DEV, ENC_L_PHASE_A, ENC_L_PHASE_B);
    NEUQ_Encoder_init(ENC_R_DEV, ENC_R_PHASE_A, ENC_R_PHASE_B);

    cpr_l = (ENC_L_COUNTS_PER_REV > 1.0f) ? ENC_L_COUNTS_PER_REV : 1.0f;
    cpr_r = (ENC_R_COUNTS_PER_REV > 1.0f) ? ENC_R_COUNTS_PER_REV : 1.0f;

    s_enc_l.dev          = ENC_L_DEV;
    s_enc_r.dev          = ENC_R_DEV;
    s_enc_l.cm_per_count = (BOARD_PI_F * WHEEL_DIAMETER_CM) / cpr_l;
    s_enc_r.cm_per_count = (BOARD_PI_F * WHEEL_DIAMETER_CM) / cpr_r;
    s_enc_l.sign         = ENC_L_SIGN;
    s_enc_r.sign         = ENC_R_SIGN;

    Encoder_ResetOdom();
}

void Encoder_ResetOdom(void)
{
    s_enc_l.last_cnt  = enc_read(s_enc_l.dev);
    s_enc_r.last_cnt  = enc_read(s_enc_r.dev);
    s_enc_l.total_cnt = 0;
    s_enc_r.total_cnt = 0;
    s_enc_l.dist_cm   = 0.0f;
    s_enc_r.dist_cm   = 0.0f;
    s_enc_l.speed_cm_s = 0.0f;
    s_enc_r.speed_cm_s = 0.0f;
    s_enc_l.speed_raw_cm_s = 0.0f;
    s_enc_r.speed_raw_cm_s = 0.0f;
}

void Encoder_Update(float dt)
{
    if (dt <= 0.0f) {
        return;
    }
    enc_update_one(&s_enc_l, dt);
    enc_update_one(&s_enc_r, dt);
}

float Encoder_GetSpeedL(void)     { return s_enc_l.speed_cm_s; }
float Encoder_GetSpeedR(void)     { return s_enc_r.speed_cm_s; }
float Encoder_GetSpeedRawL(void)  { return s_enc_l.speed_raw_cm_s; }
float Encoder_GetSpeedRawR(void)  { return s_enc_r.speed_raw_cm_s; }
float Encoder_GetDistL(void)      { return s_enc_l.dist_cm; }
float Encoder_GetDistR(void)      { return s_enc_r.dist_cm; }
float Encoder_GetDistAvg(void)    { return (s_enc_l.dist_cm + s_enc_r.dist_cm) * 0.5f; }
int32_t Encoder_GetCntL(void)     { return s_enc_l.total_cnt; }
int32_t Encoder_GetCntR(void)     { return s_enc_r.total_cnt; }
