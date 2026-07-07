/*******************************************************************************
 * demod.c  —  数字基带解调模块（最终版，可直接编译）
 *
 * 处理流水线（每个 ADC 采样点）：
 *   s8 sample → 混频(载波LUT) → FIR RRC低通(Q15,15阶) → 符号同步计数器
 *              → 抽样判决(BPSK/QPSK/16QAM) → BER统计(PN7) → 回调
 *******************************************************************************/
#include "demod.h"

/*------------------------------------------------------------
 * 载波 LUT（32点，Q7，cos/sin 一个完整周期）
 * Fc/Fs = 1/8，每采样步进 32/8 = 4 个 LUT 索引
 *------------------------------------------------------------*/
static const s8 cos_lut[32] = {
     127, 117,  90,  49,   0, -49, -90,-117,
    -127,-117, -90, -49,   0,  49,  90, 117,
     127, 117,  90,  49,   0, -49, -90,-117,
    -127,-117, -90, -49,   0,  49,  90, 117
};
static const s8 sin_lut[32] = {
       0,  49,  90, 117, 127, 117,  90,  49,
       0, -49, -90,-117,-127,-117, -90, -49,
       0,  49,  90, 117, 127, 117,  90,  49,
       0, -49, -90,-117,-127,-117, -90, -49
};

/*------------------------------------------------------------
 * FIR 根升余弦（RRC）系数
 * 参数：阶数15，SPS=8，滚降 α=0.35
 * Q15 格式（×32768），对称线性相位
 *------------------------------------------------------------*/
static const s16 fir_coeff[DEMOD_FIR_LEN] = {
    -512,  -768,  -384,   768,  3072,
    6144,  9216, 10240,  9216,  6144,
    3072,   768,  -384,  -768,  -512
};

/*------------------------------------------------------------
 * PN7 序列生成（多项式 x^7+x^6+1，用于BER比对）
 *------------------------------------------------------------*/
static u8 PN7_Next(u8 *state)
{
    u8 bit = ((*state >> 6) ^ (*state >> 5)) & 1u;
    *state  = (u8)((*state << 1) | bit) & 0x7Fu;
    return bit;
}

/*------------------------------------------------------------
 * FIR 滤波（I/Q 两路共用同一写指针，单次卷积完成两路）
 *------------------------------------------------------------*/
static void FIR_Filter_IQ(s16 *buf_i, s16 *buf_q, u8 *ptr,
                           s16 in_i,   s16 in_q,
                           s16 *out_i, s16 *out_q)
{
    s32 acc_i = 0, acc_q = 0;
    u8  i, idx;

    buf_i[*ptr] = in_i;
    buf_q[*ptr] = in_q;
    *ptr = (u8)((*ptr + 1u) % (u8)DEMOD_FIR_LEN);

    for (i = 0u; i < (u8)DEMOD_FIR_LEN; i++) {
        idx    = (u8)((*ptr + i) % (u8)DEMOD_FIR_LEN);
        acc_i += (s32)buf_i[idx] * (s32)fir_coeff[i];
        acc_q += (s32)buf_q[idx] * (s32)fir_coeff[i];
    }

    acc_i >>= 15;
    acc_q >>= 15;
    /* 饱和截断到 [-127, 127] */
    if (acc_i >  127) acc_i =  127;
    if (acc_i < -127) acc_i = -127;
    if (acc_q >  127) acc_q =  127;
    if (acc_q < -127) acc_q = -127;
    *out_i = (s16)acc_i;
    *out_q = (s16)acc_q;
}

/*------------------------------------------------------------
 * 判决函数
 *------------------------------------------------------------*/
/* BPSK: I路符号即比特 */
static u8 Decision_BPSK(s16 si)
{
    return (si > 0) ? 1u : 0u;
}

/* QPSK: I→MSB, Q→LSB，格雷码 */
static u8 Decision_QPSK(s16 si, s16 sq)
{
    return (u8)(((si > 0) ? 2u : 0u) | ((sq > 0) ? 1u : 0u));
}

/* 16QAM: I/Q 各2bit，三阈值 ±32 判决 */
static u8 Decision_Axis_QAM16(s16 v)
{
    if      (v < -32) return 0u;   /* 格雷码 00 */
    else if (v <   0) return 1u;   /* 格雷码 01 */
    else if (v <  32) return 3u;   /* 格雷码 11 */
    else              return 2u;   /* 格雷码 10 */
}
static u8 Decision_16QAM(s16 si, s16 sq)
{
    return (u8)((Decision_Axis_QAM16(si) << 2) | Decision_Axis_QAM16(sq));
}

/*============================================================
 * 公开接口实现
 *============================================================*/
void Demod_Init(DemodState_t *st, u8 mod_idx)
{
    u8 i;
    for (i = 0u; i < (u8)DEMOD_FIR_LEN; i++) {
        st->fir_buf_i[i] = 0;
        st->fir_buf_q[i] = 0;
    }
    st->carrier_idx = 0u;
    st->fir_ptr     = 0u;
    st->sym_cnt     = 0u;
    st->sample_i    = 0;
    st->sample_q    = 0;
    st->last_bits   = 0u;
    st->last_nbits  = 0u;
    st->ber_total   = 0u;
    st->ber_errors  = 0u;
    st->pn_state    = 0x01u;
    st->mod_idx     = mod_idx;
}

void Demod_PushSample(DemodState_t *st, s8 sample)
{
    s16 mix_i, mix_q, filt_i, filt_q;
    u8  bits = 0u, nbits = 0u, b, ref, errs = 0u;

    /*--- Step 1: 混频（下变频） ---*/
    mix_i = (s16)(((s16)sample *  (s16)cos_lut[st->carrier_idx]) >> 7);
    mix_q = (s16)(((s16)sample * -(s16)sin_lut[st->carrier_idx]) >> 7);
    st->carrier_idx = (u8)((st->carrier_idx + (u8)DEMOD_FC_IDX_STEP) & 0x1Fu);

    /*--- Step 2: RRC FIR 低通滤波 ---*/
    FIR_Filter_IQ(st->fir_buf_i, st->fir_buf_q, &st->fir_ptr,
                  mix_i, mix_q, &filt_i, &filt_q);

    /*--- Step 3: 符号同步（每 SPS 次抽一次） ---*/
    st->sym_cnt++;
    if (st->sym_cnt < (u8)DEMOD_SPS) return;
    st->sym_cnt  = 0u;
    st->sample_i = filt_i;
    st->sample_q = filt_q;

    /*--- Step 4: 抽样判决 ---*/
    switch (st->mod_idx) {
        case MOD_BPSK:
            bits = Decision_BPSK(filt_i);  nbits = 1u; break;
        case MOD_QPSK:
            bits = Decision_QPSK(filt_i, filt_q); nbits = 2u; break;
        case MOD_16QAM:
        default:
            bits = Decision_16QAM(filt_i, filt_q); nbits = 4u; break;
    }
    st->last_bits  = bits;
    st->last_nbits = nbits;

    /*--- Step 5: BER 统计 ---*/
    for (b = 0u; b < nbits; b++) {
        ref = PN7_Next(&st->pn_state);
        if (((bits >> b) & 1u) != ref) errs++;
    }
    st->ber_total  += nbits;
    st->ber_errors += errs;

    Demod_OnBit(bits, nbits, filt_i, filt_q);
}

u32 Demod_GetBER_Q20(DemodState_t *st)
{
    if (st->ber_total == 0u) return 0u;
    /* 避免使用 u64：若 ber_errors 较小则先移位 */
    return (u32)(((u32)st->ber_errors << 20) / st->ber_total);
}

void Demod_ResetBER(DemodState_t *st)
{
    st->ber_total  = 0u;
    st->ber_errors = 0u;
    st->pn_state   = 0x01u;
}
