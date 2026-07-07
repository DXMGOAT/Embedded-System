/*******************************************************************************
 * demod.h  —  数字基带解调模块
 *
 * 功能：
 *   1. FIR 根升余弦（RRC）低通滤波器（15阶，Q15定点）
 *   2. BPSK / QPSK / 16QAM 相干解调（乘以本地载波 + 积分清零）
 *   3. 抽样判决（符号同步计数器 + 阈值判决）
 *   4. BER 统计（与已知 PN7 序列比对）
 *
 * 使用方法：
 *   每采到一个 ADC 样本（Fs=1MHz，8倍过采样）调用一次 Demod_PushSample()
 *   每当输出一个判决比特时，回调 Demod_OnBit() 被调用（用户实现）
 *
 * 定点约定：
 *   输入样本范围 [-128, 127]（s8 或 ADC>>4 后移位）
 *   载波 LUT 范围 [-128, 127]，乘积右移7位保持量纲
 *******************************************************************************/
#ifndef __DEMOD_H
#define __DEMOD_H

#include "system.h"   /* u8 / s8 / u16 / s16 / u32 / s32 定义 */

/*------------------------------------------------------------
 * 系统参数（与 main.c 保持一致）
 *------------------------------------------------------------*/
#define DEMOD_SPS           8       /* 每符号采样数 (Samples Per Symbol) */
#define DEMOD_FC_IDX_STEP   4       /* 载波 LUT 步进（32点LUT, Fc/Fs=1/8） */
#define DEMOD_FIR_LEN       15      /* FIR 滤波器阶数（奇数，对称） */

/*------------------------------------------------------------
 * 调制方式（与 main.c mod_idx 一致）
 *------------------------------------------------------------*/
#define MOD_BPSK    0
#define MOD_QPSK    1
#define MOD_16QAM   2

/*------------------------------------------------------------
 * 解调状态结构体
 *------------------------------------------------------------*/
typedef struct {
    /* --- 载波相位 --- */
    u8   carrier_idx;          /* 当前载波 LUT 索引 [0,31] */

    /* --- FIR 环形缓冲区 (I/Q 各一份) --- */
    s16  fir_buf_i[DEMOD_FIR_LEN];
    s16  fir_buf_q[DEMOD_FIR_LEN];
    u8   fir_ptr;              /* 环形缓冲写指针 */

    /* --- 符号同步计数器 --- */
    u8   sym_cnt;              /* 0 ~ SPS-1 */

    /* --- 抽样保持值 --- */
    s16  sample_i;
    s16  sample_q;

    /* --- 判决结果 --- */
    u8   last_bits;            /* 最近一次判决输出的比特（BPSK=1bit, QPSK=2bit, QAM=4bit） */
    u8   last_nbits;           /* 本次判决输出了几个比特 */

    /* --- BER 统计 --- */
    u32  ber_total;
    u32  ber_errors;
    u8   pn_state;             /* PN7 LFSR 状态 */

    /* --- 调制方式 --- */
    u8   mod_idx;

} DemodState_t;

/*------------------------------------------------------------
 * 外部接口
 *------------------------------------------------------------*/

/**
 * @brief  初始化解调状态
 * @param  st      解调状态指针
 * @param  mod_idx 调制方式 MOD_BPSK / MOD_QPSK / MOD_16QAM
 */
void Demod_Init(DemodState_t *st, u8 mod_idx);

/**
 * @brief  推入一个新采样点（每个 ADC 采样调用一次）
 * @param  st      解调状态指针
 * @param  sample  输入样本，范围 [-128,127]
 *
 * 内部流程：
 *   乘载波 → FIR滤波 → 符号同步计数 → 抽样判决 → BER统计
 *   每产生一个判决符号，调用 Demod_OnBit() 回调
 */
void Demod_PushSample(DemodState_t *st, s8 sample);

/**
 * @brief  读取当前 BER（返回 Q20 定点，即 ber * 2^20）
 *         调用方：ber_float = (float)Demod_GetBER_Q20(st) / (1<<20)
 */
u32  Demod_GetBER_Q20(DemodState_t *st);

/**
 * @brief  重置 BER 统计计数器
 */
void Demod_ResetBER(DemodState_t *st);

/**
 * @brief  用户回调：每判决出一个符号时被调用
 * @param  bits    判决输出的比特组合（LSB first）
 * @param  nbits   比特数（BPSK=1, QPSK=2, 16QAM=4）
 * @param  si      I路判决后的整数电平（调试用）
 * @param  sq      Q路判决后的整数电平
 *
 * 注意：此函数需在应用层（main.c 或其他 .c）中实现。
 */
void Demod_OnBit(u8 bits, u8 nbits, s16 si, s16 sq);

#endif /* __DEMOD_H */
