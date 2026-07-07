/*******************************************************************************
 * 数字基带系统 —— 调制解调与误码率测试系统
 * STM32F407ZGT6 + ILI9481  320x480  竖屏
 *
 * 输入方式：触摸屏（电阻屏，XPT2046）+ 物理按键（备用）
 *
 * 触摸交互：
 *   底部导航栏（左/中/右三格）：点击切换页面或执行功能
 *   PAGE_INFO  : 调制按钮区可点击切换调制方式
 *                SNR滑动条可拖动调节
 *   PAGE_WAVEFORM/SPECTRUM/IQ/EYE/BER : 左格=上一页  右格=下一页
 *
 * 物理按键（备用，无触摸时使用）：
 *   KEY0 (PA0): 下一页
 *   KEY1 (PE3): 上一页 / 调制-
 *   KEY2 (PE4): 调制+
 *******************************************************************************/

#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h"
#include "touch.h"
#include "stm32f4xx_spi.h"
#include "demod.h"

#include <math.h>     // 为 powf() 使用
#include <stdlib.h>   // 为 rand() 使用

/*============================================================
 * 物理按键定义（备用）
 *============================================================*/
#define KEY0_GPIO       GPIOA
#define KEY0_PIN        GPIO_Pin_0
#define KEY0_RCC        RCC_AHB1Periph_GPIOA
#define KEY1_GPIO       GPIOE
#define KEY1_PIN        GPIO_Pin_3
#define KEY1_RCC        RCC_AHB1Periph_GPIOE
#define KEY2_GPIO       GPIOE
#define KEY2_PIN        GPIO_Pin_4
#define KEY2_RCC        RCC_AHB1Periph_GPIOE
#define KEY0_PRESSED()  (GPIO_ReadInputDataBit(KEY0_GPIO, KEY0_PIN) == Bit_RESET)
#define KEY1_PRESSED()  (GPIO_ReadInputDataBit(KEY1_GPIO, KEY1_PIN) == Bit_RESET)
#define KEY2_PRESSED()  (GPIO_ReadInputDataBit(KEY2_GPIO, KEY2_PIN) == Bit_RESET)

/*============================================================
 * 屏幕布局  (竖屏 320x480)
 *============================================================*/
#define SCR_W           320
#define SCR_H           480
#define TITLE_H         36
#define TITLE_Y         0
#define STATUS_H        28
#define STATUS_Y        (SCR_H - STATUS_H)
#define CONTENT_Y       TITLE_H
#define CONTENT_H       (SCR_H - TITLE_H - STATUS_H)

/* 底部导航三格坐标 */
#define NAV_LEFT_X1     0
#define NAV_LEFT_X2     105
#define NAV_MID_X1      106
#define NAV_MID_X2      213
#define NAV_RIGHT_X1    214
#define NAV_RIGHT_X2    319
#define NAV_Y1          STATUS_Y
#define NAV_Y2          (SCR_H - 1)

/* INFO页各区域 Y 范围（含内容区偏移） */
#define INFO_MOD_Y1     (CONTENT_Y + 8)
#define INFO_MOD_Y2     (CONTENT_Y + 59)
#define INFO_SNR_Y1     (CONTENT_Y + 68)
#define INFO_SNR_Y2     (CONTENT_Y + 117)
#define INFO_SNR_TRACK_Y (CONTENT_Y + 96)   /* 滑条中心Y */
#define INFO_SNR_TRACK_X1 14
#define INFO_SNR_TRACK_X2 (SCR_W - 14)

/*============================================================
 * SPI 定义（推荐SPI2）
 *============================================================*/
#define SPIx                    SPI2
#define SPIx_RCC_APB            RCC_APB1Periph_SPI2
#define SPIx_GPIO_RCC           RCC_AHB1Periph_GPIOB

#define SPIx_SCK_GPIO           GPIOB
#define SPIx_SCK_PIN            GPIO_Pin_13
#define SPIx_SCK_AF             GPIO_AF_SPI2
#define SPIx_SCK_SOURCE         GPIO_PinSource13

#define SPIx_MISO_GPIO          GPIOB
#define SPIx_MISO_PIN           GPIO_Pin_14
#define SPIx_MISO_AF            GPIO_AF_SPI2
#define SPIx_MISO_SOURCE        GPIO_PinSource14

#define SPIx_MOSI_GPIO          GPIOB
#define SPIx_MOSI_PIN           GPIO_Pin_15
#define SPIx_MOSI_AF            GPIO_AF_SPI2
#define SPIx_MOSI_SOURCE        GPIO_PinSource15

#define SPIx_NSS_GPIO           GPIOB
#define SPIx_NSS_PIN            GPIO_Pin_12

/*============================================================
 * 颜色主题
 *============================================================*/
#define COLOR_BG         BLACK
#define COLOR_TITLE_BG   0x0311
#define COLOR_TITLE_FG   WHITE
#define COLOR_STATUS_BG  0x2945
#define COLOR_STATUS_FG  0xAD75
#define COLOR_PANEL_BG   0x10A2
#define COLOR_PANEL_FG   WHITE
#define COLOR_ACCENT     CYAN
#define COLOR_GOOD       GREEN
#define COLOR_WARN       YELLOW
#define COLOR_ERR        RED
#define COLOR_GRID       0x2945
#define COLOR_WAVE_BB    GREEN
#define COLOR_WAVE_MOD   CYAN
#define COLOR_WAVE_DEM   YELLOW
#define COLOR_IQ_TX      0x07FF
#define COLOR_IQ_RX      0xF81F
#define COLOR_BTN_ACT    0x4A69   /* 按钮按下高亮色 */

#define FFT_SIZE 256
#define SIM_SYMBOL_LEN  16

/*============================================================
 * 页面枚举
 *============================================================*/
typedef enum {
    PAGE_INFO = 0,
    PAGE_WAVEFORM,
    PAGE_SPECTRUM,
    PAGE_IQ,
    PAGE_EYE,
    PAGE_BER,
    PAGE_COUNT
} Page_t;

/*============================================================
 * 系统状态结构
 *============================================================*/
typedef struct {
    u8    mod_idx;        /* 0=BPSK  1=QPSK  2=16QAM */
    s16   snr_db;         /* SNR dB，范围 -2~20 */
    float ber;
    u32   err_bits;
    u32   total_bits;
    s8    wave_bb[200];
    s8    wave_mod[200];
    s8    wave_dem[200];
    s8    iq_tx_i[64];
    s8    iq_tx_q[64];
    s8    iq_rx_i[128];
    s8    iq_rx_q[128];
    s8    eye[60][16];
    float ber_curve_bpsk[12];
    float ber_curve_qpsk[12];
    float ber_curve_qam[12];
		float fft_mag[FFT_SIZE/2];      // FFT幅度谱（调制后信号）
    float fft_mag_bb[FFT_SIZE/2];   /* FFT幅度谱（基带成形信号） */
    u8        spectrum_ready;
} SysState_t;

/*============================================================
 * 触摸事件结构
 *============================================================*/
typedef struct {
    u8  pressed;       /* 1=当前按下 */
    u8  just_pressed;  /* 1=本轮刚按下（上升沿） */
    u8  just_released; /* 1=本轮刚松开（下降沿） */
    u8  dragging;      /* 1=正在拖动（持续按下+移动） */
    u16 x;
    u16 y;
    u16 press_x;       /* 按下时的起始X */
    u16 press_y;       /* 按下时的起始Y */
    u8  prev_sta;      /* 上一帧状态 */
} TouchEvent_t;

/*============================================================
 * 全局变量
 *============================================================*/
static Page_t      g_page        = PAGE_INFO;
static SysState_t  g_state;
static u8          g_need_redraw = 1;
static TouchEvent_t g_touch;
static DemodState_t  g_demod;
static s8            g_adc_buf[64];
static u8            g_adc_buf_valid = 0;
/* SNR 拖动状态 */
static u8  g_snr_dragging = 0;   /* 正在拖动SNR滑条 */

static const char * const MOD_NAMES[3] = {"BPSK", "QPSK", "16QAM"};

/*============================================================
 * 函数声明
 *============================================================*/
static void Key_Init(void);
static u8   Key_Scan(void);
static void Touch_Update(void);
static u8   Touch_InRect(u16 x1, u16 y1, u16 x2, u16 y2);
static void SysState_Demo_Fill(void);

static void UI_DrawTitleBar(const char *title);
static void UI_DrawStatusBar(void);
static void UI_DrawNavBar(const char *left, const char *mid, const char *right);
static void UI_DrawNavHighlight(u8 zone); /* 0=左 1=中 2=右 */
static void UI_DrawWaveform(u16 x, u16 y, u16 w, u16 h,
                            s8 *data, u16 len, u16 color, const char *label);
static void UI_DrawIQPlot(u16 cx, u16 cy, u16 r,
                          s8 *ti, s8 *tq, u8 tlen,
                          s8 *ri, s8 *rq, u8 rlen);
static void UI_DrawEyeDiagram(u16 x, u16 y, u16 w, u16 h);
static void UI_DrawBERCurve(u16 x, u16 y, u16 w, u16 h);
static void UI_DrawSpectrum(u16 x, u16 y, u16 w, u16 h,
                            float *mag, u16 color, const char *label);

/* INFO页子组件 */
static void Info_DrawModButtons(void);
static void Info_DrawSNRSlider(void);
static void Info_DrawMeasurements(u16 cy);
static void Info_DrawParams(u16 cy);

static void Page_DrawInfo(void);
static void Page_DrawWaveform(void);
static void Page_DrawSpectrum(void);
static void Page_DrawIQ(void);
static void Page_DrawEye(void);
static void Page_DrawBER(void);
static void Page_Update(void);

/* 触摸处理 */
static void Touch_HandleInfo(void);
static void Touch_HandleNav(void);
static void Touch_Process(void);

static void Page_HandleKey(u8 key);


void Demod_OnBit(u8 bits, u8 nbits, s16 si, s16 sq)
{
    static u16 dem_idx = 0u;
    float ber_q20;

    /* a) 刷新 BER 显示值 */
    ber_q20 = (float)Demod_GetBER_Q20(&g_demod) / (float)(1u << 20);
    g_state.ber = ber_q20;

    /* 同步更新错误比特计数（用于 Info 页显示） */
    g_state.err_bits   = (u32)g_demod.ber_errors;
    g_state.total_bits = (u32)g_demod.ber_total;

    /* b) 把判决后的 I 路电平写入 wave_dem 数组（供 Waveform 页显示） */
    /*    si 范围约 ±127，wave_dem 类型 s8，直接赋值即可 */
    if (dem_idx < 200u) {
        g_state.wave_dem[dem_idx] = (s8)(si > 127 ? 127 : si < -127 ? -127 : si);
        dem_idx++;
    } else {
        dem_idx = 0u;   /* 循环覆盖 */
    }

    /* c) 同步更新 IQ 接收散点（供 IQ 页显示） */
    {
        static u8 iq_idx = 0u;
        g_state.iq_rx_i[iq_idx] = (s8)si;
        g_state.iq_rx_q[iq_idx] = (s8)sq;
        iq_idx = (u8)((iq_idx + 1u) & 0x7Fu);  /* 循环 128 点 */
    }

    /* d) 串口打印（每 100 个符号打印一次，避免刷屏） */
    {
        static u32 print_cnt = 0u;
        print_cnt++;
        if (print_cnt >= 100u) {
            print_cnt = 0u;
            printf("Demod: bits=%02X nbits=%d I=%d Q=%d BER=%.2e ErrBit=%lu\r\n",
                   (unsigned)bits, (int)nbits,
                   (int)si, (int)sq,
                   (double)ber_q20,
                   (unsigned long)g_demod.ber_errors);
        }
    }
}


/*============================================================
 * SPI 初始化与通信函数
 *============================================================*/
static void SPI_Init_User(void);
static u8 SPI_SendByte(u8 byte);
static void SPI_SendBuf(u8 *buf, u16 len);
static void Comm_With_FPGA(void);
static void FFT_ComputeSpectrum(void);

/* SPI 初始化 */
static void SPI_Init_User(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(SPIx_GPIO_RCC, ENABLE);
    RCC_APB1PeriphClockCmd(SPIx_RCC_APB, ENABLE);

    GPIO_PinAFConfig(SPIx_SCK_GPIO,  SPIx_SCK_SOURCE,  SPIx_SCK_AF);
    GPIO_PinAFConfig(SPIx_MISO_GPIO, SPIx_MISO_SOURCE, SPIx_MISO_AF);
    GPIO_PinAFConfig(SPIx_MOSI_GPIO, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN | SPIx_MISO_PIN | SPIx_MOSI_PIN;
    GPIO_Init(SPIx_SCK_GPIO, &GPIO_InitStructure);

    /* NSS 手动控制 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = SPIx_NSS_PIN;
    GPIO_Init(SPIx_NSS_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(SPIx_NSS_GPIO, SPIx_NSS_PIN);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStructure);

    SPI_Cmd(SPIx, ENABLE);
}

static u8 SPI_SendByte(u8 byte)
{
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPIx, byte);
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPIx);
}

static void SPI_SendBuf(u8 *buf, u16 len)
{
    u16 i;
    GPIO_ResetBits(SPIx_NSS_GPIO, SPIx_NSS_PIN);
    for (i = 0; i < len; i++) {
        SPI_SendByte(buf[i]);
    }
    GPIO_SetBits(SPIx_NSS_GPIO, SPIx_NSS_PIN);
}

/* 与FPGA通信示例 */
static void Comm_With_FPGA(void)
{
    u8  tx_buf[66];   /* [0]=命令字  [1]=mod_idx  [2]=snr_db  [3..66]=pad */
    u8  rx_buf[66];
    u16 i;

    /* --- 构造发送帧 ---
     *   Byte 0: 命令字 0xA5（起始标志）
     *   Byte 1: mod_idx（0/1/2）
     *   Byte 2: snr_db  偏移编码（+32，使范围变为正值）
     *   Byte 3~65: 0x00 占位（FPGA利用此窗口回传64字节ADC数据）
     */
    tx_buf[0] = 0xA5u;
    tx_buf[1] = g_state.mod_idx;
    tx_buf[2] = (u8)((s8)g_state.snr_db + 32);
    for (i = 3u; i < 66u; i++) tx_buf[i] = 0x00u;

    /* --- SPI 全双工收发 --- */
    GPIO_ResetBits(SPIx_NSS_GPIO, SPIx_NSS_PIN);
    for (i = 0u; i < 66u; i++) {
        rx_buf[i] = SPI_SendByte(tx_buf[i]);
    }
    GPIO_SetBits(SPIx_NSS_GPIO, SPIx_NSS_PIN);

    /* --- 校验帧头（FPGA应在 rx_buf[0] 回应 0x5A） --- */
    if (rx_buf[0] != 0x5Au) {
        printf("FPGA frame error: header=0x%02X\r\n", (unsigned)rx_buf[0]);
        /* 帧头错误时仍继续（保持波形连续） */
    }

    /* --- 处理64字节ADC数据（rx_buf[2..65]） ---
     *   FPGA 发回的是 uint8 格式（无符号0~255），
     *   转换为 s8（减128）后送解调器
     */
    for (i = 2u; i < 66u; i++) {
        s8 sample = (s8)((s16)rx_buf[i] - 128);

        /* 更新 wave_mod 显示缓冲（调制后波形来自FPGA） */
        {
            static u16 mod_idx_w = 0u;
            if (mod_idx_w < 200u) {
                g_state.wave_mod[mod_idx_w] = sample;
                mod_idx_w++;
            } else {
                mod_idx_w = 0u;
            }
        }

        /* 送入解调器（混频→FIR→判决→BER→回调） */
        Demod_PushSample(&g_demod, sample);
    }

    /* --- 若调制方式改变，重新初始化解调器 --- */
    if (g_demod.mod_idx != g_state.mod_idx) {
        Demod_Init(&g_demod, g_state.mod_idx);
        Demod_ResetBER(&g_demod);
        printf("Demod reinit: Mod=%s\r\n", MOD_NAMES[g_state.mod_idx]);
    }
}

/*============================================================
 * FFT 频谱计算（不依赖 CMSIS-DSP，简易 radix-2 DIT 实现）
 *
 * FFT_Run() 是通用核心：对任意 200 点 s8 时域缓冲区
 *   （加 Hanning 窗后零填充到 FFT_SIZE=256 点）做FFT，
 *   输出幅度谱到 out_mag[0 .. FFT_SIZE/2-1]（对应 0 ~ Fs/2）。
 *
 * FFT_ComputeSpectrum() 分别对基带信号(wave_bb)和调制后信号(wave_mod)
 *   各算一次，对应STM32液晶上要显示的两幅频谱图：
 *     1) 基带成形信号频谱（FPGA输出）  -> g_state.fft_mag_bb
 *     2) 调制后射频信号频谱            -> g_state.fft_mag
 *
 * 计算量：每次256点FFT，8级蝶形运算，STM32F407 (Cortex-M4F，硬件FPU)
 *         单次耗时很短，仅在 PAGE_SPECTRUM 页面可见时才周期性调用，
 *         避免不必要的CPU开销。
 *============================================================*/
static void FFT_Run(s8 *time_data, u16 valid_len, float *out_mag)
{
    static float re[FFT_SIZE];
    static float im[FFT_SIZE];
    u16   i, j, k;
    u16   bit;
    u16   len;
    u16   half;
    float tr, ti;
    float ang, wr_step, wi_step;
    float cur_wr, cur_wi;
    float u_re, u_im, v_re, v_im;
    float new_wr, new_wi;
    float win;

    /* 1. 取样 + 加 Hanning 窗（抑制频谱泄漏） + 零填充到 FFT_SIZE 点 */
    for (i = 0; i < FFT_SIZE; i++) {
        if (i < valid_len) {
            win = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * (float)i / (float)(valid_len - 1u));
            re[i] = (float)time_data[i] * win;
        } else {
            re[i] = 0.0f;
        }
        im[i] = 0.0f;
    }

    /* 2. 位反转重排（in-place bit-reversal permutation） */
    j = 0u;
    for (i = 1u; i < FFT_SIZE; i++) {
        bit = FFT_SIZE >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if (i < j) {
            tr = re[i]; re[i] = re[j]; re[j] = tr;
            ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }

    /* 3. 蝶形运算（Cooley-Tukey, DIT, radix-2），共 log2(FFT_SIZE) 级 */
    for (len = 2u; len <= FFT_SIZE; len = (u16)(len << 1)) {
        half    = (u16)(len >> 1);
        ang     = -2.0f * 3.14159265f / (float)len;
        wr_step = cosf(ang);
        wi_step = sinf(ang);

        for (i = 0u; i < FFT_SIZE; i = (u16)(i + len)) {
            cur_wr = 1.0f;
            cur_wi = 0.0f;
            for (j = 0u; j < half; j++) {
                u_re = re[i + j];
                u_im = im[i + j];
                v_re = re[i + j + half] * cur_wr - im[i + j + half] * cur_wi;
                v_im = re[i + j + half] * cur_wi + im[i + j + half] * cur_wr;

                re[i + j]        = u_re + v_re;
                im[i + j]        = u_im + v_im;
                re[i + j + half] = u_re - v_re;
                im[i + j + half] = u_im - v_im;

                new_wr = cur_wr * wr_step - cur_wi * wi_step;
                new_wi = cur_wr * wi_step + cur_wi * wr_step;
                cur_wr = new_wr;
                cur_wi = new_wi;
            }
        }
    }

    /* 4. 幅度谱（只取前 FFT_SIZE/2 点，对应 0~Fs/2，频谱关于Fs/2对称） */
    for (k = 0u; k < FFT_SIZE/2; k++) {
        out_mag[k] = sqrtf(re[k]*re[k] + im[k]*im[k]);
    }
}

static void FFT_ComputeSpectrum(void)
{
    FFT_Run(g_state.wave_bb,  200u, g_state.fft_mag_bb);  /* 基带成形信号频谱 */
    FFT_Run(g_state.wave_mod, 200u, g_state.fft_mag);     /* 调制后射频信号频谱 */
    g_state.spectrum_ready = 1u;
}

/*============================================================
 * 无FPGA纯软件仿真（不依赖CMSIS-DSP）
 * 实现调制、加噪、解调、动态眼图、简易频谱
 *============================================================*/
static void Software_Signal_Simulation(void)
{
    u16 i, j;
//		u16 idx;
    float noise_amp = powf(10.0f, -g_state.snr_db / 20.0f) * 45.0f;

    /* 1. 基带信号 */
    for (i = 0; i < 200; i++) {
        g_state.wave_bb[i] = ((i / 18) % 2 == 0) ? 75 : -75;
    }

    /* 2. 数字调制 */
    for (i = 0; i < 200; i++) {
        u8 bit = (i / 16) % 2;
        switch (g_state.mod_idx) {
            case 0:  /* BPSK */
                g_state.wave_mod[i] = bit ? 85 : -85; 
                break;
            case 1:  /* QPSK */
                g_state.wave_mod[i] = bit ? 70 : -70 + ((i%4>1)?25:-25); 
                break;
            case 2:  /* 16QAM */
                g_state.wave_mod[i] = (bit*40 - 20) + ((i%5)-2)*18; 
                break;
        }
    }

    /* 3. 信道加噪 */
    for (i = 0; i < 200; i++) {
        float noise = ((rand() % 2000) - 1000) * noise_amp / 1000.0f;
        g_state.wave_mod[i] += (s8)noise;
    }

    /* 4. 送入解调器 */
    for (i = 0; i < 180; i += 3) {
        Demod_PushSample(&g_demod, g_state.wave_mod[i]);
    }

    /* 5. 更新解调后信号 */
    for (i = 0; i < 200; i++) {
        g_state.wave_dem[i] = (g_state.wave_mod[i] + g_state.wave_mod[(i+1)%200]) / 2;
    }

//    /* 6. 动态眼图（基于解调后信号） */
//    for (i = 0; i < 60; i++) {
//        for (j = 0; j < 16; j++) {
//            idx = (i * 5 + j) % 200;
//            g_state.eye[i][j] = g_state.wave_dem[idx] + (rand() % 13 - 6);
//        }
//    }

    /* 7. 频谱：基于当前 wave_bb / wave_mod 计算真实FFT幅度谱（两路） */
    FFT_ComputeSpectrum();

    /* 8. 更新BER */
    g_state.ber = powf(10.0f, -g_state.snr_db / 9.0f) * 0.18f;
    if (g_state.ber > 0.45f) g_state.ber = 0.45f;
    g_state.err_bits = (u32)(g_state.ber * 80000);
    g_state.total_bits = 80000;
		/* 9. 动态更新BER理论曲线（三条全部重算） */
		{
				u8 k;
				float snr_linear, ber_val;

				for (k = 0; k < 12; k++) {
						float snr_point = k * 2.0f;   /* SNR点：0,2,4,...,22 dB */

						/* BPSK: BER = 0.5 * erfc(sqrt(Eb/N0))
						 * Eb/N0(linear) = 10^(SNR/10) */
						snr_linear = powf(10.0f, snr_point / 10.0f);
						ber_val = 0.5f * erfcf(sqrtf(snr_linear));
						if (ber_val < 1e-13f) ber_val = 1e-13f;
						g_state.ber_curve_bpsk[k] = ber_val;

						/* QPSK: BER = 0.5 * erfc(sqrt(Eb/N0))，
						 * 但 Eb/N0 = Es/N0 / 2（每符号2比特），
						 * 等价于在SNR轴右移3dB，曲线与BPSK明显分离 */
						snr_linear = powf(10.0f, snr_point / 10.0f) * 0.5f;
						ber_val = 0.5f * erfcf(sqrtf(snr_linear));
						if (ber_val < 1e-13f) ber_val = 1e-13f;
						g_state.ber_curve_qpsk[k] = ber_val;

						/* 16QAM: BER ≈ (3/8) * erfc(sqrt(0.1 * Eb/N0)) */
						snr_linear = powf(10.0f, snr_point / 10.0f);
						ber_val = (3.0f / 8.0f) * erfcf(sqrtf(0.1f * snr_linear));
						if (ber_val < 1e-13f) ber_val = 1e-13f;
						g_state.ber_curve_qam[k] = ber_val;
				}
		}
}

/*============================================================
 * 主函数
 *============================================================*/
int main(void)
{
    u32 tick = 0;
    u8  key;

    SysTick_Init(168);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    LED_Init();
    USART1_Init(115200);
    Key_Init();
    TFTLCD_Init();
    TP_Init();                  /* 触摸屏初始化 */
		SPI_Init_User();
		printf("SPI2 Initialized for FPGA. \r\n");
		Demod_Init(&g_demod, g_state.mod_idx);
		printf("Demod initialized. Mod=%s\r\n", MOD_NAMES[g_state.mod_idx]); 
	
    FRONT_COLOR = WHITE;
    BACK_COLOR  = COLOR_BG;
    LCD_Clear(COLOR_BG);

    SysState_Demo_Fill();
    g_need_redraw = 1;

    printf("LCD UI Ready. Touch enabled.\r\n");

    while (1)
    {
        /* 1. 更新触摸状态 */
        tp_dev.scan(0);
        Touch_Update();

        /* 2. 触摸事件处理 */
        if (g_touch.just_pressed || g_touch.dragging || g_touch.just_released)
            Touch_Process();

        /* 3. 物理按键（备用） */
        key = Key_Scan();
        if (key)
            Page_HandleKey(key);

        /* 4. 周期刷新状态栏 */
        tick++;
        if (tick >= 50)
        {
            tick = 0;
            if (!g_need_redraw)
                UI_DrawStatusBar();
        }

        /* 5. 整页重绘 */
        if (g_need_redraw)
        {
            g_need_redraw = 0;
            LCD_Clear(COLOR_BG);
            Page_Update();
        }
        /* 无FPGA时使用软件仿真 */
        if (tick % 12 == 0) {
            Comm_With_FPGA();
            if (g_page == PAGE_WAVEFORM || g_page == PAGE_EYE) {
                g_need_redraw = 1;
            }
            if (g_page == PAGE_SPECTRUM) {
                /* 频谱页可见时才计算FFT，节省CPU */
                FFT_ComputeSpectrum();
                g_need_redraw = 1;
            }
        }
        delay_ms(10);
        LED1 = !LED1;
    }
}

/*============================================================
 * 按键初始化
 *============================================================*/
static void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(KEY0_RCC | KEY1_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin   = KEY0_PIN;
    GPIO_Init(KEY0_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = KEY1_PIN | KEY2_PIN;
    GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);
}

static u8 Key_Scan(void)
{
    static u8 key_state = 0;
    u8 key_val = 0;
    if      (KEY0_PRESSED()) key_val = 1;
    else if (KEY1_PRESSED()) key_val = 2;
    else if (KEY2_PRESSED()) key_val = 3;
    if (key_val && !key_state) { key_state = key_val; delay_ms(20); return key_val; }
    if (!key_val) key_state = 0;
    return 0;
}

/*============================================================
 * 触摸状态更新
 *============================================================*/
static void Touch_Update(void)
{
    u8 cur_sta = (tp_dev.sta & TP_PRES_DOWN) ? 1 : 0;

    g_touch.just_pressed  = 0;
    g_touch.just_released = 0;

    if (cur_sta)
    {
        g_touch.x = tp_dev.x[0];
        g_touch.y = tp_dev.y[0];

        if (!g_touch.prev_sta)
        {
            /* 新按下 */
            g_touch.just_pressed = 1;
            g_touch.press_x = g_touch.x;
            g_touch.press_y = g_touch.y;
            g_touch.dragging = 0;
            g_snr_dragging   = 0;
        }
        else
        {
            /* 持续按下：判断是否拖动（移动超过8px） */
            s16 dx = (s16)g_touch.x - (s16)g_touch.press_x;
            s16 dy = (s16)g_touch.y - (s16)g_touch.press_y;
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            if (dx > 8 || dy > 8)
                g_touch.dragging = 1;
        }
        g_touch.pressed = 1;
    }
    else
    {
        if (g_touch.prev_sta)
            g_touch.just_released = 1;
        g_touch.pressed  = 0;
        g_touch.dragging = 0;
        if (g_touch.just_released)
            g_snr_dragging = 0;
    }

    g_touch.prev_sta = cur_sta;
}

/* 判断触摸点是否在矩形内 */
static u8 Touch_InRect(u16 x1, u16 y1, u16 x2, u16 y2)
{
    return (g_touch.x >= x1 && g_touch.x <= x2 &&
            g_touch.y >= y1 && g_touch.y <= y2) ? 1 : 0;
}

/*============================================================
 * 演示数据填充
 *============================================================*/
static void SysState_Demo_Fill(void)
{
    u8 i, j;
    static const s8  qpsk_i[4]   = { 45,  45, -45, -45};
    static const s8  qpsk_q[4]   = { 45, -45,  45, -45};
    static const s8  noise_i[8]  = { 5, -3,  7, -5,  4, -6,  8, -4};
    static const s8  noise_q[8]  = { 3, -5,  4, -7,  6, -4,  5, -8};
    static const s8  sin_tbl[32] = { 0,13,25,37,47,56,63,68,
                                     70,68,63,56,47,37,25,13,
                                      0,-13,-25,-37,-47,-56,-63,-68,
                                     -70,-68,-63,-56,-47,-37,-25,-13};

    g_state.mod_idx    = 1;
    g_state.snr_db     = 10;
    g_state.ber        = 0.00123f;
    g_state.err_bits   = 123;
    g_state.total_bits = 100000;

    for (i = 0; i < 200; i++) g_state.wave_bb[i]  = ((i/16)%2) ? 60 : -60;
    for (i = 0; i < 200; i++) g_state.wave_mod[i] = sin_tbl[i%32];
    for (i = 0; i < 200; i++) g_state.wave_dem[i] = g_state.wave_bb[i]
                                                     + (s8)(((u16)i*7u)%11u) - 5;

    for (i = 0; i < 64;  i++) {
        g_state.iq_tx_i[i] = qpsk_i[i%4];
        g_state.iq_tx_q[i] = qpsk_q[i%4];
    }
    for (i = 0; i < 128; i++) {
        g_state.iq_rx_i[i] = qpsk_i[i%4] + noise_i[i%8];
        g_state.iq_rx_q[i] = qpsk_q[i%4] + noise_q[i%8];
    }

				/* 固定眼图：模拟60条迹线，每条16个采样点
		 * j=0~3: 上升/下降沿过渡区
		 * j=4~11: 稳定张开区（眼睛睁开）
		 * j=12~15: 下一个沿过渡区
		 * 噪声用确定性伪随机（不用rand），保证每次一样 */
		for (i = 0; i < 60; i++) {
				/* 每条迹线的极性：奇数条从负到正，偶数条从正到负 */
				s8 start_level = (i % 2 == 0) ? -55 : 55;
				s8 end_level   = -start_level;

				for (j = 0; j < 16; j++) {
						s8 val;
						/* 确定性噪声（不用rand，保证重绘不变） */
						s8 noise = (s8)(((u16)i * 7u + (u16)j * 13u + 3u) % 13u) - 6;

						if (j <= 3) {
								/* 过渡沿：线性插值从 start 到 end */
								val = (s8)(start_level + (s8)((s16)(end_level - start_level) * j / 4));
						} else if (j <= 11) {
								/* 稳定区：保持 end_level，加少量噪声 */
								val = end_level + noise;
						} else {
								/* 后沿过渡区：保持稳定（下一符号沿在下一个眼图周期） */
								val = end_level + noise / 2;
						}
						g_state.eye[i][j] = val;
				}
		}

    /* BER 理论曲线（公式计算，三条曲线各不相同） */
    {
        u8 k;
        float snr_linear, ber_val;
        for (k = 0; k < 12; k++) {
            float snr_point = k * 2.0f;   /* 0, 2, 4, ... 22 dB */

            /* BPSK */
            snr_linear = powf(10.0f, snr_point / 10.0f);
            ber_val = 0.5f * erfcf(sqrtf(snr_linear));
            if (ber_val < 1e-13f) ber_val = 1e-13f;
            g_state.ber_curve_bpsk[k] = ber_val;

            /* QPSK：每符号2比特，Eb/N0 = Es/N0 / 2，曲线右移约3dB */
            snr_linear = powf(10.0f, snr_point / 10.0f) * 0.5f;
            ber_val = 0.5f * erfcf(sqrtf(snr_linear));
            if (ber_val < 1e-13f) ber_val = 1e-13f;
            g_state.ber_curve_qpsk[k] = ber_val;

            /* 16QAM */
            snr_linear = powf(10.0f, snr_point / 10.0f);
            ber_val = (3.0f / 8.0f) * erfcf(sqrtf(0.1f * snr_linear));
            if (ber_val < 1e-13f) ber_val = 1e-13f;
            g_state.ber_curve_qam[k] = ber_val;
        }
    }

    /* 初始频谱：直接基于上面填充的 wave_mod 跑一次FFT，避免Spectrum页首次进入空白 */
    FFT_ComputeSpectrum();
}
/*============================================================
 * 通用 UI 组件
 *============================================================*/
static void UI_DrawTitleBar(const char *title)
{
    char page_str[4];
    LCD_Fill(0, TITLE_Y, SCR_W-1, TITLE_Y+TITLE_H-1, COLOR_TITLE_BG);
    LCD_Fill(0, TITLE_Y, 4,       TITLE_Y+TITLE_H-1, COLOR_ACCENT);
    FRONT_COLOR = COLOR_TITLE_FG; BACK_COLOR = COLOR_TITLE_BG;
    LCD_ShowString(10, TITLE_Y+8, 200, 24, 16, (u8*)title);
    page_str[0] = (char)('1'+(u8)g_page);
    page_str[1] = '/'; page_str[2] = '0'+PAGE_COUNT; page_str[3] = '\0';
    LCD_ShowString(SCR_W-30, TITLE_Y+10, 30, 16, 12, (u8*)page_str);
}

static void UI_DrawStatusBar(void)
{
    char snr_str[12];
    u8   si;
    s16  abs_snr;
    u16  ber_color;
    float b;
    u8   exp_cnt;
    char ber_str[10];
    u8   bi;

    LCD_Fill(0, STATUS_Y, SCR_W-1, SCR_H-1, COLOR_STATUS_BG);
    FRONT_COLOR = COLOR_STATUS_FG; BACK_COLOR = COLOR_STATUS_BG;
    LCD_ShowString(4, STATUS_Y+6, 60, 16, 12, (u8*)MOD_NAMES[g_state.mod_idx]);

    snr_str[0]='S'; snr_str[1]='N'; snr_str[2]='R'; snr_str[3]=':';
    si = 4; abs_snr = g_state.snr_db;
    if (abs_snr < 0) { snr_str[si++]='-'; abs_snr=-abs_snr; }
    if (abs_snr >= 10) snr_str[si++]=(char)('0'+abs_snr/10);
    snr_str[si++]=(char)('0'+abs_snr%10);
    snr_str[si++]='d'; snr_str[si++]='B'; snr_str[si]='\0';
    LCD_ShowString(SCR_W/2-28, STATUS_Y+6, 60, 16, 12, (u8*)snr_str);

    b = g_state.ber; exp_cnt = 0;
    while (b < 0.1f && exp_cnt < 9) { b *= 10.0f; exp_cnt++; }
    ber_str[0]='<'; ber_str[1]='1'; ber_str[2]='E'; ber_str[3]='-'; bi=4;
    ber_str[bi++]=(char)('0'+exp_cnt); ber_str[bi]='\0';
    ber_color = (g_state.ber < 1e-4f) ? COLOR_GOOD :
                (g_state.ber < 1e-2f) ? COLOR_WARN : COLOR_ERR;
    FRONT_COLOR = ber_color;
    LCD_ShowString(SCR_W-54, STATUS_Y+6, 50, 16, 12, (u8*)ber_str);
}

/* 绘制底部导航栏（三格按钮） */
static void UI_DrawNavBar(const char *left, const char *mid, const char *right)
{
    LCD_Fill(0, NAV_Y1, SCR_W-1, NAV_Y2, COLOR_STATUS_BG);

    /* 分隔线 */
    LCD_DrawLine_Color(NAV_MID_X1-1, NAV_Y1, NAV_MID_X1-1, NAV_Y2, COLOR_GRID);
    LCD_DrawLine_Color(NAV_RIGHT_X1-1, NAV_Y1, NAV_RIGHT_X1-1, NAV_Y2, COLOR_GRID);

    FRONT_COLOR = COLOR_ACCENT; BACK_COLOR = COLOR_STATUS_BG;
    LCD_ShowString(NAV_LEFT_X1+4,   NAV_Y1+7, 100, 16, 12, (u8*)left);
    LCD_ShowString(NAV_MID_X1+4,    NAV_Y1+7, 100, 16, 12, (u8*)mid);
    FRONT_COLOR = COLOR_WARN;
    LCD_ShowString(NAV_RIGHT_X1+4,  NAV_Y1+7, 100, 16, 12, (u8*)right);
}

/* 短暂高亮导航格（触摸反馈） */
static void UI_DrawNavHighlight(u8 zone)
{
    u16 x1, x2;
    if      (zone == 0) { x1 = NAV_LEFT_X1;  x2 = NAV_LEFT_X2;  }
    else if (zone == 1) { x1 = NAV_MID_X1;   x2 = NAV_MID_X2;   }
    else                { x1 = NAV_RIGHT_X1;  x2 = NAV_RIGHT_X2; }
    LCD_Fill(x1, NAV_Y1, x2, NAV_Y2, COLOR_BTN_ACT);
    delay_ms(80);
    /* 重绘状态栏（包含导航） */
    UI_DrawStatusBar();
}

/*============================================================
 * 波形绘制
 *============================================================*/
static void UI_DrawWaveform(u16 x, u16 y, u16 w, u16 h,
                            s8 *data, u16 len, u16 color, const char *label)
{
    u16 i, idx0, idx1, pts;
    u16 mid = y + h/2;
    s16 v0, v1;
    u16 y0, y1, x0, x1;

    pts = (len < (u16)(w-4)) ? len : (u16)(w-4);
    LCD_Fill(x, y, x+w-1, y+h-1, COLOR_PANEL_BG);
    LCD_DrawRectangle(x, y, x+w-1, y+h-1);
    LCD_DrawLine_Color(x+1, mid, x+w-2, mid, COLOR_GRID);

    for (i = 1; i < pts; i++) {
        idx0 = (u16)((u32)(i-1)*len/pts);
        idx1 = (u16)((u32)i*len/pts);
        v0   = (s16)data[idx0] * (s16)(h/2-2) / 127;
        v1   = (s16)data[idx1] * (s16)(h/2-2) / 127;
        y0   = (u16)((s16)mid - v0);
        y1   = (u16)((s16)mid - v1);
        x0   = x+2+(i-1); x1 = x+2+i;
        if (y0 < y) y0=y; if (y0 > y+h-1) y0=y+h-1;
        if (y1 < y) y1=y; if (y1 > y+h-1) y1=y+h-1;
        LCD_DrawLine_Color(x0, y0, x1, y1, color);
    }
    FRONT_COLOR = color; BACK_COLOR = COLOR_PANEL_BG;
    LCD_ShowString(x+3, y+2, 60, 12, 12, (u8*)label);
}

/*============================================================
 * IQ 星座图
 *============================================================*/
static void UI_DrawIQPlot(u16 cx, u16 cy, u16 r,
                          s8 *ti, s8 *tq, u8 tlen,
                          s8 *ri, s8 *rq, u8 rlen)
{
    u8  i;
    s16 px, py;
    u16 sx, sy;

    LCD_Fill(cx-r-2, cy-r-2, cx+r+2, cy+r+2, COLOR_PANEL_BG);
    LCD_DrawRectangle(cx-r-2, cy-r-2, cx+r+2, cy+r+2);
    LCD_DrawLine_Color(cx-r, cy,    cx+r, cy,    COLOR_GRID);
    LCD_DrawLine_Color(cx,   cy-r,  cx,   cy+r,  COLOR_GRID);

    for (i = 0; i < rlen; i++) {
        px=(s16)ri[i]*(s16)r/64; py=-(s16)rq[i]*(s16)r/64;
        sx=(u16)((s16)cx+px); sy=(u16)((s16)cy+py);
        if (sx<cx-r||sx>cx+r||sy<cy-r||sy>cy+r) continue;
        LCD_DrawFRONT_COLOR(sx,sy,COLOR_IQ_RX);
        if (sx+1<=cx+r) LCD_DrawFRONT_COLOR(sx+1,sy,COLOR_IQ_RX);
        if (sy+1<=cy+r) LCD_DrawFRONT_COLOR(sx,sy+1,COLOR_IQ_RX);
    }
    for (i = 0; i < tlen; i++) {
        px=(s16)ti[i]*(s16)r/64; py=-(s16)tq[i]*(s16)r/64;
        sx=(u16)((s16)cx+px); sy=(u16)((s16)cy+py);
        if (sx<cx-r||sx>cx+r||sy<cy-r||sy>cy+r) continue;
        LCD_Draw_Circle(sx,sy,3);
        LCD_DrawFRONT_COLOR(sx,sy,COLOR_IQ_TX);
    }
}

/*============================================================
 * 眼图
 *============================================================*/
static void UI_DrawEyeDiagram(u16 x, u16 y, u16 w, u16 h)
{
    u8  i, j;
    u16 mid = y+h/2;
    s16 v0, v1;
    u16 y0, y1, x0, x1;

    LCD_Fill(x,y,x+w-1,y+h-1,COLOR_PANEL_BG);
    LCD_DrawRectangle(x,y,x+w-1,y+h-1);
    LCD_DrawLine_Color(x+1,    mid,   x+w-2, mid,   COLOR_GRID);
    LCD_DrawLine_Color(x+w/2,  y+1,   x+w/2, y+h-2, COLOR_GRID);

    for (i = 0; i < 60; i++)
        for (j = 1; j < 16; j++) {
            v0=(s16)g_state.eye[i][j-1]*(s16)(h/2-2)/70;
            v1=(s16)g_state.eye[i][j]  *(s16)(h/2-2)/70;
            y0=(u16)((s16)mid-v0); y1=(u16)((s16)mid-v1);
            x0=x+2+(u16)(j-1)*(w-4)/16; x1=x+2+(u16)j*(w-4)/16;
            if (y0<y+1) y0=y+1; if (y0>y+h-2) y0=y+h-2;
            if (y1<y+1) y1=y+1; if (y1>y+h-2) y1=y+h-2;
            LCD_DrawLine_Color(x0,y0,x1,y1,0x03E0);
        }
    FRONT_COLOR=COLOR_GOOD; BACK_COLOR=COLOR_PANEL_BG;
    LCD_ShowString(x+3, y+2, 80, 12, 12, (u8*)"Eye Diagram");
    LCD_ShowString(x+3, y+h-14, 200, 12, 12, (u8*)"60 traces / 2 sym");
}

/*============================================================
 * 频谱图（通用面板：可绘制 g_state.fft_mag_bb 或 g_state.fft_mag）
 *============================================================*/
static void UI_DrawSpectrum(u16 x, u16 y, u16 w, u16 h,
                            float *mag, u16 color, const char *label)
{
    u16   i, pts, idx;
    u16   base_y, bar_h, bar_x;
    u16   peak_idx;
    float max_mag, val, norm, peak_mag;

    LCD_Fill(x, y, x+w-1, y+h-1, COLOR_PANEL_BG);
    LCD_DrawRectangle(x, y, x+w-1, y+h-1);

    base_y = (u16)(y + h - 16);   /* 底部留出坐标轴标签空间 */
    pts    = (u16)(w - 4);
    if (pts > (FFT_SIZE/2)) pts = (FFT_SIZE/2);

    /* 跳过直流分量(bin 0)寻找最大幅值，避免压缩交流分量的动态范围 */
    max_mag  = 1.0f;
    peak_mag = 0.0f;
    peak_idx = 0u;
    for (i = 1u; i < (FFT_SIZE/2); i++) {
        val = mag[i];
        if (val > max_mag)  max_mag  = val;
        if (val > peak_mag) { peak_mag = val; peak_idx = i; }
    }

    /* 频率轴网格（4条竖向网格线） */
    for (i = 1u; i < 4u; i++) {
        u16 gx = (u16)(x + 2 + (u32)i * (w-4) / 4);
        LCD_DrawLine_Color(gx, y+14, gx, base_y, COLOR_GRID);
    }

    /* 幅度谱柱状图 */
    for (i = 0; i < pts; i++) {
        idx = (u16)((u32)i * (FFT_SIZE/2) / pts);
        val = mag[idx];
        norm = val / max_mag;
        if (norm > 1.0f) norm = 1.0f;
        bar_h = (u16)(norm * (float)(h - 30));
        bar_x = (u16)(x + 2 + i);
        if (bar_h > 0u) {
            LCD_DrawLine_Color(bar_x, base_y, bar_x, (u16)(base_y - bar_h), color);
        }
    }

    /* 基线 */
    LCD_DrawLine_Color((u16)(x+1), base_y, (u16)(x+w-2), base_y, COLOR_GRID);

    FRONT_COLOR = COLOR_ACCENT; BACK_COLOR = COLOR_PANEL_BG;
    LCD_ShowString((u16)(x+3), (u16)(y+2), 140, 12, 12, (u8*)label);

    FRONT_COLOR = COLOR_STATUS_FG;
    LCD_ShowString((u16)(x+3),   (u16)(base_y+2), 30, 12, 12, (u8*)"0");
    LCD_ShowString((u16)(x+w-40),(u16)(base_y+2), 40, 12, 12, (u8*)"Fs/2");

    /* 标注峰值频率所在bin（用红点标记） */
    if (peak_mag > 0.0f) {
        u16 mark_x = (u16)(x + 2 + (u32)peak_idx * (w-4) / (FFT_SIZE/2));
        u16 mark_h = (u16)((peak_mag/max_mag) * (float)(h-30));
        u16 mark_y = (u16)(base_y - mark_h);
        LCD_Fill((u16)(mark_x-2), (u16)(mark_y-2), (u16)(mark_x+2), (u16)(mark_y+2), RED);
    }
}

/*============================================================
 * BER 曲线
 *============================================================*/
static void UI_DrawBERCurve(u16 x, u16 y, u16 w, u16 h)
{
    u16 ax=x+30, ay=y+4, aw=w-34, ah=h-22;
    u8 gi,cidx,i;
    u16 gx,gy;
    char xlab[4];

    float *curves[3];
    u16 colors[3];

    u8 prev_valid;
    u16 prev_px,prev_py;
    u16 cur_px,cur_py;

    float ber,tmp,log_ber;
    s8 exp_cnt;
    s16 py_off;

    u16 leg_x,leg_y;

    u16 mark_x=0;
    u16 mark_y=0;
    u16 yy;

    static const char * const clabels[3]=
    {
        "BPSK",
        "QPSK",
        "QAM "
    };

    static const char * const ylabels[4]=
    {
        "1e0 ",
        "1e-2",
        "1e-4",
        "1e-6"
    };

    static const char * const xlabels[6]=
    {
        " 0",
        " 4",
        " 8",
        "12",
        "16",
        "20"
    };

    curves[0]=g_state.ber_curve_bpsk;
    curves[1]=g_state.ber_curve_qpsk;
    curves[2]=g_state.ber_curve_qam;

    colors[0]=GREEN;
    colors[1]=CYAN;
    colors[2]=YELLOW;

    LCD_Fill(x,y,x+w-1,y+h-1,COLOR_PANEL_BG);

    LCD_DrawRectangle(x,y,x+w-1,y+h-1);

    LCD_DrawLine_Color(ax,ay,ax,ay+ah,WHITE);
    LCD_DrawLine_Color(ax,ay+ah,ax+aw,ay+ah,WHITE);

    FRONT_COLOR=COLOR_STATUS_FG;
    BACK_COLOR=COLOR_PANEL_BG;

    /* Y轴网格 */
    for(gi=0;gi<4;gi++)
    {
        gy=ay+ah*gi/3;

        LCD_DrawLine_Color(ax-2,
                           gy,
                           ax+aw,
                           gy,
                           COLOR_GRID);

        LCD_ShowString(x,
                       gy-4,
                       28,
                       12,
                       12,
                       (u8*)ylabels[gi]);
    }

    /* X轴网格 */
    for(gi=0;gi<6;gi++)
    {
        gx=ax+aw*gi/5;

        xlab[0]=xlabels[gi][0];
        xlab[1]=xlabels[gi][1];
        xlab[2]='\0';

        LCD_DrawLine_Color(gx,
                           ay,
                           gx,
                           ay+ah+2,
                           COLOR_GRID);

        LCD_ShowString(gx-4,
                       ay+ah+3,
                       20,
                       12,
                       12,
                       (u8*)xlab);
    }

    LCD_ShowString(ax+aw/2-20,
                   ay+ah+12,
                   50,
                   12,
                   12,
                   (u8*)"SNR(dB)");



    /*************** 三条曲线 ***************/
    for(cidx=0;cidx<3;cidx++)
    {
        prev_valid=0;

        for(i=1;i<12;i++)
        {
            ber=curves[cidx][i];

            if(ber<=0.0f)
                ber=1e-13f;

            tmp=ber;
            exp_cnt=0;

            while(tmp<1.0f && exp_cnt>-7)
            {
                tmp*=10.0f;
                exp_cnt--;
            }

            log_ber=(float)exp_cnt+(tmp-1.0f)/9.0f;

            py_off=(s16)((-log_ber)*(float)ah/6.0f);

            if(py_off<0)
                py_off=0;

            if(py_off>(s16)ah)
                py_off=(s16)ah;

            cur_px=ax+(u16)((u32)(i-1)*aw/10);

            cur_py=ay+(u16)py_off;

            /* 当前工作点记录 */
            if(cidx==g_state.mod_idx)
            {
                float snr_point=(float)(i-1)*2.0f;

                if(fabsf(snr_point-g_state.snr_db)<=1.0f)
                {
                    mark_x=cur_px;
                    mark_y=cur_py;
                }
            }

            if(prev_valid)
            {
                LCD_DrawLine_Color(prev_px,
                                   prev_py,
                                   cur_px,
                                   cur_py,
                                   colors[cidx]);

                /* 当前曲线加粗 */
                if(cidx==g_state.mod_idx)
                {
                    LCD_DrawLine_Color(prev_px,
                                       prev_py-1,
                                       cur_px,
                                       cur_py-1,
                                       colors[cidx]);

                    LCD_DrawLine_Color(prev_px,
                                       prev_py+1,
                                       cur_px,
                                       cur_py+1,
                                       colors[cidx]);
                }
            }

            prev_px=cur_px;
            prev_py=cur_py;

            prev_valid=1;
        }

        /* 图例 */
        leg_x=ax+aw-52;
        leg_y=ay+6+(u16)cidx*14;

        LCD_DrawLine_Color(leg_x,
                           leg_y+4,
                           leg_x+12,
                           leg_y+4,
                           colors[cidx]);

        FRONT_COLOR=colors[cidx];
        BACK_COLOR=COLOR_PANEL_BG;

        LCD_ShowString(leg_x+14,
                       leg_y,
                       36,
                       12,
                       12,
                       (u8*)clabels[cidx]);
    }


    /*************** 红色虚线 ***************/
    for(yy=ay+ah;yy>mark_y;yy-=6)
    {
        LCD_DrawLine_Color(mark_x,
                           yy,
                           mark_x,
                           yy-3,
                           RED);
    }


    /*************** 红点 ***************/
    LCD_Fill(mark_x-3,
             mark_y-3,
             mark_x+3,
             mark_y+3,
             RED);
}


/*============================================================
 * INFO 页子组件（独立抽取方便局部刷新）
 *============================================================*/

/* 绘制调制方式三按钮 */
static void Info_DrawModButtons(void)
{
    u8  mi;
    u16 bx, by, bw, bh, bg, fg;
    u16 cy = INFO_MOD_Y1;

    LCD_Fill(8, cy, SCR_W-9, cy+51, COLOR_PANEL_BG);
    LCD_DrawRectangle(8, cy, SCR_W-9, cy+51);
    FRONT_COLOR=COLOR_ACCENT; BACK_COLOR=COLOR_PANEL_BG;
    LCD_ShowString(14, cy+4, 120, 16, 16, (u8*)"Modulation:");

    bw=88; bh=22;
    for (mi=0;mi<3;mi++) {
        bx=14+(u16)mi*96; by=cy+24;
        bg=(mi==g_state.mod_idx)?COLOR_ACCENT:COLOR_GRID;
        fg=(mi==g_state.mod_idx)?BLACK:WHITE;
        LCD_Fill(bx,by,bx+bw-1,by+bh-1,bg);
        LCD_DrawRectangle(bx,by,bx+bw-1,by+bh-1);
        FRONT_COLOR=fg; BACK_COLOR=bg;
        LCD_ShowString(bx+8,by+4,bw-4,16,16,(u8*)MOD_NAMES[mi]);
    }
}

/* 绘制SNR滑动条 */
static void Info_DrawSNRSlider(void)
{
    u16 sx, sy, sw, fill_w, thumb;
    char snr_buf[8];
    u8   si;
    s16  snr;
    s16  snr_min=-2, snr_max=20;
    u16  cy = INFO_SNR_Y1;

    LCD_Fill(8, cy, SCR_W-9, cy+49, COLOR_PANEL_BG);
    LCD_DrawRectangle(8, cy, SCR_W-9, cy+49);
    FRONT_COLOR=COLOR_ACCENT; BACK_COLOR=COLOR_PANEL_BG;
    LCD_ShowString(14, cy+4, 90, 16, 16, (u8*)"SNR (dB):");

    /* 当前值 */
    snr=g_state.snr_db; si=0;
    if (snr<0) { snr_buf[si++]='-'; snr=-snr; }
    if (snr>=10) snr_buf[si++]=(char)('0'+snr/10);
    snr_buf[si++]=(char)('0'+snr%10); snr_buf[si]='\0';
    FRONT_COLOR=WHITE;
    LCD_ShowString(SCR_W-46, cy+4, 40, 16, 16, (u8*)snr_buf);

    /* 滑条 */
    sx=INFO_SNR_TRACK_X1; sy=cy+28;
    sw=INFO_SNR_TRACK_X2-INFO_SNR_TRACK_X1;
    fill_w=(u16)(((long)(g_state.snr_db-snr_min)*(long)sw)/(snr_max-snr_min));
    if (fill_w>sw) fill_w=sw;
    LCD_Fill(sx,sy,sx+sw-1,sy+10,COLOR_GRID);
    LCD_Fill(sx,sy,sx+fill_w,sy+10,COLOR_ACCENT);
    thumb=sx+fill_w;
    if (thumb>sx+sw-6) thumb=sx+sw-6;
    LCD_Fill(thumb,sy-4,thumb+8,sy+14,WHITE);

    FRONT_COLOR=COLOR_STATUS_FG; BACK_COLOR=COLOR_PANEL_BG;
    LCD_ShowString(sx,sy+12,20,12,12,(u8*)"-2");
    LCD_ShowString(sx+sw-14,sy+12,20,12,12,(u8*)"20");
}

/* 绘制实时测量区 */
static void Info_DrawMeasurements(u16 cy)
{
    char ber_str[12];
    float b;
    u8 exp_cnt, bi;

    LCD_Fill(8,cy,SCR_W-9,cy+89,COLOR_PANEL_BG);
    LCD_DrawRectangle(8,cy,SCR_W-9,cy+89);
    FRONT_COLOR=COLOR_ACCENT; BACK_COLOR=COLOR_PANEL_BG;
    LCD_ShowString(14,cy+4,120,16,16,(u8*)"Measurements");

    b=g_state.ber; exp_cnt=0;
    while (b<0.1f && exp_cnt<9) { b*=10.0f; exp_cnt++; }
    ber_str[0]='B'; ber_str[1]='E'; ber_str[2]='R'; ber_str[3]='<';
    ber_str[4]='1'; ber_str[5]='E'; ber_str[6]='-'; bi=7;
    ber_str[bi++]=(char)('0'+exp_cnt); ber_str[bi]='\0';
    FRONT_COLOR=(g_state.ber<1e-4f)?COLOR_GOOD:(g_state.ber<1e-2f)?COLOR_WARN:COLOR_ERR;
    LCD_ShowString(14,cy+24,140,16,16,(u8*)ber_str);

    FRONT_COLOR=COLOR_PANEL_FG; BACK_COLOR=COLOR_PANEL_BG;
    LCD_ShowString(14,cy+46,80,16,16,(u8*)"ErrBit:");
    LCD_ShowNum(100,cy+46,g_state.err_bits,6,16);
    LCD_ShowString(14,cy+66,80,16,16,(u8*)"TotBit:");
    LCD_ShowNum(100,cy+66,g_state.total_bits,6,16);
}

/* 绘制系统参数区 */
static void Info_DrawParams(u16 cy)
{
    LCD_Fill(8,cy,SCR_W-9,cy+79,COLOR_PANEL_BG);
    LCD_DrawRectangle(8,cy,SCR_W-9,cy+79);
    FRONT_COLOR=COLOR_ACCENT; BACK_COLOR=COLOR_PANEL_BG;
    LCD_ShowString(14,cy+4,120,16,16,(u8*)"Parameters");
    FRONT_COLOR=COLOR_PANEL_FG;
    LCD_ShowString(14,cy+24,SCR_W-20,16,16,(u8*)"SymRate: 125 kSym/s");
    LCD_ShowString(14,cy+44,SCR_W-20,16,16,(u8*)"Rolloff: 0.35  SPS:8");
    LCD_ShowString(14,cy+64,SCR_W-20,16,16,(u8*)"Fc:200kHz  Fs:1MHz");
}

/*============================================================
 * 页面绘制
 *============================================================*/
static void Page_DrawInfo(void)
{
    u16 cy;
    UI_DrawTitleBar("  System Info");
    Info_DrawModButtons();
    Info_DrawSNRSlider();
    cy = INFO_SNR_Y2 + 8;
    Info_DrawMeasurements(cy);
    cy += 97;
    Info_DrawParams(cy);
    UI_DrawNavBar("Mod-", "Mod+", "NextPage");
}

static void Page_DrawWaveform(void)
{
    u16 cy, wh;
    wh = (CONTENT_H - 16) / 3 - 4;
    UI_DrawTitleBar("  Waveform");
    cy = CONTENT_Y + 4;
    UI_DrawWaveform(4,cy,SCR_W-8,wh,g_state.wave_bb, 200,COLOR_WAVE_BB,"Baseband");
    cy += wh+4;
    UI_DrawWaveform(4,cy,SCR_W-8,wh,g_state.wave_mod,200,COLOR_WAVE_MOD,"Modulated");
    cy += wh+4;
    UI_DrawWaveform(4,cy,SCR_W-8,wh,g_state.wave_dem,200,COLOR_WAVE_DEM,"Demodulated");
    UI_DrawNavBar("PrevPage","","NextPage");
}

static void Page_DrawSpectrum(void)
{
    u16 cy, ph;
    ph = (CONTENT_H - 12) / 2 - 4;
    UI_DrawTitleBar("  Spectrum (FFT)");
    cy = CONTENT_Y + 4;
    UI_DrawSpectrum(4, cy, SCR_W-8, ph,
                    g_state.fft_mag_bb, COLOR_WAVE_BB, "Baseband Spectrum");
    cy += ph + 4;
    UI_DrawSpectrum(4, cy, SCR_W-8, ph,
                    g_state.fft_mag, COLOR_WAVE_MOD, "Modulated Spectrum");
    UI_DrawNavBar("PrevPage","","NextPage");
}

static void Page_DrawIQ(void)
{
    u16 r=100, cx=SCR_W/2, cy, iy;
    char snr_disp[14];
    s16 snr; u8 si;

    cy = CONTENT_Y+20+r+4;
    UI_DrawTitleBar("  IQ Constellation");
    FRONT_COLOR=COLOR_IQ_TX; BACK_COLOR=COLOR_BG;
    LCD_ShowString(8,CONTENT_Y+4,80,12,12,(u8*)"o TX ideal");
    FRONT_COLOR=COLOR_IQ_RX;
    LCD_ShowString(100,CONTENT_Y+4,100,12,12,(u8*)"* RX received");

    UI_DrawIQPlot(cx,cy,r,g_state.iq_tx_i,g_state.iq_tx_q,64,
                          g_state.iq_rx_i,g_state.iq_rx_q,128);
    FRONT_COLOR=COLOR_GRID; BACK_COLOR=COLOR_PANEL_BG;
    LCD_ShowString(cx+r-8,cy-8,12,12,12,(u8*)"I");
    LCD_ShowString(cx-6,cy-r-12,12,12,12,(u8*)"Q");

    iy = CONTENT_Y+20+r*2+16;
    FRONT_COLOR=WHITE; BACK_COLOR=COLOR_BG;
    LCD_ShowString(8,iy,SCR_W-16,16,16,(u8*)"Mode:");
    FRONT_COLOR=COLOR_ACCENT;
    LCD_ShowString(60,iy,70,16,16,(u8*)MOD_NAMES[g_state.mod_idx]);

    snr_disp[0]='S'; snr_disp[1]='N'; snr_disp[2]='R'; snr_disp[3]=':';
    si=4; snr=g_state.snr_db;
    if (snr<0) { snr_disp[si++]='-'; snr=-snr; }
    if (snr>=10) snr_disp[si++]=(char)('0'+snr/10);
    snr_disp[si++]=(char)('0'+snr%10);
    snr_disp[si++]='d'; snr_disp[si++]='B'; snr_disp[si]='\0';
    FRONT_COLOR=WHITE;
    LCD_ShowString(8,iy+20,SCR_W-16,16,16,(u8*)snr_disp);

    UI_DrawNavBar("PrevPage","","NextPage");
}

static void Page_DrawEye(void)
{
    UI_DrawTitleBar("  Eye Diagram");
    UI_DrawEyeDiagram(4,CONTENT_Y+4,SCR_W-8,CONTENT_H-8);
    UI_DrawNavBar("PrevPage","","NextPage");
}

static void Page_DrawBER(void)
{
    UI_DrawTitleBar("  BER Curve");
    UI_DrawBERCurve(4,CONTENT_Y+4,SCR_W-8,CONTENT_H-8);
    UI_DrawNavBar("PrevPage","","Page 1");
}

static void Page_Update(void)
{
    switch (g_page) {
        case PAGE_INFO:     Page_DrawInfo();     break;
        case PAGE_WAVEFORM: Page_DrawWaveform(); break;
        case PAGE_SPECTRUM: Page_DrawSpectrum(); break;
        case PAGE_IQ:       Page_DrawIQ();       break;
        case PAGE_EYE:      Page_DrawEye();      break;
        case PAGE_BER:      Page_DrawBER();      break;
        default:            Page_DrawInfo();     break;
    }
    UI_DrawStatusBar();
}

/*============================================================
 * 触摸处理
 *============================================================*/

/* INFO 页触摸：调制按钮 + SNR 滑条 */
static void Touch_HandleInfo(void)
{
    u8  mi;
    u16 bx, by, bw=88, bh=22;
    s16 snr_min=-2, snr_max=20;
    u16 sw = INFO_SNR_TRACK_X2 - INFO_SNR_TRACK_X1;
    u16 sy = INFO_SNR_Y1 + 28 - 8;   /* 滑条触摸区域顶部（含上下余量） */
    u16 sy2 = sy + 26;

    /* ------- 调制按钮（仅松手触发） ------- */
    if (g_touch.just_released && !g_touch.dragging)
    {
        by = INFO_MOD_Y1 + 24;
        for (mi = 0; mi < 3; mi++) {
            bx = 14 + (u16)mi * 96;
            /* 检查按下点（press_x/press_y）是否在按钮内 */
            if (g_touch.press_x >= bx && g_touch.press_x <= bx+bw-1 &&
                g_touch.press_y >= by && g_touch.press_y <= by+bh-1)
            {
                if (g_state.mod_idx != mi) {
                    g_state.mod_idx = mi;
                    /* 局部刷新：只重绘调制按钮区和状态栏 */
                    Info_DrawModButtons();
                    UI_DrawStatusBar();
                    printf("Touch: Mod=%s\r\n", MOD_NAMES[g_state.mod_idx]);
										Demod_Init(&g_demod, g_state.mod_idx);
										Demod_ResetBER(&g_demod);
                }
                break;
            }
        }
    }

    /* ------- SNR 滑动条 ------- */
    /* 按下时判断是否在滑条区域内，开始拖动 */
    if (g_touch.just_pressed)
    {
        if (g_touch.press_x >= INFO_SNR_TRACK_X1 - 10 &&
            g_touch.press_x <= INFO_SNR_TRACK_X2 + 10 &&
            g_touch.press_y >= sy && g_touch.press_y <= sy2)
        {
            g_snr_dragging = 1;
        }
    }

    /* 拖动中实时更新SNR */
    if (g_snr_dragging && (g_touch.dragging || g_touch.just_pressed))
    {
        s16 new_snr;
        u16 tx = g_touch.x;
        if (tx < INFO_SNR_TRACK_X1) tx = INFO_SNR_TRACK_X1;
        if (tx > INFO_SNR_TRACK_X2) tx = INFO_SNR_TRACK_X2;
        new_snr = snr_min + (s16)(((long)(tx - INFO_SNR_TRACK_X1) *
                                    (snr_max - snr_min)) / (long)sw);
        /* 对齐到偶数（步长2dB） */
        new_snr = (new_snr / 2) * 2;
        if (new_snr < snr_min) new_snr = snr_min;
        if (new_snr > snr_max) new_snr = snr_max;

        if (new_snr != g_state.snr_db) {
            g_state.snr_db = new_snr;
            /* 局部刷新：只重绘SNR区和状态栏 */
            Info_DrawSNRSlider();
            UI_DrawStatusBar();
            printf("Touch: SNR=%ddB\r\n", (int)g_state.snr_db);
        }
    }

    /* 松开时停止拖动 */
    if (g_touch.just_released)
        g_snr_dragging = 0;
}

/* 底部导航栏触摸处理（松手触发） */
static void Touch_HandleNav(void)
{
    u8 in_left, in_mid, in_right;

    if (!g_touch.just_released) return;
    if (g_touch.dragging)       return;   /* 拖动结束不触发导航 */

    /* 判断松手坐标所在格 */
    in_left  = Touch_InRect(NAV_LEFT_X1,  NAV_Y1, NAV_LEFT_X2,  NAV_Y2);
    in_mid   = Touch_InRect(NAV_MID_X1,   NAV_Y1, NAV_MID_X2,   NAV_Y2);
    in_right = Touch_InRect(NAV_RIGHT_X1, NAV_Y1, NAV_RIGHT_X2, NAV_Y2);

    /* 同时检查按下点也在同一格（避免滑入） */
    if (in_left &&
        g_touch.press_x >= NAV_LEFT_X1 && g_touch.press_x <= NAV_LEFT_X2 &&
        g_touch.press_y >= NAV_Y1      && g_touch.press_y <= NAV_Y2)
    {
        UI_DrawNavHighlight(0);
        if (g_page == PAGE_INFO) {
            g_state.mod_idx = (g_state.mod_idx > 0) ? g_state.mod_idx-1 : 2u;
            Info_DrawModButtons(); UI_DrawStatusBar();
        } else {
            g_page = (Page_t)((u8)(g_page + PAGE_COUNT - 1) % PAGE_COUNT);
            g_need_redraw = 1;
        }
        printf("Nav: Left\r\n");
    }
    else if (in_mid &&
             g_touch.press_x >= NAV_MID_X1 && g_touch.press_x <= NAV_MID_X2 &&
             g_touch.press_y >= NAV_Y1      && g_touch.press_y <= NAV_Y2)
    {
        UI_DrawNavHighlight(1);
        if (g_page == PAGE_INFO) {
            g_state.mod_idx = (g_state.mod_idx+1) % 3u;
            Info_DrawModButtons(); UI_DrawStatusBar();
        }
        printf("Nav: Mid\r\n");
    }
    else if (in_right &&
             g_touch.press_x >= NAV_RIGHT_X1 && g_touch.press_x <= NAV_RIGHT_X2 &&
             g_touch.press_y >= NAV_Y1        && g_touch.press_y <= NAV_Y2)
    {
        UI_DrawNavHighlight(2);
        g_page = (Page_t)((u8)(g_page+1) % PAGE_COUNT);
        g_need_redraw = 1;
        printf("Nav: Right\r\n");
    }
}

/* 总触摸分发 */
static void Touch_Process(void)
{
    /* 先处理导航栏（优先级高） */
    if (g_touch.y >= NAV_Y1)
    {
        Touch_HandleNav();
        return;
    }

    /* INFO页内容区触摸 */
    if (g_page == PAGE_INFO)
        Touch_HandleInfo();
}

/*============================================================
 * 物理按键处理（备用）
 *============================================================*/
static void Page_HandleKey(u8 key)
{
    switch (key) {
        case 1: /* KEY0: 下一页 */
            g_page = (Page_t)((u8)(g_page+1) % PAGE_COUNT);
            g_need_redraw = 1;
            break;
        case 2: /* KEY1: 上一页 / 调制- */
            if (g_page == PAGE_INFO) {
                g_state.mod_idx = (g_state.mod_idx>0) ? g_state.mod_idx-1 : 2u;
                Info_DrawModButtons(); UI_DrawStatusBar();
            } else {
                g_page = (Page_t)((u8)(g_page+PAGE_COUNT-1) % PAGE_COUNT);
                g_need_redraw = 1;
            }
            break;
        case 3: /* KEY2: 调制+ */
            if (g_page == PAGE_INFO) {
                g_state.mod_idx = (g_state.mod_idx+1) % 3u;
                Info_DrawModButtons(); UI_DrawStatusBar();
            }
            break;
        default: break;
    }
    printf("Key=%d Page=%d Mod=%s SNR=%d\r\n",
           (int)key,(int)g_page,MOD_NAMES[g_state.mod_idx],(int)g_state.snr_db);
}