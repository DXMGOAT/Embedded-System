/*******************************************************************************
 * top.v  —  数字基带系统 FPGA 顶层
 * 器件: EP4CE10F17C8   系统时钟: 50 MHz (CLK_IN)
 *
 * 引脚对应:
 *   A11 = SPI_MISO  (FPGA → STM32 PB14)
 *   A12 = SPI_MOSI  (STM32 PB15 → FPGA)
 *   另需: SPI_SCK, SPI_NSS (由 STM32 SPI2 提供, 分别接 FPGA 任意 IO)
 *
 * SPI 帧格式 (STM32 → FPGA, 共 66 字节):
 *   [0]  0xA5  起始标志
 *   [1]  mod_idx  (0=BPSK 1=QPSK 2=16QAM)
 *   [2]  snr_byte (snr_db + 32, 使正数化)
 *   [3..65] 0x00 占位 (FPGA 在此窗口同时回传 64 字节调制后采样数据)
 *
 * FPGA → STM32 回传帧 (同步于 SPI 从机 MISO, 共 66 字节):
 *   [0]  0x5A  帧头
 *   [1]  0x00  保留
 *   [2..65] 64 字节调制信号采样 (uint8, 中心=128)
 *******************************************************************************/

module top (
    input  wire        CLK_IN,      // 50 MHz 晶振
    input  wire        SPI_SCK,     // SPI 时钟  (STM32 主机提供)
    input  wire        SPI_NSS,     // SPI 片选  (低有效)
    input  wire        SPI_MOSI,    // 数据输入  A12
    output wire        SPI_MISO,    // 数据输出  A11
    output wire [7:0]  LED          // 调试 LED (可选)
);
	wire rst_n=1'b1;
	reg [6:0] pn_state;

	wire feedback;

	assign feedback =
	pn_state[6]^pn_state[5];
	
	always @(posedge CLK_IN)
	begin

		if(!rst_n)
			pn_state<=7'b0000001;

		else
			pn_state<={pn_state[5:0],feedback};

	end
	
	wire [3:0] bits_in;
	assign bits_in =
	{pn_state[6],
	 pn_state[5],
	 pn_state[4],
	 pn_state[3]};
	 
    // -------------------------------------------------------
    // 内部信号
    // -------------------------------------------------------
    wire [1:0] mod_idx;
    wire [7:0] snr_byte;
    wire       frame_valid;

    // 调制后波形送往回传 FIFO
    wire [7:0]  mod_sample;
    wire        mod_valid;

    // 回传 FIFO 读端口
    wire [7:0]  tx_byte;
    wire        tx_rd;
    wire        fifo_empty;

    // -------------------------------------------------------
    // SPI 从机控制器
    // -------------------------------------------------------
    spi_slave u_spi (
        .clk         (CLK_IN),
        .spi_sck     (SPI_SCK),
        .spi_nss     (SPI_NSS),
        .spi_mosi    (SPI_MOSI),
        .spi_miso    (SPI_MISO),
        // 解析出的参数
        .mod_idx     (mod_idx),
        .snr_byte    (snr_byte),
        .frame_valid (frame_valid),
        // 回传数据接口
        .tx_byte     (tx_byte),
        .tx_rd       (tx_rd),
        .fifo_empty  (fifo_empty)
    );

    // -------------------------------------------------------
    // 调制 + 成形滤波 + 信道加噪
    // -------------------------------------------------------

	modulator u_mod
	(
		.clk(CLK_IN),
		.rst_n(rst_n),

		.mode(mod_idx),

		.bits_in(bits_in),

		.sample_out(mod_sample)
	);
	
	assign mod_valid=1'b1;
	
    // -------------------------------------------------------
    // 回传 FIFO (64 深度, 8 位宽)
    // -------------------------------------------------------
    tx_fifo u_fifo (
        .clk      (CLK_IN),
        .rst_n    (1'b1),
        .wr_en    (mod_valid),
        .wr_data  (mod_sample),
        .rd_en    (tx_rd),
        .rd_data  (tx_byte),
        .empty    (fifo_empty)
    );

    // -------------------------------------------------------
    // 调试 LED
    // -------------------------------------------------------
    assign LED[1:0] = mod_idx;
    assign LED[3:2] = snr_byte[5:4];
    assign LED[4]   = frame_valid;
    assign LED[7:5] = 3'b0;

endmodule
