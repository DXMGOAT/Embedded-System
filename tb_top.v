/*******************************************************************************
 * tb_top.v  —  顶层功能仿真
 * ModelSim / Quartus 内置仿真均可使用
 *
 * 仿真内容:
 *   1. 发送一帧 66 字节 SPI 数据 (mod=QPSK, SNR=10dB, snr_byte=42)
 *   2. 捕获 FPGA 回传的 64 字节调制样本
 *   3. 打印到控制台
 *******************************************************************************/
`timescale 1ns/1ps

module tb_top;

    // -------------------------------------------------------
    // DUT 信号
    // -------------------------------------------------------
    reg  clk;
    reg  sck, nss, mosi;
    wire miso;
    wire [7:0] led;

    top dut (
        .CLK_IN  (clk),
        .SPI_SCK (sck),
        .SPI_NSS (nss),
        .SPI_MOSI(mosi),
        .SPI_MISO(miso),
        .LED     (led)
    );

    // -------------------------------------------------------
    // 50 MHz 系统时钟
    // -------------------------------------------------------
    initial clk = 0;
    always #10 clk = ~clk;   // 20 ns → 50 MHz

    // -------------------------------------------------------
    // SPI 时钟参数 (约 2.625 MHz → 周期 381 ns)
    // -------------------------------------------------------
    localparam SPI_HALF = 190;   // ns

    // SPI 发送一个字节并接收一个字节
    task spi_txrx_byte;
        input  [7:0] tx;
        output [7:0] rx;
        integer i;
        begin
            rx = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                mosi = tx[i];
                #(SPI_HALF);
                sck = 1;
                rx[i] = miso;
                #(SPI_HALF);
                sck = 0;
            end
        end
    endtask

    // -------------------------------------------------------
    // 主测试序列
    // -------------------------------------------------------
    reg [7:0] rx_frame [0:65];
    reg [7:0] dummy;
    integer j;

    initial begin
        $display("=== FPGA 基带系统仿真开始 ===");
        sck  = 0;
        nss  = 1;
        mosi = 0;
        #1000;  // 等待 DUT 稳定

        // --- 发起一帧 SPI 事务 ---
        nss = 0;
        #200;

        // Byte 0: 帧头 0xA5
        spi_txrx_byte(8'hA5, rx_frame[0]);
        // Byte 1: mod_idx = 1 (QPSK)
        spi_txrx_byte(8'h01, rx_frame[1]);
        // Byte 2: snr_byte = 42 (SNR=10dB)
        spi_txrx_byte(8'h2A, rx_frame[2]);
        // Byte 3..65: 填 0x00, 接收调制数据
        for (j = 3; j < 66; j = j + 1)
            spi_txrx_byte(8'h00, rx_frame[j]);

        #200;
        nss = 1;

        // --- 打印结果 ---
        $display("回传帧头: 0x%02X (期望 0x5A)", rx_frame[0]);
        $display("保留字节: 0x%02X", rx_frame[1]);
        $display("调制样本 (前32字节, uint8中值=128):");
        for (j = 2; j < 34; j = j + 1)
            $write("%4d", rx_frame[j]);
        $display("");

        // 等待更多输出
        #50000;
        $display("=== 仿真结束 ===");
        $finish;
    end

    // -------------------------------------------------------
    // 波形文件 (可选, 取消注释后生成 .vcd)
    // -------------------------------------------------------
    // initial begin
    //     $dumpfile("tb_top.vcd");
    //     $dumpvars(0, tb_top);
    // end

endmodule
