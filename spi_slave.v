/*******************************************************************************
 * spi_slave.v  (修订版 v3 — 彻底修复 multiple constant drivers)
 *
 * 根本原因:
 *   Verilog 规定: 一个 reg 只能在 【一个】 always 块里被赋值。
 *   原代码中 frame_valid 在两个 always 块里都有 <= 赋值, 故报错。
 *
 * 修复方案:
 *   将字节解析逻辑 (原第二个 always) 完全合并进唯一的 always 块。
 *   用局部 wire rx_byte 避免引入额外寄存器, 保持时序清晰。
 *******************************************************************************/

module spi_slave (
    input  wire       clk,
    input  wire       spi_sck,
    input  wire       spi_nss,
    input  wire       spi_mosi,
    output reg        spi_miso,
    output reg  [1:0] mod_idx,
    output reg  [7:0] snr_byte,
    output reg        frame_valid,
    input  wire [7:0] tx_byte,
    output reg        tx_rd,
    input  wire       fifo_empty
);

    // ------------------------------------------------------------------
    // 1. 三级移位同步 (消除亚稳态)
    // ------------------------------------------------------------------
    reg [2:0] sck_r;
    reg [1:0] nss_r;
    reg [1:0] mosi_r;

    always @(posedge clk) begin
        sck_r  <= {sck_r[1:0],  spi_sck};
        nss_r  <= {nss_r[0],    spi_nss};
        mosi_r <= {mosi_r[0],   spi_mosi};
    end

    wire sck_rise = (sck_r[2:1] == 2'b01);   // SCK 上升沿
    wire sck_fall = (sck_r[2:1] == 2'b10);   // SCK 下降沿
    wire nss_lo   = ~nss_r[1];               // 片选低有效
    wire mosi_bit =  mosi_r[1];              // 稳定的 MOSI

    // ------------------------------------------------------------------
    // 2. 工作寄存器
    // ------------------------------------------------------------------
    reg [2:0] bit_cnt;    // 位计数: 7(MSB) → 0(LSB)
    reg [6:0] byte_cnt;   // 字节序号: 0~65
    reg [7:0] rx_shift;   // 接收移位寄存器
    reg [7:0] tx_shift;   // 发送移位寄存器
    reg       tx_loaded;  // tx_shift 已装载标志

    // 当前字节完整值 (bit_cnt==0 时有效)
    wire [7:0] rx_byte = {rx_shift[6:0], mosi_bit};

    // ------------------------------------------------------------------
    // 3. 唯一 always 块 — 所有输出寄存器均在此驱动
    // ------------------------------------------------------------------
    always @(posedge clk) begin

        // 脉冲信号每拍默认清 0
        frame_valid <= 1'b0;
        tx_rd       <= 1'b0;

        if (!nss_lo) begin
            // ── NSS 拉高: 复位 ──────────────────────────────────────
            bit_cnt   <= 3'd7;
            byte_cnt  <= 7'd0;
            tx_loaded <= 1'b0;
            rx_shift  <= 8'h00;
            tx_shift  <= 8'h00;
            spi_miso  <= 1'b0;

        end else begin

            // ── SCK 下降沿: 装载并移出 MISO ─────────────────────────
            if (sck_fall) begin
                if (!tx_loaded) begin
                    // 根据当前字节序号决定回传内容
                    case (byte_cnt)
                        7'd0:    tx_shift <= 8'h5A;          // 回传帧头
                        7'd1:    tx_shift <= 8'h00;          // 保留
                        default: begin
                            if (!fifo_empty) begin
                                tx_shift <= tx_byte;
                                tx_rd    <= 1'b1;            // 弹出 FIFO
                            end else begin
                                tx_shift <= 8'h80;           // FIFO 空补中值
                            end
                        end
                    endcase
                    tx_loaded <= 1'b1;
                end
                // 移出当前 MSB
                spi_miso <= tx_shift[7];
                tx_shift <= {tx_shift[6:0], 1'b0};
            end

            // ── SCK 上升沿: 移入 MOSI, 字节完成时解析 ───────────────
            if (sck_rise) begin
                rx_shift <= {rx_shift[6:0], mosi_bit};

                if (bit_cnt == 3'd0) begin
                    // 一个字节收完, 解析 (rx_byte 此时有效)
                    // byte_cnt 是本字节的序号 (尚未递增)
                    case (byte_cnt)
                        7'd0: begin
                            // 字节[0]: 帧头 0xA5 (可在此做校验, 暂不处理)
                        end
                        7'd1: begin
                            // 字节[1]: mod_idx (0/1/2)
                            if (rx_byte <= 8'd2)
                                mod_idx <= rx_byte[1:0];
                        end
                        7'd2: begin
                            // 字节[2]: snr_byte = snr_db + 32
                            snr_byte <= rx_byte;
                        end
                        7'd65: begin
                            // 字节[65]: 帧尾, 产生 frame_valid 脉冲
                            frame_valid <= 1'b1;
                        end
                        default: ; // 字节[3..64]: 占位, 不处理
                    endcase

                    // 字节计数与状态复位
                    byte_cnt  <= (byte_cnt == 7'd65) ? 7'd0 : byte_cnt + 7'd1;
                    bit_cnt   <= 3'd7;
                    tx_loaded <= 1'b0;   // 下一字节重新装载

                end else begin
                    bit_cnt <= bit_cnt - 3'd1;
                end
            end

        end // nss_lo
    end // always

endmodule