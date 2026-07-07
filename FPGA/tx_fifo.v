/*******************************************************************************
 * tx_fifo.v
 *
 * 64×8-bit 循环 FIFO
 * 写端: 调制器每个样本写入 (full 时覆盖最旧数据)
 * 读端: SPI 从机每次回传一个字节时读出
 *******************************************************************************/

module tx_fifo (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       wr_en,
    input  wire [7:0] wr_data,
    input  wire       rd_en,
    output reg  [7:0] rd_data,
    output wire       empty,
    output wire       full
);

    reg [7:0] mem [0:63];
    reg [5:0] wr_ptr, rd_ptr;
    reg [6:0] count;   // 0~64

    assign empty = (count == 7'd0);
    assign full  = (count == 7'd64);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr  <= 6'd0;
            rd_ptr  <= 6'd0;
            count   <= 7'd0;
            rd_data <= 8'h80;
        end else begin
            // 写 (full 时覆盖, 保证调制器连续输出不阻塞)
            if (wr_en) begin
                mem[wr_ptr] <= wr_data;
                wr_ptr      <= wr_ptr + 6'd1;
                if (!full)
                    count <= count + 7'd1;
                else
                    rd_ptr <= rd_ptr + 6'd1;  // 覆盖时移动读指针
            end

            // 读
            if (rd_en && !empty) begin
                rd_data <= mem[rd_ptr];
                rd_ptr  <= rd_ptr + 6'd1;
                count   <= count - 7'd1;
            end
        end
    end

endmodule
