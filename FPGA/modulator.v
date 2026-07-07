module modulator
(
    input               clk,
    input               rst_n,

    input       [1:0]   mode,
    input       [3:0]   bits_in,

    output reg signed [7:0] sample_out
);

parameter signed AMP1 = 8'sd42;
parameter signed AMP3 = 8'sd126;

/////////////////////////////////////////////////////////
// RRC 系数
/////////////////////////////////////////////////////////

parameter signed C0  = -16;
parameter signed C1  = -24;
parameter signed C2  = -12;
parameter signed C3  =  24;
parameter signed C4  =  96;
parameter signed C5  = 192;
parameter signed C6  = 288;
parameter signed C7  = 320;
parameter signed C8  = 288;
parameter signed C9  = 192;
parameter signed C10 = 96;
parameter signed C11 = 24;
parameter signed C12 = -12;
parameter signed C13 = -24;
parameter signed C14 = -16;

/////////////////////////////////////////////////////////
// Gray Mapping
/////////////////////////////////////////////////////////

reg signed [7:0] sym_i;
reg signed [7:0] sym_q;

always @(*)
begin

    case(mode)

    // BPSK
    2'd0:
    begin
        sym_i = bits_in[0] ? 8'sd127 : -8'sd127;
        sym_q = 0;
    end

    // QPSK
    2'd1:
    begin
        sym_i = bits_in[1] ? 8'sd90 : -8'sd90;
        sym_q = bits_in[0] ? 8'sd90 : -8'sd90;
    end

    // 16QAM Gray
    default:
    begin

        case(bits_in[3:2])
        2'b00: sym_i=-AMP3;
        2'b01: sym_i=-AMP1;
        2'b11: sym_i= AMP1;
        default:sym_i= AMP3;
        endcase

        case(bits_in[1:0])
        2'b00: sym_q=-AMP3;
        2'b01: sym_q=-AMP1;
        2'b11: sym_q= AMP1;
        default:sym_q= AMP3;
        endcase

    end

    endcase

end

/////////////////////////////////////////////////////////
// SPS=8
/////////////////////////////////////////////////////////

reg [2:0] sps_cnt;

reg signed [7:0] hold_i;
reg signed [7:0] hold_q;

always @(posedge clk or negedge rst_n)
begin
    if(!rst_n)
    begin
        sps_cnt<=0;
        hold_i<=0;
        hold_q<=0;
    end
    else
    begin

        sps_cnt<=sps_cnt+1'b1;

        if(sps_cnt==3'd0)
        begin
            hold_i<=sym_i;
            hold_q<=sym_q;
        end

    end
end

/////////////////////////////////////////////////////////
// FIR shift register
/////////////////////////////////////////////////////////

reg signed [7:0] shift_i[0:14];
reg signed [7:0] shift_q[0:14];

integer k;

always @(posedge clk or negedge rst_n)
begin

    if(!rst_n)
    begin

        for(k=0;k<15;k=k+1)
        begin
            shift_i[k]<=0;
            shift_q[k]<=0;
        end

    end
    else
    begin

        for(k=14;k>0;k=k-1)
        begin
            shift_i[k]<=shift_i[k-1];
            shift_q[k]<=shift_q[k-1];
        end

        shift_i[0]<=hold_i;
        shift_q[0]<=hold_q;

    end

end

/////////////////////////////////////////////////////////
// RRC FIR
/////////////////////////////////////////////////////////

wire signed [23:0] rrc_i;
wire signed [23:0] rrc_q;

assign rrc_i =

shift_i[0]*C0 +
shift_i[1]*C1 +
shift_i[2]*C2 +
shift_i[3]*C3 +
shift_i[4]*C4 +
shift_i[5]*C5 +
shift_i[6]*C6 +
shift_i[7]*C7 +
shift_i[8]*C8 +
shift_i[9]*C9 +
shift_i[10]*C10 +
shift_i[11]*C11 +
shift_i[12]*C12 +
shift_i[13]*C13 +
shift_i[14]*C14;

assign rrc_q =

shift_q[0]*C0 +
shift_q[1]*C1 +
shift_q[2]*C2 +
shift_q[3]*C3 +
shift_q[4]*C4 +
shift_q[5]*C5 +
shift_q[6]*C6 +
shift_q[7]*C7 +
shift_q[8]*C8 +
shift_q[9]*C9 +
shift_q[10]*C10 +
shift_q[11]*C11 +
shift_q[12]*C12 +
shift_q[13]*C13 +
shift_q[14]*C14;

/////////////////////////////////////////////////////////
// Carrier LUT
/////////////////////////////////////////////////////////

reg [4:0] carrier_idx;

wire signed [7:0] cos_lut;
wire signed [7:0] sin_lut;

carrier_lut carrier_lut_u
(
    .addr(carrier_idx),
    .cos_out(cos_lut),
    .sin_out(sin_lut)
);

always @(posedge clk or negedge rst_n)
begin

    if(!rst_n)
        carrier_idx<=0;
    else
        carrier_idx<=carrier_idx+5'd4;

end

/////////////////////////////////////////////////////////
// IQ 调制
/////////////////////////////////////////////////////////

wire signed [15:0] base_i;
wire signed [15:0] base_q;

assign base_i = rrc_i >>> 8;
assign base_q = rrc_q >>> 8;

wire signed [23:0] mul_i;
wire signed [23:0] mul_q;

assign mul_i = base_i*cos_lut;
assign mul_q = base_q*sin_lut;

wire signed [23:0] rf_wave;

assign rf_wave = mul_i - mul_q;

/////////////////////////////////////////////////////////
// 输出
/////////////////////////////////////////////////////////

wire signed [15:0] rf_scale;

assign rf_scale = rf_wave >>> 7;

always @(posedge clk or negedge rst_n)
begin

    if(!rst_n)
        sample_out<=0;

    else
    begin

        if(rf_scale>127)
            sample_out<=127;

        else if(rf_scale<-127)
            sample_out<=-127;

        else
            sample_out<=rf_scale[7:0];

    end

end

endmodule