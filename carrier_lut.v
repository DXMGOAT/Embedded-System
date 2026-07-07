module carrier_lut
(
    input      [4:0] addr,

    output reg signed [7:0] cos_out,
    output reg signed [7:0] sin_out
);

always @(*)
begin
    case(addr)

    5'd0 :
    begin cos_out=8'sd127; sin_out=8'sd0; end

    5'd1 :
    begin cos_out=8'sd117; sin_out=8'sd49; end

    5'd2 :
    begin cos_out=8'sd90; sin_out=8'sd90; end

    5'd3 :
    begin cos_out=8'sd49; sin_out=8'sd117; end

    5'd4 :
    begin cos_out=8'sd0; sin_out=8'sd127; end

    5'd5 :
    begin cos_out=-8'sd49; sin_out=8'sd117; end

    5'd6 :
    begin cos_out=-8'sd90; sin_out=8'sd90; end

    5'd7 :
    begin cos_out=-8'sd117; sin_out=8'sd49; end

    5'd8 :
    begin cos_out=-8'sd127; sin_out=8'sd0; end

    5'd9 :
    begin cos_out=-8'sd117; sin_out=-8'sd49; end

    5'd10:
    begin cos_out=-8'sd90; sin_out=-8'sd90; end

    5'd11:
    begin cos_out=-8'sd49; sin_out=-8'sd117; end

    5'd12:
    begin cos_out=8'sd0; sin_out=-8'sd127; end

    5'd13:
    begin cos_out=8'sd49; sin_out=-8'sd117; end

    5'd14:
    begin cos_out=8'sd90; sin_out=-8'sd90; end

    5'd15:
    begin cos_out=8'sd117; sin_out=-8'sd49; end

    // 16~31重复一个周期

    5'd16:
    begin cos_out=8'sd127; sin_out=8'sd0; end

    5'd17:
    begin cos_out=8'sd117; sin_out=8'sd49; end

    5'd18:
    begin cos_out=8'sd90; sin_out=8'sd90; end

    5'd19:
    begin cos_out=8'sd49; sin_out=8'sd117; end

    5'd20:
    begin cos_out=8'sd0; sin_out=8'sd127; end

    5'd21:
    begin cos_out=-8'sd49; sin_out=8'sd117; end

    5'd22:
    begin cos_out=-8'sd90; sin_out=8'sd90; end

    5'd23:
    begin cos_out=-8'sd117; sin_out=8'sd49; end

    5'd24:
    begin cos_out=-8'sd127; sin_out=8'sd0; end

    5'd25:
    begin cos_out=-8'sd117; sin_out=-8'sd49; end

    5'd26:
    begin cos_out=-8'sd90; sin_out=-8'sd90; end

    5'd27:
    begin cos_out=-8'sd49; sin_out=-8'sd117; end

    5'd28:
    begin cos_out=8'sd0; sin_out=-8'sd127; end

    5'd29:
    begin cos_out=8'sd49; sin_out=-8'sd117; end

    5'd30:
    begin cos_out=8'sd90; sin_out=-8'sd90; end

    default:
    begin cos_out=8'sd117; sin_out=-8'sd49; end

    endcase
end

endmodule