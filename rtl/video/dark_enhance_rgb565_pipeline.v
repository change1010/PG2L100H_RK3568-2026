`timescale 1ns/1ps

// 生产暗光像素流水线：固定延迟 13 拍，每拍接收一个 RGB565 像素。
// 用逐位除法保持整数运算结果，避免可变组合除法。
// 时序和帧标识由外层画布同步延迟。
// 物理时序结论以对应版本的 Pango 报告为准。
module dark_enhance_rgb565_pipeline (
    input clk,
    input rst_n,
    input enable,
    input [7:0] threshold,
    input [7:0] strength,
    input timing_hs,
    input timing_vs,
    input timing_de,
    input [15:0] pixel_in,
    output timing_hs_out,
    output timing_vs_out,
    output timing_de_out,
    output reg [15:0] pixel_out
);
    localparam integer STAGES = 13;
    wire [7:0] red_in = {pixel_in[15:11], pixel_in[15:13]};
    wire [7:0] green_in = {pixel_in[10:5], pixel_in[10:9]};
    wire [7:0] blue_in = {pixel_in[4:0], pixel_in[4:2]};
    wire [15:0] luma_sum = 16'd77 * red_in + 16'd150 * green_in +
                          16'd29 * blue_in + 16'd128;
    reg [7:0] red_0, green_0, blue_0, luma_0, threshold_0, strength_0;
    reg [7:0] red_1, green_1, blue_1, luma_1, gap_1, strength_1;
    reg [7:0] red_2, green_2, blue_2, luma_2;
    reg [15:0] lift_2;
    reg [7:0] red_3, green_3, blue_3, luma_3, target_3;
    reg [7:0] luma_4, target_4;
    reg [15:0] numerator_red_4, numerator_green_4, numerator_blue_4;
    reg [STAGES-1:0] hs_pipe, vs_pipe, de_pipe, bypass_pipe;
    reg [15:0] original_pipe [0:STAGES-2];

    // RGB565 不保留商的最低两位；逐位除法从高位开始计算，
    // 省略最低两位的迭代不影响最终像素，
    // 并减少两级算术流水。
    localparam integer DIVIDE_STAGES = 6;
    reg [7:0] denominator [0:DIVIDE_STAGES];
    reg [15:0] remainder_red [0:DIVIDE_STAGES], remainder_green [0:DIVIDE_STAGES], remainder_blue [0:DIVIDE_STAGES];
    reg [7:0] quotient_red [0:DIVIDE_STAGES], quotient_green [0:DIVIDE_STAGES], quotient_blue [0:DIVIDE_STAGES];
    wire [15:0] saturation_limit_4 = {luma_4, 8'd0};
    integer index;

    assign timing_hs_out = hs_pipe[STAGES-1];
    assign timing_vs_out = vs_pipe[STAGES-1];
    assign timing_de_out = de_pipe[STAGES-1];

    // 第 0..4 级依次计算亮度、差值、提升量、目标亮度和分子；
    // 第 5 级饱和，第 6..11 级求商，第 12 级选择增强、消隐或旁路输出。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            red_0 <= 0; green_0 <= 0; blue_0 <= 0; luma_0 <= 0;
            threshold_0 <= 0; strength_0 <= 0;
            red_1 <= 0; green_1 <= 0; blue_1 <= 0; luma_1 <= 0; gap_1 <= 0; strength_1 <= 0;
            red_2 <= 0; green_2 <= 0; blue_2 <= 0; luma_2 <= 0; lift_2 <= 0;
            red_3 <= 0; green_3 <= 0; blue_3 <= 0; luma_3 <= 0; target_3 <= 0;
            luma_4 <= 0; target_4 <= 0;
            numerator_red_4 <= 0; numerator_green_4 <= 0; numerator_blue_4 <= 0;
            hs_pipe <= 0; vs_pipe <= 0; de_pipe <= 0; bypass_pipe <= 0;
            for (index = 0; index < STAGES-1; index = index + 1)
                original_pipe[index] <= 0;
            denominator[0] <= 1;
            remainder_red[0] <= 0; remainder_green[0] <= 0; remainder_blue[0] <= 0;
            quotient_red[0] <= 0; quotient_green[0] <= 0; quotient_blue[0] <= 0;
            pixel_out <= 0;
        end else begin
            hs_pipe <= {hs_pipe[STAGES-2:0], timing_hs};
            vs_pipe <= {vs_pipe[STAGES-2:0], timing_vs};
            de_pipe <= {de_pipe[STAGES-2:0], timing_de};
            bypass_pipe <= {bypass_pipe[STAGES-2:0],
                (!enable || strength == 0 || luma_sum[15:8] >= threshold)};
            original_pipe[0] <= pixel_in;
            for (index = 1; index < STAGES-1; index = index + 1)
                original_pipe[index] <= original_pipe[index-1];

            red_0 <= red_in; green_0 <= green_in; blue_0 <= blue_in;
            luma_0 <= luma_sum[15:8]; threshold_0 <= threshold; strength_0 <= strength;

            red_1 <= red_0; green_1 <= green_0; blue_1 <= blue_0; luma_1 <= luma_0;
            gap_1 <= luma_0 < threshold_0 ? threshold_0-luma_0 : 8'd0;
            strength_1 <= strength_0;

            red_2 <= red_1; green_2 <= green_1; blue_2 <= blue_1; luma_2 <= luma_1;
            lift_2 <= (16'd1 * gap_1 * strength_1) + 16'd128;

            red_3 <= red_2; green_3 <= green_2; blue_3 <= blue_2; luma_3 <= luma_2;
            target_3 <= luma_2 + lift_2[15:8];

            luma_4 <= luma_3; target_4 <= target_3;
            numerator_red_4 <= (16'd1 * red_3 * target_3) + (luma_3 >> 1);
            numerator_green_4 <= (16'd1 * green_3 * target_3) + (luma_3 >> 1);
            numerator_blue_4 <= (16'd1 * blue_3 * target_3) + (luma_3 >> 1);

            // 饱和处理把商限制为 8 位。
            // 纯黑像素直接使用目标灰度，余数置零。
            denominator[0] <= luma_4 == 0 ? 8'd1 : luma_4;
            quotient_red[0] <= luma_4 == 0 ? target_4 :
                (numerator_red_4 >= saturation_limit_4 ? 8'd255 : 8'd0);
            quotient_green[0] <= luma_4 == 0 ? target_4 :
                (numerator_green_4 >= saturation_limit_4 ? 8'd255 : 8'd0);
            quotient_blue[0] <= luma_4 == 0 ? target_4 :
                (numerator_blue_4 >= saturation_limit_4 ? 8'd255 : 8'd0);
            remainder_red[0] <= luma_4 == 0 || numerator_red_4 >= saturation_limit_4 ?
                16'd0 : numerator_red_4;
            remainder_green[0] <= luma_4 == 0 || numerator_green_4 >= saturation_limit_4 ?
                16'd0 : numerator_green_4;
            remainder_blue[0] <= luma_4 == 0 || numerator_blue_4 >= saturation_limit_4 ?
                16'd0 : numerator_blue_4;

            if (!de_pipe[STAGES-2]) pixel_out <= 0;
            else if (bypass_pipe[STAGES-2]) pixel_out <= original_pipe[STAGES-2];
            else pixel_out <= {quotient_red[DIVIDE_STAGES][7:3], quotient_green[DIVIDE_STAGES][7:2], quotient_blue[DIVIDE_STAGES][7:3]};
        end
    end

    genvar stage;
    generate for (stage = 0; stage < DIVIDE_STAGES; stage = stage + 1) begin: divide_bit
        wire [15:0] trial_divisor = {8'd0, denominator[stage]} << (7-stage);
        localparam [7:0] QUOTIENT_BIT = 8'd1 << (7-stage);
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                denominator[stage+1] <= 1;
                remainder_red[stage+1] <= 0; remainder_green[stage+1] <= 0;
                remainder_blue[stage+1] <= 0;
                quotient_red[stage+1] <= 0; quotient_green[stage+1] <= 0;
                quotient_blue[stage+1] <= 0;
            end else begin
                denominator[stage+1] <= denominator[stage];
                remainder_red[stage+1] <= remainder_red[stage] >= trial_divisor ?
                    remainder_red[stage] - trial_divisor : remainder_red[stage];
                remainder_green[stage+1] <= remainder_green[stage] >= trial_divisor ?
                    remainder_green[stage] - trial_divisor : remainder_green[stage];
                remainder_blue[stage+1] <= remainder_blue[stage] >= trial_divisor ?
                    remainder_blue[stage] - trial_divisor : remainder_blue[stage];
                quotient_red[stage+1] <= quotient_red[stage] |
                    (remainder_red[stage] >= trial_divisor ? QUOTIENT_BIT : 8'd0);
                quotient_green[stage+1] <= quotient_green[stage] |
                    (remainder_green[stage] >= trial_divisor ? QUOTIENT_BIT : 8'd0);
                quotient_blue[stage+1] <= quotient_blue[stage] |
                    (remainder_blue[stage] >= trial_divisor ? QUOTIENT_BIT : 8'd0);
            end
        end
    end endgenerate
endmodule
