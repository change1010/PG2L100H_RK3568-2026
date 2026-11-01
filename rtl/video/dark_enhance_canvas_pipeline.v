`timescale 1ns/1ps

// 公共画布的像素、时序及帧标识统一延迟 13 拍。
// sample_* 接控制器本拍生效的参数，保证帧首像素使用新配置。
module dark_enhance_canvas_pipeline (
    input rst_n,
    input pix_clk,
    input timing_hs,
    input timing_vs,
    input timing_de,
    input [15:0] pixel_in,
    input [2:0] source_in,
    input sample_enable,
    input [7:0] sample_threshold,
    input [7:0] sample_strength,
    input [31:0] sample_generation,
    input [31:0] sample_request,
    output control_boundary,
    output timing_hs_out,
    output timing_vs_out,
    output timing_de_out,
    output [15:0] pixel_out,
    output [2:0] source_out,
    output [31:0] image_generation_out,
    output [31:0] image_request_out
);
    localparam integer STAGES = 13;
    (* ASYNC_REG = "TRUE" *) reg [2:0] pix_reset_pipe;
    wire pix_rst_n = pix_reset_pipe[2];
    reg vs_d;
    reg [2:0] source_pipe [0:STAGES-1];
    reg [31:0] generation_pipe [0:STAGES-1];
    reg [31:0] request_pipe [0:STAGES-1];
    integer index;

    always @(posedge pix_clk or negedge rst_n)
        if (!rst_n) pix_reset_pipe <= 0;
        else pix_reset_pipe <= {pix_reset_pipe[1:0], 1'b1};

    // 仅接受消隐期内的场同步下降沿；异常边界不提交待执行命令。
    assign control_boundary = pix_rst_n && vs_d && !timing_vs && !timing_de;
    assign source_out = source_pipe[STAGES-1];
    assign image_generation_out = generation_pipe[STAGES-1];
    assign image_request_out = request_pipe[STAGES-1];

    dark_enhance_rgb565_pipeline pixels (
        .clk(pix_clk), .rst_n(pix_rst_n), .enable(sample_enable),
        .threshold(sample_threshold), .strength(sample_strength),
        .timing_hs(timing_hs), .timing_vs(timing_vs), .timing_de(timing_de),
        .pixel_in(pixel_in), .timing_hs_out(timing_hs_out),
        .timing_vs_out(timing_vs_out), .timing_de_out(timing_de_out),
        .pixel_out(pixel_out)
    );

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            vs_d <= 0;
            for (index = 0; index < STAGES; index = index + 1) begin
                source_pipe[index] <= 0;
                generation_pipe[index] <= 0;
                request_pipe[index] <= 0;
            end
        end else begin
            vs_d <= timing_vs;
            source_pipe[0] <= source_in;
            generation_pipe[0] <= sample_generation;
            request_pipe[0] <= sample_request;
            for (index = 1; index < STAGES; index = index + 1) begin
                source_pipe[index] <= source_pipe[index-1];
                generation_pipe[index] <= generation_pipe[index-1];
                request_pipe[index] <= request_pipe[index-1];
            end
        end
    end
endmodule
