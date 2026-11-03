`timescale 1ns/1ps

// 在像素域关联增强参数与采集帧。
// 结果须经快照握手后供 RK 读取。
// first_capture_valid 只表示采集已开始，不表示 DMA 已完成。
module dark_enhance_frame_tracker (
    input rst_n,
    input pix_clk,
    input capture_boundary,
    input capture_begin,
    input [31:0] capture_epoch,
    input [31:0] capture_sequence,
    input [2:0] capture_source,
    input [31:0] image_generation,
    input [31:0] image_request,
    output reg applied_valid,
    output reg [31:0] applied_generation,
    output reg [31:0] applied_request,
    output reg [31:0] applied_epoch,
    output reg [31:0] applied_sequence,
    output reg [2:0] applied_source,
    output reg first_capture_valid,
    output reg [31:0] first_capture_epoch,
    output reg [31:0] first_capture_sequence,
    output reg [2:0] first_capture_source
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] pix_reset_pipe;
    wire pix_rst_n = pix_reset_pipe[2];
    always @(posedge pix_clk or negedge rst_n)
        if (!rst_n) pix_reset_pipe <= 0;
        else pix_reset_pipe <= {pix_reset_pipe[1:0], 1'b1};

    wire new_image = image_generation != 0 &&
        (!applied_valid || image_generation != applied_generation);

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            applied_valid <= 0; applied_generation <= 0; applied_request <= 0;
            applied_epoch <= 0; applied_sequence <= 0; applied_source <= 0;
            first_capture_valid <= 0; first_capture_epoch <= 0;
            first_capture_sequence <= 0; first_capture_source <= 0;
        end else if (capture_boundary) begin
            if (new_image) begin
                applied_valid <= 1;
                applied_generation <= image_generation;
                applied_request <= image_request;
                applied_epoch <= capture_epoch;
                applied_sequence <= capture_sequence;
                applied_source <= capture_source;
                first_capture_valid <= 0;
                first_capture_epoch <= 0;
                first_capture_sequence <= 0;
                first_capture_source <= 0;
            end
            if (image_generation != 0 && capture_begin &&
                (new_image || !first_capture_valid)) begin
                first_capture_valid <= 1;
                first_capture_epoch <= capture_epoch;
                first_capture_sequence <= capture_sequence;
                first_capture_source <= capture_source;
            end
        end
    end
endmodule
