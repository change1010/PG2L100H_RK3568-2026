`timescale 1ns / 1ps
// 仅处理本地 HDMI 显示；PCIe 直接采集上游 color_canvas。
// 各模式统一延迟 9 拍；降噪采用官方灰度高斯滤波。
// 窗口以当前像素为右下角；前两行、列在降噪时保留原灰度，
// 在边缘模式输出黑色；四画面分别判断边界，避免跨画面取样。
module display_process #(
    parameter integer IMG_WIDTH = 1920,
    parameter integer IMG_HEIGHT = 1080,
    parameter integer BINARY_THRESHOLD = 128,
    parameter integer EDGE_THRESHOLD = 128
)(
    input clk, input rst, input [2:0] process_mode, input quad_mode,
    input timing_hs, input timing_vs, input timing_de,
    input [15:0] color_canvas_data,
    output reg display_hs, output reg display_vs, output reg display_de,
    output reg [15:0] display_canvas_data
);
    reg [7:0] quad_pipe;
    wire y_hs, y_vs, y_de;
    wire [7:0] gray;
    stage1_rgb_to_ycbcr u_gray (
        .clk(clk), .rst_n(~rst), .vsync_in(timing_vs), .hsync_in(timing_hs),
        .de_in(timing_de), .red(color_canvas_data[15:11]),
        .green(color_canvas_data[10:5]), .blue(color_canvas_data[4:0]),
        .vsync_out(y_vs), .hsync_out(y_hs), .de_out(y_de), .y(gray), .cb(), .cr()
    );
    wire binary;
    stage1_binarization #(.Binar_THRESHOLD(BINARY_THRESHOLD)) u_binary (
        .clk(clk), .rst_n(~rst), .vsync_in(y_vs), .hsync_in(y_hs),
        .de_in(y_de), .y_in(gray), .vsync_out(), .hsync_out(), .de_out(), .pix(binary)
    );
    wire window_valid;
    wire [7:0] m11,m12,m13,m21,m22,m23,m31,m32,m33;
    stage1_matrix #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_matrix (
        .clk(clk), .rst_n(~rst), .vs(y_vs), .de(y_de), .quad(quad_pipe[2]), .data(gray),
        .valid(window_valid), .m11(m11),.m12(m12),.m13(m13),
        .m21(m21),.m22(m22),.m23(m23),.m31(m31),.m32(m32),.m33(m33)
    );
    wire [7:0] smooth, edge_data;
    stage1_gauss_filter u_smooth (
        .video_clk(clk),.rst_n(~rst),.matrix_de(window_valid),.matrix_vs(1'b0),
        .matrix11(m11),.matrix12(m12),.matrix13(m13),
        .matrix21(m21),.matrix22(m22),.matrix23(m23),
        .matrix31(m31),.matrix32(m32),.matrix33(m33),
        .gauss_filter_vs(),.gauss_filter_de(),.gauss_filter_data(smooth)
    );
    stage1_sobel #(.SOBEL_THRESHOLD(EDGE_THRESHOLD)) u_edge (
        .video_clk(clk),.rst_n(~rst),.matrix_de(window_valid),.matrix_vs(1'b0),
        .matrix11(m11),.matrix12(m12),.matrix13(m13),
        .matrix21(m21),.matrix22(m22),.matrix23(m23),
        .matrix31(m31),.matrix32(m32),.matrix33(m33),
        .sobel_vs(),.sobel_de(),.sobel_data(edge_data)
    );
    reg [15:0] raw_pipe [0:7];
    reg [2:0] mode_pipe [0:7];
    reg [7:0] hs_pipe, vs_pipe, de_pipe;
    reg [7:0] gray_pipe [0:4];
    reg [3:0] binary_pipe;
    reg [2:0] valid_pipe;
    integer i;
    function [15:0] gray565;
        input [7:0] value;
        begin gray565 = {value[7:3],value[7:2],value[7:3]}; end
    endfunction
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            hs_pipe<=0; vs_pipe<=0; de_pipe<=0; quad_pipe<=0;
            binary_pipe<=0; valid_pipe<=0;
            display_hs<=0; display_vs<=0; display_de<=0; display_canvas_data<=0;
            for (i=0;i<8;i=i+1) begin raw_pipe[i]<=0; mode_pipe[i]<=0; end
            for (i=0;i<5;i=i+1) gray_pipe[i]<=0;
        end else begin
            hs_pipe <= {hs_pipe[6:0],timing_hs};
            vs_pipe <= {vs_pipe[6:0],timing_vs};
            de_pipe <= {de_pipe[6:0],timing_de};
            quad_pipe <= {quad_pipe[6:0],quad_mode};
            raw_pipe[0] <= color_canvas_data; mode_pipe[0] <= process_mode;
            for (i=1;i<8;i=i+1) begin raw_pipe[i]<=raw_pipe[i-1]; mode_pipe[i]<=mode_pipe[i-1]; end
            gray_pipe[0] <= gray;
            for (i=1;i<5;i=i+1) gray_pipe[i]<=gray_pipe[i-1];
            binary_pipe <= {binary_pipe[2:0],binary};
            valid_pipe <= {valid_pipe[1:0],window_valid};
            display_hs<=hs_pipe[7]; display_vs<=vs_pipe[7]; display_de<=de_pipe[7];
            if (!de_pipe[7]) display_canvas_data<=0;
            else case (mode_pipe[7])
                0: display_canvas_data<=raw_pipe[7];
                1: display_canvas_data<=gray565(valid_pipe[2] ? smooth : gray_pipe[4]);
                2: display_canvas_data<=gray565(gray_pipe[4]);
                3: display_canvas_data<=binary_pipe[3] ? 16'hffff : 16'h0000;
                4: display_canvas_data<=valid_pipe[2] ? gray565(edge_data) : 16'h0000;
                default: display_canvas_data<=raw_pipe[7];
            endcase
        end
    end
endmodule
