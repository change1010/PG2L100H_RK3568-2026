`timescale 1ns/1ps
// 按有效像素计数进行居中裁剪。
// 保留旧参数拼写以兼容现有例化。
module cmos_tailor #(
    parameter INPUT_WDITH = 1280,
    parameter INPUT_HIGHT = 720,
    parameter OUTPUT_WDITH = 960,
    parameter OUTPUT_HIGHT = 540
)(
    input rst_n,
    input cam_pclk,
    input cam_vsync,
    input cam_href,
    input [15:0] cam_data,
    input cam_data_valid,
    output reg cmos_frame_valid,
    output reg [15:0] cmos_frame_data
);
    localparam LEFT = (INPUT_WDITH - OUTPUT_WDITH) / 2;
    localparam RIGHT = LEFT + OUTPUT_WDITH;
    localparam TOP = (INPUT_HIGHT - OUTPUT_HIGHT) / 2;
    localparam BOTTOM = TOP + OUTPUT_HIGHT;
    reg [1:0] reset_release;
    wire crop_rst_n = reset_release[1];
    reg href_previous;
    reg [11:0] h_cnt;
    reg [11:0] v_cnt;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) reset_release <= 0;
        else reset_release <= {reset_release[0], 1'b1};
    end
    always @(posedge cam_pclk or negedge crop_rst_n) begin
        if (!crop_rst_n) begin
            href_previous <= 0;
            h_cnt <= 0;
            v_cnt <= 0;
            cmos_frame_valid <= 0;
            cmos_frame_data <= 0;
        end else begin
            href_previous <= cam_href;
            cmos_frame_valid <= 0;
            if (cam_vsync) begin
                h_cnt <= 0;
                v_cnt <= 0;
            end else begin
                if (!cam_href) begin
                    h_cnt <= 0;
                    if (href_previous && v_cnt < INPUT_HIGHT)
                        v_cnt <= v_cnt + 1'b1;
                end else if (cam_data_valid) begin
                    // 异常长行到上限后停止计数，避免回绕后误输出。
                    if (h_cnt < INPUT_WDITH) h_cnt <= h_cnt + 1'b1;
                    if (h_cnt >= LEFT && h_cnt < RIGHT &&
                        v_cnt >= TOP && v_cnt < BOTTOM) begin
                        cmos_frame_valid <= 1;
                        cmos_frame_data <= cam_data;
                    end
                end
            end
        end
    end
endmodule
