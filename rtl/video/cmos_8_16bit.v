`timescale 1ns/1ps
// 字节拼接、行场时序及下游写入均使用摄像头像素时钟。
// de_o 每个像素仅有效一拍，pixel_href 覆盖整段有效行。
module cmos_8_16bit(
    input pclk,
    input rst_n,
    input de_i,
    input [7:0] pdata_i,
    input vs_i,
    output pixel_clk,
    output reg de_o,
    output reg [15:0] pdata_o,
    output reg pixel_href,
    output reg pixel_vsync
);
    reg [1:0] reset_release;
    wire capture_rst_n = reset_release[1];
    reg vs_previous;
    reg frame_started;
    reg second_byte;
    reg [7:0] first_byte;
    assign pixel_clk = pclk; // 兼容旧端口名，实际不分频
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) reset_release <= 0;
        else reset_release <= {reset_release[0], 1'b1};
    end
    always @(posedge pclk or negedge capture_rst_n) begin
        if (!capture_rst_n) begin
            vs_previous <= 0;
            frame_started <= 0;
            second_byte <= 0;
            first_byte <= 0;
            de_o <= 0;
            pdata_o <= 0;
            pixel_href <= 0;
            pixel_vsync <= 0;
        end else begin
            vs_previous <= vs_i;
            if (vs_i && !vs_previous) frame_started <= 1;
            pixel_vsync <= vs_i;
            pixel_href <= frame_started && !vs_i && de_i;
            de_o <= 0;
            if (!frame_started || vs_i || !de_i) begin
                // 行场中断时丢弃未拼完的字节，避免下一像素错位。
                second_byte <= 0;
            end else if (!second_byte) begin
                first_byte <= pdata_i;
                second_byte <= 1;
            end else begin
                pdata_o <= {first_byte, pdata_i};
                de_o <= 1;
                second_byte <= 0;
            end
        end
    end
endmodule
