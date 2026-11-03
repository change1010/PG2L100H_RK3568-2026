`timescale 1ns/1ps
// 基于官方 matrix_3x3 生成当前像素及其左上方的 3x3 窗口。
// 三块行 RAM 轮转以避免同址读写。
// 用有效位屏蔽旧帧和四画面边界，无需复位 RAM。
module stage1_matrix #(
    parameter integer IMG_WIDTH = 1920,
    parameter integer IMG_HEIGHT = 1080
)(
    input clk, input rst_n, input vs, input de, input quad,
    input [7:0] data,
    output reg valid,
    output reg [7:0] m11, m12, m13, m21, m22, m23, m31, m32, m33
);
    reg [7:0] bank0 [0:IMG_WIDTH-1];
    reg [7:0] bank1 [0:IMG_WIDTH-1];
    reg [7:0] bank2 [0:IMG_WIDTH-1];
    reg [7:0] r0, r1, r2, data_d;
    reg [1:0] bank, bank_d;
    reg [11:0] x, y;
    reg de_d, valid_d, start_d, vs_d;
    wire frame_start = vs && !vs_d;
    // RAM 读延迟固定一拍，不受 DE 间隙影响。
    always @(posedge clk) begin
        r0 <= bank0[x]; r1 <= bank1[x]; r2 <= bank2[x];
        if (rst_n && de) begin
            case (bank)
                0: bank0[x] <= data;
                1: bank1[x] <= data;
                2: bank2[x] <= data;
            endcase
        end
    end
    wire [7:0] top_data = bank_d == 0 ? r1 : bank_d == 1 ? r2 : r0;
    wire [7:0] mid_data = bank_d == 0 ? r2 : bank_d == 1 ? r0 : r1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 0; y <= 0; bank <= 0; bank_d <= 0; data_d <= 0;
            de_d <= 0; valid_d <= 0; start_d <= 0; valid <= 0; vs_d <= 0;
            {m11,m12,m13,m21,m22,m23,m31,m32,m33} <= 0;
        end else begin
            vs_d <= vs;
            de_d <= de;
            data_d <= data;
            bank_d <= bank;
            start_d <= x == 0 || (quad && x == IMG_WIDTH/2);
            valid_d <= de && x >= 2 && y >= 2 && y < IMG_HEIGHT &&
                (!quad || ((x < IMG_WIDTH/2 || x >= IMG_WIDTH/2+2) &&
                           (y < IMG_HEIGHT/2 || y >= IMG_HEIGHT/2+2)));
            if (frame_start) begin x <= 0; y <= 0; bank <= 0; end
            else if (de) begin
                if (x == IMG_WIDTH-1) begin
                    x <= 0;
                    y <= y == IMG_HEIGHT-1 ? 0 : y + 1'b1;
                    bank <= bank == 2 ? 0 : bank + 1'b1;
                end else x <= x + 1'b1;
            end
            valid <= valid_d && de_d;
            if (de_d) begin
                {m11,m12,m13} <= start_d ? {16'd0,top_data} : {m12,m13,top_data};
                {m21,m22,m23} <= start_d ? {16'd0,mid_data} : {m22,m23,mid_data};
                {m31,m32,m33} <= start_d ? {16'd0,data_d} : {m32,m33,data_d};
            end
        end
    end
endmodule
