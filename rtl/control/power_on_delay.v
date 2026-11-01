`timescale 1ns/1ps
// 板载晶振为 27 MHz；异步置复位保证时钟停止时仍可复位。
module power_on_delay(
    input clk_sys,
    input reset_n,
    output camera1_rstn,
    output camera2_rstn,
    output camera_pwnd,
    output reg initial_en
);
    reg [1:0] reset_release;
    reg [18:0] cnt1;
    reg [15:0] cnt2;
    reg [19:0] cnt3;
    reg camera_rstn_reg;
    reg camera_pwnd_reg;
    wire sequence_rst_n = reset_release[1];
    assign camera1_rstn = camera_rstn_reg;
    assign camera2_rstn = camera_rstn_reg;
    assign camera_pwnd = camera_pwnd_reg;
    always @(posedge clk_sys or negedge reset_n) begin
        if (!reset_n) reset_release <= 2'b00;
        else reset_release <= {reset_release[0], 1'b1};
    end
    always @(posedge clk_sys or negedge sequence_rst_n) begin
        if (!sequence_rst_n) begin
            cnt1 <= 0;
            cnt2 <= 0;
            cnt3 <= 0;
            camera_pwnd_reg <= 1'b1;
            camera_rstn_reg <= 1'b0;
            initial_en <= 1'b0;
        end else begin
            if (cnt1 < 19'd135000) cnt1 <= cnt1 + 1'b1;
            else camera_pwnd_reg <= 1'b0;
            if (!camera_pwnd_reg) begin
                if (cnt2 < 16'd35100) cnt2 <= cnt2 + 1'b1;
                else camera_rstn_reg <= 1'b1;
            end
            if (camera_rstn_reg) begin
                if (cnt3 < 20'd567000) cnt3 <= cnt3 + 1'b1;
                else initial_en <= 1'b1;
            end
        end
    end
endmodule
