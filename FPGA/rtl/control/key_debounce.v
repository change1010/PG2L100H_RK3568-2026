`timescale 1ns / 1ps
// 按键低电平有效；148.5 MHz 下连续稳定 2,970,000 拍对应 20 ms。
module key_debounce #(
    parameter integer CNT_BITS = 22,
    parameter integer STABLE_CYCLES = 2970000
)(
    input clk, input rst_n, input key_n,
    output press, output level
);
    reg sync1, sync2, clean, clean_d;
    reg [CNT_BITS-1:0] count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync1 <= 1'b1;
            sync2 <= 1'b1;
            clean <= 1'b1;
            clean_d <= 1'b1;
            count <= 0;
        end else begin
            sync1 <= key_n;
            sync2 <= sync1;
            clean_d <= clean;
            if (sync2 == clean) count <= 0;
            else if (count == STABLE_CYCLES - 1) begin
                clean <= sync2;
                count <= 0;
            end else count <= count + 1'b1;
        end
    end
    assign level = clean;
    assign press = clean_d && !clean;
endmodule
