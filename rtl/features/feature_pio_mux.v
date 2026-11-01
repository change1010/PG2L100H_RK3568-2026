`timescale 1ns/1ps
// PCIe 核选中 BAR0 后仍保留主机地址，单位为 128 位字。
// BAR0 按 8 KiB 对齐，取字地址低 9 位作为寄存器偏移。
module feature_pio_mux #(
    parameter integer ENABLE_DARK = 0
)(
    input clk,input rst_n,
    input read_en,input [31:0] read_addr,
    input write_en,input [15:0] write_byte_en,input [31:0] write_addr,input [127:0] write_data,
    input [127:0] legacy_data,input [127:0] feature_data,
    output feature_read_en,output feature_write_en,
    output [31:0] feature_write_data,output [127:0] result_data,
    output [31:0] read_offset,output [31:0] write_offset
);
    assign read_offset={23'd0,read_addr[8:0]};
    assign write_offset={23'd0,write_addr[8:0]};
    // 保留已退役的旧 ROAD 寄存器和包 RAM 地址，避免暴露旧 RAM 别名。
    // 这些地址读零、写忽略，
    // 使旧驱动在能力检查阶段失败。
    wire read_selected=(read_offset>=32'h0c0 && read_offset<=32'h0ff) ||
        (read_offset>=32'h127 && read_offset<=32'h13f) ||
        (read_offset>=32'h140 && read_offset<=32'h15f) ||
        (ENABLE_DARK!=0 && read_offset>=32'h160 && read_offset<=32'h176) ||
        (read_offset>=32'h177 && read_offset<=32'h17f) ||
        (read_offset>=32'h180 && read_offset<=32'h1ff);
    wire write_selected=(write_offset>=32'h0c0 && write_offset<=32'h0ff) ||
        (write_offset>=32'h127 && write_offset<=32'h13f) ||
        (write_offset>=32'h140 && write_offset<=32'h15e) ||
        (ENABLE_DARK!=0 && write_offset>=32'h160 && write_offset<=32'h176);
    reg feature_selected;
    assign feature_read_en=read_en && read_selected;
    assign feature_write_en=write_en && write_selected && &write_byte_en[3:0];
    assign feature_write_data=write_data[31:0];
    assign result_data=feature_selected ? feature_data : legacy_data;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) feature_selected<=0;
        else if(read_en) feature_selected<=read_selected;
endmodule
