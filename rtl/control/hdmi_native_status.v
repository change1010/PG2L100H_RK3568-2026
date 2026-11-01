`timescale 1ns/1ps
// 高位字返回原生 HDMI 诊断，已退役的源控制低位字返回零。
// 各时钟域分别冻结快照后跨域传递。
module hdmi_native_status (
    input wire rst_n, in_clk, mem_clk, pix_clk, bus_clk,
    input wire [31:0] capture_rejected, committed, rejected, underflows, missed,
    input wire frame_valid, source_live,
    input wire read_en, input wire [31:0] read_addr,
    output reg [127:0] read_data
);
    wire [127:0] capture_snapshot, writer_snapshot, reader_snapshot;
    stage1_status_cdc capture_status (
        .rst_n(rst_n),.src_clk(in_clk),.dst_clk(bus_clk),
        .source_state({96'd0,capture_rejected}),.state_snapshot(capture_snapshot),.generation()
    );
    stage1_status_cdc writer_status (
        .rst_n(rst_n),.src_clk(mem_clk),.dst_clk(bus_clk),
        .source_state({31'd0,source_live,32'd0,rejected,committed}),
        .state_snapshot(writer_snapshot),.generation()
    );
    stage1_status_cdc reader_status (
        .rst_n(rst_n),.src_clk(pix_clk),.dst_clk(bus_clk),
        .source_state({63'd0,frame_valid,missed,underflows}),
        .state_snapshot(reader_snapshot),.generation()
    );
    always @(posedge bus_clk or negedge rst_n) begin
        if (!rst_n) read_data <= 0;
        else if (read_en) case (read_addr)
            32'h17d: read_data <= {capture_snapshot[31:0],writer_snapshot[63:0],32'd0};
            32'h17e: read_data <= {30'd0,writer_snapshot[96],reader_snapshot[64],
                                  reader_snapshot[63:0],32'd0};
            32'h17f: read_data <= {32'd129600,32'd1,32'h48313038,32'd0};
            default: read_data <= 0;
        endcase
    end
endmodule
