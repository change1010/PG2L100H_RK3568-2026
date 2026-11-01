`timescale 1ns/1ps

// 传递最新状态；数据冻结到往返握手确认后才能更新。
// 两域共用异步复位，各自在本时钟域同步释放。
module dark_enhance_snapshot_cdc #(
    parameter integer WIDTH = 288
)(
    input rst_n, input pix_clk, input bus_clk,
    input [WIDTH-1:0] state_pix,
    output reg [WIDTH-1:0] snapshot_bus,
    output reg [31:0] revision_bus
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] pix_reset, bus_reset;
    wire pix_rst_n = pix_reset[2], bus_rst_n = bus_reset[2];
    always @(posedge pix_clk or negedge rst_n)
        if (!rst_n) pix_reset <= 0; else pix_reset <= {pix_reset[1:0],1'b1};
    always @(posedge bus_clk or negedge rst_n)
        if (!rst_n) bus_reset <= 0; else bus_reset <= {bus_reset[1:0],1'b1};

    reg [WIDTH-1:0] frozen_pix;
    reg toggle_pix, ack_bus, seen_bus, capture_wait_bus;
    (* ASYNC_REG = "TRUE" *) reg ack_meta_pix, ack_synced_pix;
    (* ASYNC_REG = "TRUE" *) reg toggle_meta_bus, toggle_synced_bus;
    wire [1:0] ack_sync_pix = {ack_synced_pix,ack_meta_pix};
    wire [1:0] toggle_sync_bus = {toggle_synced_bus,toggle_meta_bus};

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            frozen_pix <= 0; toggle_pix <= 0; ack_meta_pix <= 0; ack_synced_pix <= 0;
        end else begin
            ack_meta_pix <= ack_bus; ack_synced_pix <= ack_meta_pix;
            if (ack_sync_pix[1] == toggle_pix && state_pix != frozen_pix) begin
                frozen_pix <= state_pix;
                toggle_pix <= ~toggle_pix;
            end
        end
    end
    always @(posedge bus_clk or negedge bus_rst_n) begin
        if (!bus_rst_n) begin
            toggle_meta_bus <= 0; toggle_synced_bus <= 0; ack_bus <= 0; seen_bus <= 0;
            capture_wait_bus <= 0; snapshot_bus <= 0; revision_bus <= 0;
        end else begin
            toggle_meta_bus <= toggle_pix; toggle_synced_bus <= toggle_meta_bus;
            if (!capture_wait_bus && toggle_sync_bus[1] != seen_bus) begin
                seen_bus <= toggle_sync_bus[1];
                capture_wait_bus <= 1;
            end else if (capture_wait_bus) begin
                snapshot_bus <= frozen_pix;
                revision_bus <= revision_bus + 1'b1;
                ack_bus <= seen_bus;
                capture_wait_bus <= 0;
            end
        end
    end
endmodule
