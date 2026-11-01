`timescale 1ns/1ps
module hdmi_native_source #(
    parameter integer WIDTH = 1920, HEIGHT = 1080, FIFO_BITS = 9,
    parameter [24:0] BASE = 25'd2097152,
    parameter integer SOURCE_TIMEOUT = 12500000
) (
    input wire rst_n, in_clk, mem_clk, pix_clk,
    input wire in_vs, in_de, input wire [15:0] in_pixel,
    input wire frame_start, display_enable, quad_mode, read_en,
    output wire [15:0] read_data,
    output wire frame_valid,
    output wire source_live,
    output wire wr_req, output wire [9:0] wr_len,
    output wire [24:0] wr_addr, output wire [255:0] wr_data,
    input wire wr_data_req, wr_finish, wr_fault,
    output wire rd_req, output wire [9:0] rd_len, output wire [24:0] rd_addr,
    input wire rd_valid, rd_finish, rd_fault, input wire [255:0] rd_data,
    output wire [31:0] committed_frames, rejected_frames,
    output wire [31:0] capture_rejected, underflow_frames, missed_requests,
    output frame_boundary, output [31:0] source_sequence, source_epoch,
    output [63:0] source_timestamp_ticks, output [127:0] native_identity,
    output [127:0] native_left_check,output [63:0] native_left_counts
);
    (* ASYNC_REG="TRUE" *) reg [2:0] in_reset, mem_reset;
    always @(posedge in_clk or negedge rst_n)
        if (!rst_n) in_reset <= 0; else in_reset <= {in_reset[1:0],1'b1};
    always @(posedge mem_clk or negedge rst_n)
        if (!rst_n) mem_reset <= 0; else mem_reset <= {mem_reset[1:0],1'b1};
    wire put_valid, put_ready, get_valid, get_ready;
    wire [314:0] put_token, get_token;
    wire read_owned, latest_valid, frame_committed;
    wire [1:0] read_slot, latest_slot;
    wire [31:0] latest_sequence;
    wire [31:0] latest_epoch;
    wire [31:0] latest_left_crc;
    wire [63:0] latest_timestamp;
    hdmi_native_capture #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_capture (
        .clk(in_clk),.rst_n(in_reset[2]),.vs(in_vs),.de(in_de),.pixel(in_pixel),
        .token_valid(put_valid),.token_ready(put_ready),.token(put_token),
        .rejected_frames(capture_rejected),.frame_boundary(frame_boundary),
        .source_sequence(source_sequence),.source_epoch(source_epoch),
        .source_timestamp_ticks(source_timestamp_ticks)
    );
    video_token_fifo_bram #(.WIDTH(315),.ADDR_BITS(FIFO_BITS)) u_tokens (
        .wr_clk(in_clk),.wr_rst_n(in_reset[2]),.wr_valid(put_valid),
        .wr_ready(put_ready),.wr_data(put_token),
        .rd_clk(mem_clk),.rd_rst_n(mem_reset[2]),.rd_valid(get_valid),
        .rd_ready(get_ready),.rd_data(get_token)
    );
    hdmi_native_writer #(.FRAME_BEATS(WIDTH*HEIGHT/16),.BASE(BASE),
        .SOURCE_TIMEOUT(SOURCE_TIMEOUT)) u_writer (
        .clk(mem_clk),.rst_n(mem_reset[2]),.token_valid(get_valid),
        .token_ready(get_ready),.token(get_token),.read_owned(read_owned),.read_slot(read_slot),
        .wr_req(wr_req),.wr_len(wr_len),.wr_addr(wr_addr),.wr_data(wr_data),
        .wr_data_req(wr_data_req),.wr_finish(wr_finish),.wr_fault(wr_fault),
        .latest_valid(latest_valid),.source_live(source_live),.latest_slot(latest_slot),
        .latest_sequence(latest_sequence),.frame_committed(frame_committed),
        .committed_frames(committed_frames),.rejected_frames(rejected_frames),
        .latest_epoch(latest_epoch),.latest_timestamp(latest_timestamp),.latest_left_crc(latest_left_crc)
    );
    hdmi_native_reader #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.BASE(BASE)) u_reader (
        .rst_n(rst_n),.mem_clk(mem_clk),.pix_clk(pix_clk),
        .frame_start(frame_start),.display_enable(display_enable),.quad_mode(quad_mode),
        .read_en(read_en),.read_data(read_data),.frame_valid(frame_valid),
        .underflow_frames(underflow_frames),.missed_requests(missed_requests),
        .latest_valid(latest_valid),.source_live(source_live),.latest_slot(latest_slot),
        .latest_sequence(latest_sequence),.read_owned(read_owned),.read_slot(read_slot),
        .latest_epoch(latest_epoch),.latest_timestamp(latest_timestamp),.native_identity(native_identity),
        .latest_left_crc(latest_left_crc),.native_left_check(native_left_check),.native_left_counts(native_left_counts),
        .rd_req(rd_req),.rd_len(rd_len),.rd_addr(rd_addr),.rd_valid(rd_valid),
        .rd_finish(rd_finish),.rd_fault(rd_fault),.rd_data(rd_data)
    );
`ifndef SYNTHESIS
    initial begin
        if (WIDTH % 32 != 0 || HEIGHT % 2 != 0 || (WIDTH*HEIGHT/16) % 16 != 0)
            $fatal(1,"native geometry must have whole 16-beat frames and 32-pixel/even rows");
    end
`endif
endmodule
