`timescale 1ns/1ps
module rgb_small_channel #(
    parameter integer WIDTH=1920, HEIGHT=1080, FIFO_BITS=9,
    parameter [24:0] BASE=25'd2621440
) (
    input rst_n, tx_rst_n, pix_clk, mem_clk, tx_clk,
    input boundary, de, invalid, input [15:0] pixel,
    input [31:0] source_sequence, source_epoch, source_config, image_generation,
    input [2:0] source_layout,
    input enable, input [3:0] period_div,
    output wr_req, output [9:0] wr_len, output [24:0] wr_addr, output [255:0] wr_data,
    input wr_data_req, wr_finish, wr_fault,
    output rd_req, output [24:0] rd_addr,
    input rd_valid, rd_finish, rd_fault, input [255:0] rd_data,
    output frame_valid, output [31:0] frame_seq, frame_epoch, frame_config,
    output [2:0] frame_layout, input frame_accept, frame_done,
    output [127:0] data, output data_valid, data_error, input data_ready,
    output [127:0] mem_status, pix_status
);
    localparam [15:0] FRAME_BEATS=(WIDTH/3)*(HEIGHT/3)*3/32;
    (* ASYNC_REG = "TRUE" *) reg [2:0] pix_reset, mem_reset, fifo_mem_reset, tx_alive;
    wire capture_rst_n=rst_n && tx_rst_n;
    always @(posedge pix_clk or negedge capture_rst_n)
        if(!capture_rst_n) pix_reset<=0; else pix_reset<={pix_reset[1:0],1'b1};
    always @(posedge mem_clk or negedge capture_rst_n)
        if(!capture_rst_n) fifo_mem_reset<=0; else fifo_mem_reset<={fifo_mem_reset[1:0],1'b1};
    always @(posedge mem_clk or negedge rst_n)
        if(!rst_n) mem_reset<=0; else mem_reset<={mem_reset[1:0],1'b1};
    always @(posedge mem_clk or negedge tx_rst_n)
        if(!tx_rst_n) tx_alive<=0; else tx_alive<={tx_alive[1:0],1'b1};
    wire put_valid, put_ready, get_valid, get_ready;
    wire [381:0] put_token, get_token;
    wire committed, locked, writer_busy, reader_busy, reader_start;
    wire [1:0] completed_slot, locked_slot;
    wire [31:0] seq_done, epoch_done, config_done, commits, rejects, capture_rejects, skipped;
    wire [2:0] layout_done;
    wire [24:0] reader_base;
    wire [127:0] reader_data;
    wire reader_valid, reader_error, reader_ready;
    rgb_small_capture #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_capture (
        .clk(pix_clk),.rst_n(pix_reset[2]),.boundary(boundary),.de(de),.invalid(invalid),.pixel(pixel),
        .enable(enable),.period_div(period_div),.source_sequence(source_sequence),.source_epoch(source_epoch),
        .source_config(source_config),.image_generation(image_generation),.source_layout(source_layout),
        .token_valid(put_valid),.token_ready(put_ready),.token(put_token),.rejected_frames(capture_rejects)
    );
    video_token_fifo_bram #(.WIDTH(382),.ADDR_BITS(FIFO_BITS)) u_tokens (
        .wr_clk(pix_clk),.wr_rst_n(pix_reset[2]),.wr_valid(put_valid),.wr_ready(put_ready),.wr_data(put_token),
        .rd_clk(mem_clk),.rd_rst_n(fifo_mem_reset[2]),.rd_valid(get_valid),.rd_ready(get_ready),.rd_data(get_token)
    );
    rgb_small_writer #(.FRAME_BEATS(FRAME_BEATS),.BASE(BASE)) u_writer (
        .clk(mem_clk),.rst_n(mem_reset[2]),.session_alive(tx_alive[2]),
        .token_valid(get_valid),.token_ready(get_ready),.token(get_token),
        .slot_locked(locked),.locked_slot(locked_slot),.wr_req(wr_req),.wr_len(wr_len),.wr_addr(wr_addr),
        .wr_data(wr_data),.wr_data_req(wr_data_req),.wr_finish(wr_finish),.wr_fault(wr_fault),
        .frame_done(committed),.completed_slot(completed_slot),.completed_sequence(seq_done),
        .completed_epoch(epoch_done),.completed_config(config_done),.completed_layout(layout_done),
        .committed_frames(commits),.rejected_frames(rejects),.busy(writer_busy)
    );
    road_plane_cdc u_mailbox (
        .rst_n(rst_n),.tx_rst_n(tx_rst_n),.mem_clk(mem_clk),.tx_clk(tx_clk),
        .frame_done(committed),.slot_index(completed_slot),.frame_seq(seq_done),.frame_epoch(epoch_done),
        .frame_config(config_done),.frame_layout(layout_done),.cfg_base(BASE),.cfg_slot_beats(FRAME_BEATS),
        .slot_locked(locked),.locked_slot(locked_slot),.skipped_frames(skipped),
        .reader_start(reader_start),.reader_base(reader_base),.reader_data(reader_data),
        .reader_valid(reader_valid),.reader_error(reader_error),.reader_busy(reader_busy),.reader_ready(reader_ready),
        .frame_valid(frame_valid),.tx_frame_seq(frame_seq),.tx_frame_epoch(frame_epoch),
        .tx_frame_config(frame_config),.tx_frame_layout(frame_layout),.frame_accept(frame_accept),
        .tx_frame_done(frame_done),.data(data),.data_valid(data_valid),.data_error(data_error),.data_ready(data_ready)
    );
    road_plane_reader #(.TOTAL_BEATS(FRAME_BEATS)) u_reader (
        .mem_clk(mem_clk),.rst_n(mem_reset[2]),.start(reader_start),.abort(!tx_alive[2]),
        .cfg_base_beat(reader_base),.cfg_slot_beats(FRAME_BEATS),.slot_idx(2'd0),
        .rd_req(rd_req),.rd_addr(rd_addr),.rd_valid(rd_valid),.rd_data(rd_data),.rd_finish(rd_finish),
        .rd_fault(rd_fault),.data(reader_data),.data_valid(reader_valid),.data_error(reader_error),
        .data_ready(reader_ready),.done(),.bursts_done(),.error(),.busy(reader_busy)
    );
    // 计数器整体冻结后跨域传递，避免各位独立同步产生混合值。
    stage1_status_cdc u_mem_status (
        .src_clk(mem_clk),.dst_clk(tx_clk),.rst_n(rst_n && tx_rst_n),
        .source_state({24'd0,rd_fault,wr_fault,reader_busy,writer_busy,locked,1'b0,locked_slot,
                       skipped,rejects,commits}),.state_snapshot(mem_status),.generation()
    );
    stage1_status_cdc u_pix_status (
        .src_clk(pix_clk),.dst_clk(tx_clk),.rst_n(rst_n && tx_rst_n),
        .source_state({96'd0,capture_rejects}),.state_snapshot(pix_status),.generation()
    );
endmodule
