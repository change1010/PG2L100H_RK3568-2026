`timescale 1ns/1ps
module video_dma_frame_store #(
    parameter [21:0] FRAME_PIXELS = 22'd2073600,
    parameter [12:0] LINE_PIXELS = 13'd1920,
    parameter [24:0] DDR_BASE = 25'd524288,
    parameter integer TRACK_IMAGE_GENERATION = 0,
    parameter integer CHECK_PIX_INVALID = 0,
    parameter integer TRACK_NATIVE_IDENTITY = 0
)(
    input rst_n,
    input pix_clk,
    input pix_vs,
    input pix_de,
    input [15:0] pix_data,
    input pix_invalid,
    input [2:0] pix_mode,
    input [31:0] pix_config_generation,
    input [31:0] pix_image_generation,
    input [127:0] pix_native_identity,
    output capture_boundary,
    output capture_begin,
    output capture_end,
    output capture_invalid,
    output [31:0] capture_sequence,
    output [31:0] capture_epoch,
    input mem_clk,
    output reg wr_req,
    output reg [24:0] wr_addr,
    input wr_data_req,
    output reg [255:0] wr_data,
    input wr_finish,
    output reg rd_req,
    output reg [24:0] rd_addr,
    input rd_valid,
    input [255:0] rd_data,
    input rd_finish,
    input tx_clk,
    input run,
    output source_idle,
    output frame_valid,
    output [31:0] frame_seq,
    output [2:0] frame_mode,
    output [31:0] frame_epoch,
    output [31:0] frame_config_generation,
    output [127:0] frame_native_identity,
    input frame_accept,
    input frame_done,
    output data_valid,
    output [127:0] data,
    input data_ready,
    output reg [31:0] captured,
    output reg [31:0] dropped
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] pix_reset;
    wire pix_reset_n = pix_reset[2];
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) pix_reset <= 0;
        else pix_reset <= {pix_reset[1:0], 1'b1};
    end
    (* ASYNC_REG = "TRUE" *) reg [2:0] mem_reset;
    wire mem_reset_n = mem_reset[2];
    always @(posedge mem_clk or negedge rst_n) begin
        if (!rst_n) mem_reset <= 0;
        else mem_reset <= {mem_reset[1:0], 1'b1};
    end
    (* ASYNC_REG = "TRUE" *) reg [2:0] tx_reset;
    wire tx_reset_n = tx_reset[2];
    always @(posedge tx_clk or negedge rst_n) begin
        if (!rst_n) tx_reset <= 0;
        else tx_reset <= {tx_reset[1:0], 1'b1};
    end
    localparam [24:0] FRAME_BEATS = {7'd0, FRAME_PIXELS[21:4]};
    reg [255:0] capture_ram [0:127];
    reg [24:0] capture_addr [0:7];
    reg [7:0] capture_put;
    reg [7:0] capture_get;
    (* ASYNC_REG = "TRUE" *) reg [7:0] cap_get_meta, cap_get_sync;
    (* ASYNC_REG = "TRUE" *) reg [7:0] cap_put_meta, cap_put_sync;
    reg [2:0] write_bank, read_bank;
    reg [3:0] write_word, read_word;
    reg [3:0] pixel_word;
    reg [255:0] packed_pixels;
    reg [2:0] slot_end, slot_release;
    (* ASYNC_REG = "TRUE" *) reg [2:0] release_meta, release_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] end_meta, end_sync;
    reg [2:0] end_guard;
    reg [2:0] slot_taken, release_seen, end_seen;
    reg [31:0] slot_seq [0:2];
    reg [2:0] slot_mode [0:2];
    reg [31:0] slot_epoch [0:2];
    reg [31:0] slot_config_generation [0:2];
    reg [127:0] slot_native_identity [0:2];
    reg [31:0] stream_epoch;
    // 预先寄存下一个版本值，包含 32 位回绕；
    // 图像比较只需选择寄存值，避免再串接 32 位加法器。
    reg [31:0] stream_epoch_next;
    reg previous_session;
    reg source_interrupted;
    reg [2:0] previous_source;
    reg [31:0] previous_image_generation, capture_image_generation;
    reg [2:0] slot_bad;
    reg [1:0] capture_slot;
    reg capturing, bad_frame, vs_d, de_d;
    reg [31:0] frame_number;
    reg [21:0] pixel_count;
    reg [12:0] line_count;
    reg [24:0] capture_offset;
    reg drop_event;
    (* ASYNC_REG = "TRUE" *) reg [1:0] run_pix, run_mem;
    (* ASYNC_REG = "TRUE" *) reg [1:0] drop_sync;
    reg drop_seen;
    integer slot_index;
    reg free_found;
    reg [1:0] free_slot;
    always @* begin
        free_found = 0;
        free_slot = 0;
        if (!slot_taken[0]) begin free_found = 1; free_slot = 0; end
        else if (!slot_taken[1]) begin free_found = 1; free_slot = 1; end
        else if (!slot_taken[2]) begin free_found = 1; free_slot = 2; end
    end
    assign capture_boundary = vs_d && !pix_vs;
    assign capture_begin = capture_boundary && run_pix[1] && free_found;
    assign capture_end = capture_boundary && capturing;
    assign capture_invalid = bad_frame || pixel_count != FRAME_PIXELS;
    assign capture_sequence = frame_number;
    // STOP 期间或无空闲帧槽时也检查图像变化。
    // config_generation 保留配置确认含义，与图像版本独立。
    wire image_changed = TRACK_IMAGE_GENERATION != 0 &&
        previous_image_generation != pix_image_generation;
    wire epoch_changed = image_changed || (run_pix[1] &&
        (!previous_session || previous_source!=pix_mode ||
         (CHECK_PIX_INVALID != 0 && source_interrupted)));
    assign capture_epoch = epoch_changed ? stream_epoch_next : stream_epoch;
    always @(posedge pix_clk or negedge pix_reset_n) begin
        if (!pix_reset_n) begin
            cap_get_meta <= 0; cap_get_sync <= 0;
            release_meta <= 0; release_sync <= 0;
            run_pix <= 0;
        end else begin
            cap_get_meta <= capture_get; cap_get_sync <= cap_get_meta;
            release_meta <= slot_release; release_sync <= release_meta;
            run_pix <= {run_pix[0], run};
        end
    end
    always @(posedge pix_clk or negedge pix_reset_n) begin
        if (!pix_reset_n) begin
            capture_put <= 0; write_bank <= 0; write_word <= 0;
            pixel_word <= 0; packed_pixels <= 0; slot_end <= 0;
            slot_taken <= 0; release_seen <= 0; slot_bad <= 0;
            capture_slot <= 0; capturing <= 0; bad_frame <= 0;
            vs_d <= 0; de_d <= 0; frame_number <= 0; pixel_count <= 0;
            line_count <= 0; capture_offset <= 0; drop_event <= 0;
            stream_epoch<=0; stream_epoch_next<=1;
            previous_session<=0; previous_source<=0;
            source_interrupted <= 0;
            previous_image_generation <= 0; capture_image_generation <= 0;
            for (slot_index = 0; slot_index < 3; slot_index = slot_index + 1) begin
                slot_seq[slot_index] <= 0;
                slot_mode[slot_index] <= 0;
                slot_epoch[slot_index]<=0; slot_config_generation[slot_index]<=0;
                slot_native_identity[slot_index] <= 0;
            end
        end else begin
            vs_d <= pix_vs;
            de_d <= pix_de;
            if (!run_pix[1]) previous_session <= 0;
            if (CHECK_PIX_INVALID != 0 && pix_de && pix_invalid) source_interrupted <= 1;
            for (slot_index = 0; slot_index < 3; slot_index = slot_index + 1)
                if (release_sync[slot_index] != release_seen[slot_index]) begin
                    slot_taken[slot_index] <= 0;
                    release_seen[slot_index] <= release_sync[slot_index];
                end
            if (vs_d && !pix_vs) begin
                stream_epoch<=capture_epoch;
                if (epoch_changed) stream_epoch_next<=stream_epoch_next+1'b1;
                previous_session<=run_pix[1]; previous_source<=pix_mode;
                previous_image_generation <= pix_image_generation;
                source_interrupted <= 0;
                frame_number <= frame_number + 1'b1;
                if (capturing) begin
                    slot_bad[capture_slot] <= bad_frame || pixel_count != FRAME_PIXELS;
                    slot_end[capture_slot] <= !slot_end[capture_slot];
                end
                capturing <= run_pix[1] && free_found;
                bad_frame <= 0;
                pixel_count <= 0; line_count <= 0;
                pixel_word <= 0; write_word <= 0; capture_offset <= 0;
                if (run_pix[1] && free_found) begin
                    capture_slot <= free_slot;
                    slot_taken[free_slot] <= 1;
                    slot_seq[free_slot] <= frame_number;
                    slot_mode[free_slot] <= pix_mode;
                    slot_epoch[free_slot]<=capture_epoch;
                    slot_config_generation[free_slot]<=pix_config_generation;
                    capture_image_generation <= pix_image_generation;
                end else if (run_pix[1]) drop_event <= !drop_event;
            end else if (capturing) begin
                if (CHECK_PIX_INVALID != 0 && pix_de && pix_invalid) bad_frame <= 1;
                if (pix_mode != slot_mode[capture_slot]) bad_frame <= 1;
                if (TRACK_IMAGE_GENERATION != 0 &&
                    pix_image_generation != capture_image_generation) bad_frame <= 1;
                if (de_d && !pix_de) begin
                    if (line_count != LINE_PIXELS) bad_frame <= 1;
                    line_count <= 0;
                end
                if (pix_de) begin
                    // 原生读取端在 VS 之后、DE 之前选定源帧。
                    if (TRACK_NATIVE_IDENTITY != 0) begin
                        if (pixel_count == 0)
                            slot_native_identity[capture_slot] <= pix_native_identity;
                        else if (pix_native_identity != slot_native_identity[capture_slot])
                            bad_frame <= 1;
                    end
                    if (pixel_count < FRAME_PIXELS + 1) pixel_count <= pixel_count + 1'b1;
                    if (line_count < LINE_PIXELS + 1) line_count <= line_count + 1'b1;
                    if (pixel_count >= FRAME_PIXELS) bad_frame <= 1;
                    if (!bad_frame && pixel_count < FRAME_PIXELS) begin
                        if (capture_put[write_bank] != cap_get_sync[write_bank]) begin
                            bad_frame <= 1;
                        end else begin
                            packed_pixels[{pixel_word, 4'b0} +: 16] <= pix_data;
                            pixel_word <= pixel_word + 1'b1;
                            if (pixel_word == 15) begin
                                capture_ram[{write_bank, write_word}] <= {pix_data, packed_pixels[239:0]};
                                write_word <= write_word + 1'b1;
                                if (write_word == 15) begin
                                    capture_addr[write_bank] <= DDR_BASE + {23'd0, capture_slot} * FRAME_BEATS + capture_offset;
                                    capture_offset <= capture_offset + 16;
                                    capture_put[write_bank] <= !capture_put[write_bank];
                                    write_bank <= write_bank + 1'b1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    reg [2:0] ready_slots;
    reg [1:0] send_slot;
    reg descriptor;
    reg descriptor_claimed;
    reg [31:0] descriptor_seq;
    reg [2:0] descriptor_mode;
    reg [31:0] descriptor_epoch, descriptor_config_generation;
    reg [127:0] descriptor_native_identity;
    reg accepted_toggle, done_toggle;
    (* ASYNC_REG = "TRUE" *) reg [1:0] accept_sync, done_sync;
    reg accept_seen, done_seen;
    reg reading;
    reg [24:0] read_offset;
    reg [255:0] output_ram [0:127];
    reg [7:0] output_put, output_get;
    (* ASYNC_REG = "TRUE" *) reg [7:0] out_get_meta, out_get_sync;
    (* ASYNC_REG = "TRUE" *) reg [7:0] out_put_meta, out_put_sync;
    reg [2:0] output_write_bank, output_read_bank;
    reg [3:0] output_write_word;
    reg [4:0] output_read_word;
    reg [2:0] read_state;
    reg [1:0] write_state;
    reg completed_found;
    reg [1:0] completed_slot;
    reg ready_found;
    reg [1:0] oldest_slot;
    reg [31:0] oldest_seq;
    reg [31:0] captured_mem, dropped_mem;
    reg [63:0] stats_snapshot;
    reg stats_toggle, stats_ack;
    (* ASYNC_REG = "TRUE" *) reg [1:0] stats_ack_sync, stats_toggle_sync;
    integer ready_index;
    (* ASYNC_REG = "TRUE" *) reg [1:0] claimed_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] capture_busy_sync, idle_sync;
    wire memory_idle = !capture_busy_sync[1] && !descriptor && ready_slots == 0 &&
        end_seen == end_guard && capture_get == cap_put_sync && write_state == 0 && read_state == 0;
    assign source_idle = idle_sync[1];
    always @* begin
        ready_found = 0; oldest_slot = 0; oldest_seq = 0;
        for (ready_index = 0; ready_index < 3; ready_index = ready_index + 1)
            if (ready_slots[ready_index] && (!ready_found || $signed(slot_seq[ready_index] - oldest_seq) < 0)) begin
                ready_found = 1;
                oldest_slot = ready_index[1:0];
                oldest_seq = slot_seq[ready_index];
            end
    end
    always @* begin
        completed_found = 1;
        if (end_seen[0] != end_guard[0]) completed_slot = 0;
        else if (end_seen[1] != end_guard[1]) completed_slot = 1;
        else if (end_seen[2] != end_guard[2]) completed_slot = 2;
        else begin completed_found = 0; completed_slot = 0; end
    end
    always @(posedge mem_clk or negedge mem_reset_n) begin
        if (!mem_reset_n) begin
            cap_put_meta <= 0; cap_put_sync <= 0;
            end_meta <= 0; end_sync <= 0; end_guard <= 0;
            accept_sync <= 0; done_sync <= 0;
            out_get_meta <= 0; out_get_sync <= 0;
            run_mem <= 0; drop_sync <= 0; claimed_sync <= 0; capture_busy_sync <= 0;
        end else begin
            cap_put_meta <= capture_put; cap_put_sync <= cap_put_meta;
            end_meta <= slot_end; end_sync <= end_meta;
            end_guard <= end_sync;
            accept_sync <= {accept_sync[0], accepted_toggle};
            done_sync <= {done_sync[0], done_toggle};
            out_get_meta <= output_get; out_get_sync <= out_get_meta;
            run_mem <= {run_mem[0], run};
            drop_sync <= {drop_sync[0], drop_event};
            claimed_sync <= {claimed_sync[0], descriptor_claimed};
            capture_busy_sync <= {capture_busy_sync[0], capturing || (|slot_taken)};
        end
    end
    always @(posedge mem_clk or negedge mem_reset_n) begin
        if (!mem_reset_n) begin
            capture_get <= 0; read_bank <= 0; read_word <= 0;
            wr_req <= 0; wr_addr <= 0; wr_data <= 0; write_state <= 0;
        end else case (write_state)
            0: if (cap_put_sync[read_bank] != capture_get[read_bank]) begin
                wr_addr <= capture_addr[read_bank];
                read_word <= 0; wr_req <= 1; write_state <= 1;
            end
            1: begin
                if (wr_data_req) begin
                    wr_data <= capture_ram[{read_bank, read_word}];
                    read_word <= read_word + 1'b1;
                end
                if (wr_finish) begin
                    wr_req <= 0;
                    capture_get[read_bank] <= cap_put_sync[read_bank];
                    read_bank <= read_bank + 1'b1;
                    write_state <= 2;
                end
            end
            default: write_state <= 0;
        endcase
    end
    always @(posedge mem_clk or negedge mem_reset_n) begin
        if (!mem_reset_n) begin
            ready_slots <= 0; slot_release <= 0; end_seen <= 0;
            send_slot <= 0; descriptor <= 0; descriptor_seq <= 0; descriptor_mode <= 0;
            descriptor_epoch<=0; descriptor_config_generation<=0;
            descriptor_native_identity <= 0;
            accept_seen <= 0; done_seen <= 0; reading <= 0;
            read_offset <= 0; output_put <= 0; output_write_bank <= 0;
            output_write_word <= 0; read_state <= 0; rd_req <= 0; rd_addr <= 0;
            captured_mem <= 0; dropped_mem <= 0; drop_seen <= 0;
        end else begin
            if (drop_seen != drop_sync[1]) begin
                dropped_mem <= dropped_mem + 1'b1;
                drop_seen <= drop_sync[1];
            end
            if (completed_found && drop_seen == drop_sync[1] && capture_get == cap_put_sync && write_state == 0) begin
                end_seen[completed_slot] <= end_guard[completed_slot];
                if (slot_bad[completed_slot] || !run_mem[1]) begin
                    slot_release[completed_slot] <= !slot_release[completed_slot];
                    dropped_mem <= dropped_mem + 1'b1;
                end else begin
                    ready_slots[completed_slot] <= 1;
                    captured_mem <= captured_mem + 1'b1;
                end
            end
            if (!descriptor && !claimed_sync[1] && ready_found &&
                (run_mem[1] || (!completed_found && drop_seen == drop_sync[1]))) begin
                if (run_mem[1]) begin
                    send_slot <= oldest_slot;
                    descriptor_seq <= slot_seq[oldest_slot];
                    descriptor_mode <= slot_mode[oldest_slot];
                    descriptor_epoch<=slot_epoch[oldest_slot];
                    descriptor_config_generation<=slot_config_generation[oldest_slot];
                    descriptor_native_identity <= slot_native_identity[oldest_slot];
                    descriptor <= 1;
                    read_offset <= 0;
                end else begin
                    slot_release[oldest_slot] <= !slot_release[oldest_slot];
                    dropped_mem <= dropped_mem + 1'b1;
                end
                ready_slots[oldest_slot] <= 0;
            end
            if (accept_seen != accept_sync[1]) begin
                accept_seen <= accept_sync[1]; reading <= 1;
            end
            if (done_seen != done_sync[1]) begin
                done_seen <= done_sync[1]; descriptor <= 0; reading <= 0;
                slot_release[send_slot] <= !slot_release[send_slot];
            end
            case (read_state)
                0: if (reading && read_offset < FRAME_BEATS && output_put[output_write_bank] == out_get_sync[output_write_bank]) begin
                    rd_addr <= DDR_BASE + {23'd0, send_slot} * FRAME_BEATS + read_offset;
                    rd_req <= 1; output_write_word <= 0; read_state <= 1;
                end
                1: begin
                    if (rd_valid) begin
                        output_ram[{output_write_bank, output_write_word}] <= rd_data;
                        output_write_word <= output_write_word + 1'b1;
                    end
                    if (rd_finish) begin
                        rd_req <= 0; read_state <= 2;
                        output_put[output_write_bank] <= !output_put[output_write_bank];
                        output_write_bank <= output_write_bank + 1'b1;
                        read_offset <= read_offset + 16;
                    end
                end
                default: read_state <= 0;
            endcase
        end
    end
    (* ASYNC_REG = "TRUE" *) reg [1:0] descriptor_sync;
    assign frame_valid = descriptor_sync[1] && !descriptor_claimed;
    assign frame_seq = descriptor_seq;
    assign frame_mode = descriptor_mode;
    assign frame_epoch = descriptor_epoch;
    assign frame_config_generation = descriptor_config_generation;
    assign frame_native_identity = descriptor_native_identity;
    reg data_pending;
    reg [127:0] data_latched;
    wire bank_available = out_put_sync[output_read_bank] != output_get[output_read_bank];
    assign data_valid = data_pending;
    wire [255:0] output_beat = output_ram[{output_read_bank, output_read_word[4:1]}];
    assign data = data_latched;
    always @(posedge mem_clk or negedge mem_reset_n) begin
        if (!mem_reset_n) begin
            stats_snapshot <= 0; stats_toggle <= 0; stats_ack_sync <= 0;
        end else begin
            stats_ack_sync <= {stats_ack_sync[0], stats_ack};
            if (stats_ack_sync[1] == stats_toggle) begin
                stats_snapshot <= {dropped_mem, captured_mem};
                stats_toggle <= !stats_toggle;
            end
        end
    end
    always @(posedge tx_clk or negedge tx_reset_n) begin
        if (!tx_reset_n) begin
            descriptor_sync <= 0; descriptor_claimed <= 0;
            accepted_toggle <= 0; done_toggle <= 0;
            out_put_meta <= 0; out_put_sync <= 0;
            output_get <= 0; output_read_bank <= 0; output_read_word <= 0;
            data_pending <= 0; data_latched <= 0;
            captured <= 0; dropped <= 0; stats_ack <= 0; stats_toggle_sync <= 0;
            idle_sync <= 0;
        end else begin
            descriptor_sync <= {descriptor_sync[0], descriptor};
            idle_sync <= {idle_sync[0], memory_idle};
            out_put_meta <= output_put; out_put_sync <= out_put_meta;
            stats_toggle_sync <= {stats_toggle_sync[0], stats_toggle};
            if (stats_toggle_sync[1] != stats_ack) begin
                {dropped, captured} <= stats_snapshot;
                stats_ack <= stats_toggle_sync[1];
            end
            if (!descriptor_sync[1]) descriptor_claimed <= 0;
            if (frame_valid && frame_accept) begin
                accepted_toggle <= !accepted_toggle;
                descriptor_claimed <= 1;
            end
            if (frame_done) done_toggle <= !done_toggle;
            if (!data_pending || data_ready) begin
                data_pending <= bank_available;
                if (bank_available) begin
                data_latched <= output_read_word[0] ? output_beat[255:128] : output_beat[127:0];
                output_read_word <= output_read_word + 1'b1;
                if (output_read_word == 31) begin
                    output_get[output_read_bank] <= out_put_sync[output_read_bank];
                    output_read_bank <= output_read_bank + 1'b1;
                end
                end
            end
        end
    end
endmodule
