`timescale 1ns/1ps
module video_dma_tx #(
    parameter [31:0] FRAME_BYTES = 32'd4147200,
    parameter integer ENABLE_FEATURE_ID = 0,
    parameter integer ENABLE_NATIVE_ID = 0
)(
    input clk,
    input rst_n,
    input link_ready,
    input [15:0] requester_id,
    input reg_wr,
    input [11:0] reg_addr,
    input [31:0] reg_data,
    input [11:0] read_addr,
    output reg [31:0] read_data,
    output reg run,
    input source_idle,
    input frame_valid,
    input [31:0] frame_seq,
    input [2:0] frame_mode,
    input [31:0] frame_epoch,
    input [31:0] frame_config_generation,
    input [127:0] frame_native_identity,
    output frame_accept,
    output reg frame_done,
    input data_valid,
    input [127:0] data,
    output data_ready,
    output tx_valid,
    output [127:0] tx_data,
    output tx_last,
    input tx_ready,
    input [31:0] captured,
    input [31:0] dropped
);
    localparam [2:0] IDLE = 0, PAYLOAD = 1, METADATA = 2, COMMIT = 3, STOP = 4, DISCARD = 5;
    (* ASYNC_REG = "TRUE" *) reg [2:0] reset_sync;
    wire local_rst_n = reset_sync[2];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) reset_sync <= 0;
        else reset_sync <= {reset_sync[1:0], 1'b1};
    end
    reg [2:0] state;
    reg [63:0] slots [0:3];
    reg [63:0] completion;
    reg [31:0] producer, consumer, error;
    reg [31:0] stop_requested, stop_sent;
    reg [6:0] stop_guard;
    reg [31:0] frame_number;
    reg [2:0] layout;
    reg [31:0] identity_epoch, identity_config_generation;
    reg [127:0] native_identity;
    reg [22:0] byte_offset;
    reg [31:0] captured_meta, captured_sync, dropped_meta, dropped_sync;
    integer slot_index;
    wire [31:0] occupancy = producer - consumer;
    wire configured = completion != 0 && completion[5:0] == 0 &&
        slots[0] != 0 && slots[1] != 0 && slots[2] != 0 && slots[3] != 0 &&
        slots[0][11:0] == 0 && slots[1][11:0] == 0 && slots[2][11:0] == 0 && slots[3][11:0] == 0;
    wire available = run && link_ready && configured && occupancy < 4;
    assign frame_accept = state == IDLE && frame_valid && (available || !run);
    reg [1:0] packet_state;
    reg [63:0] packet_addr;
    reg [9:0] packet_dwords;
    reg [3:0] packet_beat;
    wire packet_last = packet_beat == ((packet_dwords + 3) >> 2) - 1;
    wire packet_done = packet_state == 2 && tx_ready && tx_valid && packet_last;
    wire [31:0] header = 32'h60000000 | {22'd0, packet_dwords};
    reg [127:0] payload;
    function [127:0] wire_order;
        input [127:0] host_data;
        integer lane;
        begin
            for (lane = 0; lane < 4; lane = lane + 1)
                wire_order[lane*32 +: 32] = {host_data[lane*32 +: 8], host_data[lane*32+8 +: 8], host_data[lane*32+16 +: 8], host_data[lane*32+24 +: 8]};
        end
    endfunction
    always @* begin
        case (state)
            PAYLOAD: payload = data;
            METADATA: payload = ENABLE_NATIVE_ID && packet_beat==2 ? native_identity :
                (ENABLE_FEATURE_ID || ENABLE_NATIVE_ID) && packet_beat==1 ?
                {29'd0,layout,32'h31444946,identity_config_generation,identity_epoch} :
                {32'd1, FRAME_BYTES, 29'd0, layout, frame_number};
            COMMIT: payload = {96'd0, producer + 32'd1};
            STOP: payload = {96'd0, stop_requested};
            default: payload = 0;
        endcase
    end
    assign tx_valid = packet_state == 1 || (packet_state == 2 && (state != PAYLOAD || data_valid));
    assign tx_last = packet_state == 2 && packet_last;
    assign tx_data = packet_state == 1 ? {packet_addr[31:0], packet_addr[63:32], requester_id, 8'd0, (packet_dwords == 1 ? 8'h0f : 8'hff), header} : wire_order(payload);
    assign data_ready = (state == PAYLOAD && packet_state == 2 && tx_ready) || state == DISCARD;
    always @* begin
        read_data = 0;
        case (read_addr)
            12'h100: read_data = 32'h56444d41;
            12'h101: read_data = 32'h00020001;
            12'h102: read_data = {31'd0, run};
            12'h103: read_data = {28'd0, link_ready, configured, state != IDLE, !run && state == IDLE && stop_requested == stop_sent};
            12'h104: read_data = consumer;
            12'h105: read_data = producer;
            12'h106: read_data = FRAME_BYTES;
            12'h107: read_data = error;
            12'h108: read_data = stop_requested;
            12'h109: read_data = stop_sent;
            12'h10a: read_data = captured_sync;
            12'h10b: read_data = dropped_sync;
            12'h110: read_data = slots[0][31:0];
            12'h111: read_data = slots[0][63:32];
            12'h112: read_data = slots[1][31:0];
            12'h113: read_data = slots[1][63:32];
            12'h114: read_data = slots[2][31:0];
            12'h115: read_data = slots[2][63:32];
            12'h116: read_data = slots[3][31:0];
            12'h117: read_data = slots[3][63:32];
            12'h118: read_data = completion[31:0];
            12'h119: read_data = completion[63:32];
            default: read_data = 0;
        endcase
    end
    always @(posedge clk or negedge local_rst_n) begin
        if (!local_rst_n) begin
            run <= 0; producer <= 0; consumer <= 0; error <= 0;
            stop_requested <= 0; stop_sent <= 0; completion <= 0;
            stop_guard <= 0;
            captured_meta <= 0; captured_sync <= 0; dropped_meta <= 0; dropped_sync <= 0;
            for (slot_index = 0; slot_index < 4; slot_index = slot_index + 1) slots[slot_index] <= 0;
        end else begin
            captured_meta <= captured; captured_sync <= captured_meta;
            dropped_meta <= dropped; dropped_sync <= dropped_meta;
            if (run || stop_requested == stop_sent) stop_guard <= 0;
            else if (!stop_guard[6]) stop_guard <= stop_guard + 1'b1;
            if (packet_done && state == COMMIT) producer <= producer + 1'b1;
            if (packet_done && state == STOP) stop_sent <= stop_requested;
            if (reg_wr) begin
                if (reg_addr == 12'h102) begin
                    if (!reg_data[0]) run <= 0;
                    else if (configured && link_ready && stop_requested == stop_sent) run <= 1;
                    else error <= error | 1;
                end else if (reg_addr == 12'h104) begin
                    if (reg_data - consumer <= occupancy) consumer <= reg_data;
                    else error <= error | 2;
                end else if (reg_addr == 12'h108) begin
                    if (!run && stop_requested == stop_sent) stop_requested <= reg_data;
                    else error <= error | 4;
                end else if (!run && state == IDLE && stop_requested == stop_sent) begin
                    case (reg_addr)
                        12'h110: slots[0][31:0] <= reg_data;
                        12'h111: slots[0][63:32] <= reg_data;
                        12'h112: slots[1][31:0] <= reg_data;
                        12'h113: slots[1][63:32] <= reg_data;
                        12'h114: slots[2][31:0] <= reg_data;
                        12'h115: slots[2][63:32] <= reg_data;
                        12'h116: slots[3][31:0] <= reg_data;
                        12'h117: slots[3][63:32] <= reg_data;
                        12'h118: completion[31:0] <= reg_data;
                        12'h119: completion[63:32] <= reg_data;
                        12'h107: error <= 0;
                        default: begin end
                    endcase
                end
            end
        end
    end
    always @(posedge clk or negedge local_rst_n) begin
        if (!local_rst_n) begin
            state <= IDLE; packet_state <= 0; packet_addr <= 0;
            packet_dwords <= 0; packet_beat <= 0; byte_offset <= 0;
            frame_number <= 0; layout <= 0; frame_done <= 0;
            identity_epoch<=0; identity_config_generation<=0;
            native_identity <= 0;
        end else begin
            frame_done <= 0;
            if (state == IDLE) begin
                if (!run && stop_guard[6] && source_idle && !frame_valid && stop_requested != stop_sent && configured && link_ready) state <= STOP;
                else if (frame_accept) begin
                    frame_number <= frame_seq; layout <= frame_mode; byte_offset <= 0;
                    identity_epoch<=frame_epoch; identity_config_generation<=frame_config_generation;
                    native_identity <= frame_native_identity;
                    state <= available ? PAYLOAD : DISCARD;
                end
            end else if (state == DISCARD) begin
                if (data_valid) begin
                    byte_offset <= byte_offset + 16;
                    if (byte_offset == FRAME_BYTES - 16) begin state <= IDLE; frame_done <= 1; end
                end
            end else case (packet_state)
                0: if (link_ready && (state != PAYLOAD || data_valid)) begin
                    packet_state <= 1; packet_beat <= 0;
                    case (state)
                        PAYLOAD: begin packet_addr <= slots[producer[1:0]] + {41'd0, byte_offset}; packet_dwords <= 32; end
                        METADATA: begin packet_addr <= completion + {56'd0, producer[1:0], 6'b0} + 16; packet_dwords <= ENABLE_NATIVE_ID ? 12 : ENABLE_FEATURE_ID ? 8 : 4; end
                        COMMIT: begin packet_addr <= completion + {56'd0, producer[1:0], 6'b0}; packet_dwords <= 1; end
                        STOP: begin packet_addr <= completion + 256; packet_dwords <= 1; end
                        default: begin end
                    endcase
                end
                1: if (tx_ready) packet_state <= 2;
                2: if (tx_ready && tx_valid) begin
                    packet_beat <= packet_beat + 1'b1;
                    if (packet_last) begin
                        packet_state <= 0;
                        case (state)
                            PAYLOAD: begin
                                byte_offset <= byte_offset + 128;
                                if (byte_offset == FRAME_BYTES - 128) state <= METADATA;
                            end
                            METADATA: state <= COMMIT;
                            COMMIT: begin state <= IDLE; frame_done <= 1; end
                            STOP: state <= IDLE;
                            default: state <= IDLE;
                        endcase
                    end
                end
                default: packet_state <= 0;
            endcase
        end
    end
endmodule
