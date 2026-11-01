`timescale 1ns/1ps
// 地址以 32 字节为单位；收到有效 END 且所有 DDR 写入完成后才发布帧槽。
// 超时只使帧失效，不能提前释放尚未完成的事务。
module hdmi_native_writer #(
    parameter integer FRAME_BEATS = 129600,
    parameter [24:0] BASE = 25'd2097152,
    parameter integer SOURCE_TIMEOUT = 12500000
) (
    input wire clk, rst_n,
    input wire token_valid, output reg token_ready,
    input wire [314:0] token,
    input wire read_owned, input wire [1:0] read_slot,
    output reg wr_req, output wire [9:0] wr_len,
    output reg [24:0] wr_addr, output reg [255:0] wr_data,
    input wire wr_data_req, wr_finish, wr_fault,
    output reg latest_valid, source_live, frame_committed,
    output reg [1:0] latest_slot,
    output reg [31:0] latest_sequence, committed_frames, rejected_frames,
    output reg [31:0] latest_epoch, output reg [63:0] latest_timestamp,
    output reg [31:0] latest_left_crc
);
    localparam SEEK=0, FILL=1, SEND=2, WAIT_END=3;
    reg [1:0] state, slot;
    reg [255:0] burst [0:15];
    reg [4:0] count, sent;
    reg [24:0] done_beats;
    reg [31:0] frame_sequence, age;
    reg burst_bad;
    reg [1:0] free_slot;
    reg [1:0] slot_hint, candidate_slot;
    reg free_found;
    integer i;
    wire [1:0] kind = token[314:313];
    wire [31:0] token_sequence = token[312:281];
    wire [24:0] token_offset = token[280:256];
    wire first_word = kind == 1 && token_offset == 0;
    wire word_matches = token_sequence == frame_sequence && token_offset == done_beats + count;
    assign wr_len = 10'd16;
    always @* begin
        free_slot = slot_hint; free_found = 0; candidate_slot = 0;
        for (i=3; i>=0; i=i-1) begin
            candidate_slot = slot_hint + i;
            if (!(latest_valid && latest_slot == candidate_slot) &&
                !(read_owned && read_slot == candidate_slot)) begin
                free_slot = candidate_slot; free_found = 1;
            end
        end
        token_ready = 0;
        case (state)
            SEEK: token_ready = !first_word;
            FILL: token_ready = count < 16 &&
                (!first_word || token_sequence == frame_sequence);
            WAIT_END: token_ready = !first_word;
            default: token_ready = 0;
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= SEEK; slot <= 0; slot_hint <= 0; count <= 0; sent <= 0;
            done_beats <= 0; frame_sequence <= 0; age <= 0; burst_bad <= 0;
            wr_req <= 0; wr_addr <= 0; wr_data <= 0;
            latest_valid <= 0; source_live <= 0; frame_committed <= 0;
            latest_slot <= 0; latest_sequence <= 0;
            committed_frames <= 0; rejected_frames <= 0;
            latest_epoch<=0;latest_timestamp<=0;
            latest_left_crc<=0;
        end else begin
            frame_committed <= 0;
            if (age < SOURCE_TIMEOUT) age <= age + 1'b1;
            else source_live <= 0;
            if (wr_fault) source_live <= 0;
            case (state)
                SEEK: if (token_valid && first_word && free_found) begin
                    slot <= free_slot; frame_sequence <= token_sequence;
                    slot_hint <= free_slot + 1'b1;
                    done_beats <= 0; count <= 0; state <= FILL;
                end
                FILL: begin
                    if (count == 16) begin
                        wr_addr <= BASE + slot*FRAME_BEATS + done_beats;
                        wr_req <= 1; sent <= 0; burst_bad <= wr_fault; state <= SEND;
                    end else if (token_valid) begin
                        if (kind != 1 || !word_matches || done_beats + count >= FRAME_BEATS) begin
                            rejected_frames <= rejected_frames + 1'b1;
                            source_live <= 0; state <= SEEK; count <= 0;
                        end else if (token_ready) begin
                            burst[count[3:0]] <= token[255:0]; count <= count + 1'b1;
                        end
                    end
                end
                SEND: begin
                    if (wr_fault) burst_bad <= 1;
                    if (wr_data_req) begin
                        wr_req <= 0;
                        if (sent < 16) begin
                            wr_data <= burst[sent[3:0]]; sent <= sent + 1'b1;
                        end else burst_bad <= 1;
                    end
                    if (wr_finish) begin
                        wr_req <= 0; count <= 0;
                        if (burst_bad || wr_fault || (wr_data_req && sent >= 16) ||
                            !(sent == 16 || (sent == 15 && wr_data_req))) begin
                            rejected_frames <= rejected_frames + 1'b1;
                            source_live <= 0; state <= SEEK;
                        end else begin
                            done_beats <= done_beats + 16;
                            state <= done_beats + 16 == FRAME_BEATS ? WAIT_END : FILL;
                        end
                    end
                end
                WAIT_END: if (token_valid) begin
                    if (kind == 2 && token_sequence == frame_sequence &&
                        token_offset == FRAME_BEATS && token[0] && !wr_fault) begin
                        latest_valid <= 1; latest_slot <= slot; latest_sequence <= frame_sequence;
                        latest_epoch<=token[32:1];latest_timestamp<=token[96:33];
                        latest_left_crc<=token[128:97];
                        committed_frames <= committed_frames + 1'b1;
                        frame_committed <= 1; source_live <= 1; age <= 0;
                    end else begin
                        rejected_frames <= rejected_frames + 1'b1; source_live <= 0;
                    end
                    state <= SEEK;
                end
                default: state <= SEEK;
            endcase
        end
    end
endmodule
