`timescale 1ns/1ps
// 三个帧槽轮转，禁止覆盖最新发布帧或正在读取的帧。
// 总线停顿时继续占有事务，直到写入实际完成。
module rgb_small_writer #(
    parameter integer FRAME_BEATS = 21600,
    parameter [24:0] BASE = 25'd2621440
) (
    input clk, rst_n, session_alive,
    input token_valid, output reg token_ready, input [381:0] token,
    input slot_locked, input [1:0] locked_slot,
    output reg wr_req, output [9:0] wr_len,
    output reg [24:0] wr_addr, output reg [255:0] wr_data,
    input wr_data_req, wr_finish, wr_fault,
    output reg frame_done, output reg [1:0] completed_slot,
    output reg [31:0] completed_sequence, completed_epoch, completed_config,
    output reg [2:0] completed_layout,
    output reg [31:0] committed_frames, rejected_frames,
    output busy
);
    localparam SEEK=0, FILL=1, SEND=2, WAIT_END=3;
    reg [1:0] state, slot, hint;
    reg latest_valid;
    reg [255:0] burst[0:15];
    reg [4:0] count, sent;
    reg [24:0] done_beats;
    reg [98:0] identity;
    reg burst_bad, cancelled;
    reg [1:0] free_slot;
    reg free_found;
    integer i, candidate;
    wire [1:0] kind=token[381:380];
    wire [98:0] token_identity=token[379:281];
    wire [24:0] token_offset=token[280:256];
    wire first_word=kind==1 && token_offset==0;
    assign wr_len=16;
    assign busy=state!=SEEK;
    always @* begin
        free_slot=0; free_found=0; candidate=0;
        for(i=2;i>=0;i=i-1) begin
            candidate=hint+i;
            if(candidate>=3) candidate=candidate-3;
            if(!(latest_valid && completed_slot==candidate) &&
               !(slot_locked && locked_slot==candidate)) begin
                free_slot=candidate; free_found=1;
            end
        end
        token_ready=0;
        case(state)
            SEEK: token_ready=!first_word;
            FILL: token_ready=count<16 && (!first_word || token_identity==identity);
            WAIT_END: token_ready=!first_word;
            default: token_ready=0;
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=SEEK; slot<=0; hint<=0; latest_valid<=0; count<=0; sent<=0;
            done_beats<=0; identity<=0; burst_bad<=0; cancelled<=0; wr_req<=0; wr_addr<=0; wr_data<=0;
            frame_done<=0; completed_slot<=0; completed_sequence<=0; completed_epoch<=0;
            completed_config<=0; completed_layout<=0; committed_frames<=0; rejected_frames<=0;
        end else begin
            frame_done<=0;
            if(!session_alive) cancelled<=1;
            case(state)
                SEEK: if(token_valid && first_word && free_found && session_alive) begin
                    slot<=free_slot; hint<=free_slot==2 ? 0 : free_slot+1'b1;
                    identity<=token_identity; done_beats<=0; count<=0; cancelled<=0; state<=FILL;
                end
                FILL: if(count==16) begin
                    wr_addr<=BASE+slot*FRAME_BEATS+done_beats;
                    wr_req<=1; sent<=0; burst_bad<=wr_fault; state<=SEND;
                end else if(token_valid) begin
                    if(kind!=1 || token_identity!=identity ||
                       token_offset!=done_beats+count || done_beats+count>=FRAME_BEATS) begin
                        rejected_frames<=rejected_frames+1'b1; count<=0; state<=SEEK;
                    end else if(token_ready) begin
                        burst[count[3:0]]<=token[255:0]; count<=count+1'b1;
                    end
                end
                SEND: begin
                    if(wr_fault) burst_bad<=1;
                    if(wr_data_req) begin
                        wr_req<=0;
                        if(sent<16) begin wr_data<=burst[sent[3:0]]; sent<=sent+1'b1; end
                        else burst_bad<=1;
                    end
                    if(wr_finish) begin
                        wr_req<=0; count<=0;
                        if(burst_bad || wr_fault || (wr_data_req && sent>=16) ||
                           !(sent==16 || (sent==15 && wr_data_req))) begin
                            rejected_frames<=rejected_frames+1'b1; state<=SEEK;
                        end else begin
                            done_beats<=done_beats+16;
                            state<=done_beats+16==FRAME_BEATS ? WAIT_END : FILL;
                        end
                    end
                end
                WAIT_END: if(token_valid) begin
                    if(kind==2 && token_identity==identity &&
                       token_offset==FRAME_BEATS && token[0] && !wr_fault && !cancelled && session_alive) begin
                        latest_valid<=1; completed_slot<=slot;
                        {completed_sequence,completed_epoch,completed_config,completed_layout}<=identity;
                        committed_frames<=committed_frames+1'b1; frame_done<=1;
                    end else rejected_frames<=rejected_frames+1'b1;
                    state<=SEEK;
                end
                default: state<=SEEK;
            endcase
        end
    end
endmodule
