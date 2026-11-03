`timescale 1ns/1ps
// 以邮箱传递最新状态；短脉冲先累计为事件或报警计数，避免跨域漏采。
// 共用复位同时终止握手两端。
module stage1_status_cdc (
    input src_clk, input dst_clk, input rst_n,
    input [127:0] source_state,
    output reg [127:0] state_snapshot,
    output reg [31:0] generation
);
    reg [127:0] held_state;
    reg request, acknowledge;
    (* async_reg = "true" *) reg [1:0] src_reset_pipe, dst_reset_pipe;
    wire src_reset_n = src_reset_pipe[1];
    wire dst_reset_n = dst_reset_pipe[1];
    (* async_reg = "true" *) reg ack_meta, ack_sync;
    (* async_reg = "true" *) reg req_meta, req_sync;
    reg initial_snapshot;
    always @(posedge src_clk or negedge rst_n) begin
        if (!rst_n) src_reset_pipe<=0;
        else src_reset_pipe<={src_reset_pipe[0],1'b1};
    end
    always @(posedge dst_clk or negedge rst_n) begin
        if (!rst_n) dst_reset_pipe<=0;
        else dst_reset_pipe<={dst_reset_pipe[0],1'b1};
    end
    always @(posedge src_clk or negedge src_reset_n) begin
        if (!src_reset_n) begin
            held_state<=0; request<=0; ack_meta<=0; ack_sync<=0; initial_snapshot<=1;
        end else begin
            ack_meta<=acknowledge; ack_sync<=ack_meta;
            if (request == ack_sync && (initial_snapshot || held_state != source_state)) begin
                held_state<=source_state;
                request<=~request;
                initial_snapshot<=0;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_reset_n) begin
        if (!dst_reset_n) begin
            req_meta<=0; req_sync<=0; acknowledge<=0;
            state_snapshot<=0; generation<=0;
        end else begin
            req_meta<=request; req_sync<=req_meta;
            // held_state 已保持稳定至少两个目标时钟周期。
            if (req_sync != acknowledge) begin
                state_snapshot<=held_state;
                generation<=generation + 1'b1;
                acknowledge<=req_sync;
            end
        end
    end
endmodule
