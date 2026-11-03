`timescale 1ns / 1ps

// 五按键状态机。key_level_n已经过同步和消抖，低电平表示按下。
// 短按在释放时产生，长按在达到阈值时产生；长按不会再产生短按。
module key_state_ctrl #(
    parameter integer LONG_PRESS_CYCLES = 148500000,
    parameter integer LONG_COUNT_BITS = 28
)(
    input             clk,
    input             rst_n,
    input      [4:0]  key_level_n,
    input             timing_vs,
    input             timing_de,
    output reg [2:0]  source_mode,
    output reg [2:0]  process_mode,
    output reg        drive_state,
    output reg [1:0]  turn_request,
    output reg        alarm_reset,
    output reg [31:0] event_seq,
    output reg [31:0] alarm_reset_seq
);
    localparam [1:0] TURN_NONE  = 2'd0;

    reg [4:0] held;
    reg [4:0] long_sent;
    reg [LONG_COUNT_BITS-1:0] hold_count [0:4];
    reg [5:0] blank_count;
    reg [2:0] requested_source;
    reg [2:0] requested_process;
    integer i;
    integer event_count;
    integer k;
    reg [4:0] long_event;
    wire frame_boundary = timing_vs && !timing_de && blank_count == 6'd31;
    wire key0_short = held[0] && key_level_n[0] && !long_sent[0];
    wire key4_short = held[4] && key_level_n[4] && !long_sent[4];
    wire [2:0] next_source = key0_short ?
        ((requested_source == 3'd4) ? 3'd0 : requested_source + 1'b1) :
        requested_source;

    // 统计 FPGA 负责的输入源切换、显示模式切换和报警复位事件。
    always @* begin
        for (k = 0; k < 5; k = k + 1)
            long_event[k] = held[k] && !long_sent[k] && !key_level_n[k] &&
                            hold_count[k] >= LONG_PRESS_CYCLES - 2;
        event_count = 0;
        if (key0_short) event_count = event_count + 1;
        if (key4_short) event_count = event_count + 1;
        if (long_event[0]) event_count = event_count + 1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            held <= 5'b0;
            long_sent <= 5'b0;
            blank_count <= 0;
            requested_source <= 3'd0;
            requested_process <= 3'd0;
            source_mode <= 3'd0;
            process_mode <= 3'd0;
            // 驾驶意图由 RK 管理，这些兼容输出保持无效。
            drive_state <= 1'b0;
            turn_request <= TURN_NONE;
            alarm_reset <= 1'b0;
            event_seq <= 32'd0;
            alarm_reset_seq <= 32'd0;
            for (i = 0; i < 5; i = i + 1)
                hold_count[i] <= {LONG_COUNT_BITS{1'b0}};
        end else begin
            if (!timing_vs || timing_de) blank_count <= 0;
            else if (!(&blank_count)) blank_count <= blank_count + 1'b1;
            alarm_reset <= 1'b0;
            if (key0_short)
                requested_source <= next_source;

            // 记录按下、长按和释放事件。长按阈值由参数决定，便于仿真缩短。
            for (i = 0; i < 5; i = i + 1) begin
                if (!key_level_n[i]) begin
                    if (!held[i]) begin
                        held[i] <= 1'b1;
                        long_sent[i] <= 1'b0;
                        hold_count[i] <= {LONG_COUNT_BITS{1'b0}};
                    end else if (!long_sent[i]) begin
                        if (long_event[i]) begin
                            long_sent[i] <= 1'b1;
                        end else begin
                            hold_count[i] <= hold_count[i] + 1'b1;
                        end
                    end
                end else if (held[i]) begin
                    held[i] <= 1'b0;
                    hold_count[i] <= {LONG_COUNT_BITS{1'b0}};
                    long_sent[i] <= 1'b0;
                end
            end

            if (key4_short) begin
                requested_process <= (requested_process == 3'd4) ? 3'd0 : requested_process + 1'b1;
            end
            event_seq <= event_seq + event_count;

            // KEY0长按：只清报警脉冲，不改变输入源。
            if (long_event[0]) begin
                alarm_reset <= 1'b1;
                alarm_reset_seq <= alarm_reset_seq + 1'b1;
            end
            // 输入源和处理模式只在场边界提交，避免一帧中途改变布局。
            if (frame_boundary) begin
                source_mode <= next_source;
                process_mode <= requested_process;
            end
        end
    end
endmodule
