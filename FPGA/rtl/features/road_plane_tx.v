`timescale 1ns/1ps
// 将完整载荷传到旧单窗口或轮转的 RGB 主机帧槽。
// 每个 TLP 在发出包头前完整缓存；
// 全部 PLANE_BYTES 字节发完后再发送元数据，
// 最后用 COMMIT 向四项记录环发布 producer+1。
// 记录地址为 completion + producer[1:0]*64，各 32 位字含义如下：
// 字 0 为提交值，字 4 为元数据版本，字 5 为字节数，字 6 为类型，字 7 为源布局；
// 字 8 为完整源帧序号，字 9 为 epoch，字 10 为源配置版本。
// 提交值必须最后写入。
// 提交或拒绝帧后，均由 frame_done 释放 DDR 帧槽。
module road_plane_tx #(
    parameter [31:0] PLANE_BYTES = 32'd230400,   // 载荷含 14400 个流数据拍、1800 个 TLP
    parameter [31:0] KIND        = 32'd2,
    parameter [31:0] METADATA_VERSION = 32'd2,
    parameter [2:0]  LAYOUT      = 3'd1,         // 1 = GRAY8 单平面
    parameter integer RGB_METADATA = 0,
    parameter [31:0] HOST_SLOT_BYTES = 0
)(
    input               clk,
    input               rst_n,
    input               link_ready,
    input  [15:0]       requester_id,

    // 配置（来自 road_plane_ctl）
    input               cfg_run,
    input  [24:0]       cfg_base_beat,     // DDR 起始 beat
    input  [15:0]       cfg_slot_beats,    // 每槽 beat 数（7200）
    input  [1:0]        cfg_slot_count,
    input  [63:0]      cfg_host_addr,     // RK 侧平面缓冲区总线地址（驱动分配后写入）

    // 帧提交（来自 road_plane_store_960 的 frame_done 脉冲）
    input               frame_valid,
    input  [31:0]       frame_seq,
    input  [31:0]       frame_epoch,
    input  [31:0]       frame_config,
    input  [2:0]        frame_layout,
    output              frame_accept,
    output reg          frame_done,

    // DDR 读出流（由读仲裁器提供）
    input               data_valid,
    input               data_error,
    input  [127:0]      data,
    output              data_ready,

    // TLP 发送流
    output              tx_valid,
    output [127:0]      tx_data,
    output              tx_last,
    input               tx_ready,

    // 状态
    output reg [31:0]   producer,
    output reg [31:0]   dropped_frames,
    input  [63:0]       completion,        // 完成记录基址（字节）
    input  [31:0]       consumer,          // 主机已消费序号（来自 road_plane_ctl 0x138）
    output reg          busy
);
    localparam [2:0] IDLE = 0, PAYLOAD = 1, METADATA = 2, COMMIT = 3;

    // 复位同步
    (* ASYNC_REG = "TRUE" *) reg [2:0] reset_sync;
    wire local_rst_n = reset_sync[2];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) reset_sync <= 3'b0;
        else        reset_sync <= {reset_sync[1:0], 1'b1};
    end

    reg [2:0]  state;
    reg [31:0] frame_number, frame_source_epoch, frame_source_config;
    reg [2:0] frame_source_layout;
    reg [22:0] byte_offset;         // 0 .. 230400-128
    reg [127:0] packet_buffer[0:7];
    reg [3:0] buffered;
    reg [63:0] host_hold, completion_hold;
    reg [1:0] host_slot, frame_host_slot;

    // TLP 组包
    reg [1:0]   packet_state;
    reg [63:0]  packet_addr;
    reg [9:0]   packet_dwords;
    reg [3:0]   packet_beat;
    wire        packet_last = packet_beat == ((packet_dwords + 3) >> 2) - 1;
    wire        packet_done = packet_state == 2 && tx_ready && tx_valid && packet_last;
    wire [31:0] header      = 32'h60000000 | {22'd0, packet_dwords};

    function [127:0] wire_order;
        input [127:0] host_data;
        integer lane;
        begin
            for (lane = 0; lane < 4; lane = lane + 1)
                wire_order[lane*32 +: 32] = {
                    host_data[lane*32 +: 8], host_data[lane*32+8 +: 8],
                    host_data[lane*32+16 +: 8], host_data[lane*32+24 +: 8]};
        end
    endfunction

    reg [127:0] payload;
    always @* begin
        case (state)
            PAYLOAD:  payload = packet_buffer[packet_beat[2:0]];
            // 旧格式使用字 4..11，RGB 使用字 4..15，均放在一条记录内。
            METADATA: case (packet_beat)
                0: payload = {{29'd0, frame_source_layout}, KIND, PLANE_BYTES, METADATA_VERSION};
                1: payload = {RGB_METADATA ? 32'd1 : 32'd0,
                              frame_source_config, frame_source_epoch, frame_number};
                default: payload = {{30'd0,frame_host_slot},32'd2,32'd1920,32'h01680280};
            endcase
            COMMIT:   payload = {96'd0, producer + 32'd1};
            default:  payload = 128'd0;
        endcase
    end

    // 主机反压时保留邮箱中的帧，不计为丢帧。
    wire [31:0] inflight = producer - consumer;
    wire        room     = (inflight < {30'd0, cfg_slot_count});

    assign frame_accept = state == IDLE && frame_valid && cfg_run && link_ready && room &&
                          cfg_host_addr!=0 && completion!=0;
    assign tx_valid     = packet_state == 1 ||
                          packet_state == 2;
    assign tx_last      = packet_state == 2 && packet_last;
    assign tx_data      = (packet_state == 1)
                        ? {packet_addr[31:0], packet_addr[63:32], requester_id,
                           8'd0, (packet_dwords == 1 ? 8'h0f : 8'hff), header}
                        : wire_order(payload);
    assign data_ready   = state == PAYLOAD && packet_state == 0 && buffered < 8;

    always @(posedge clk or negedge local_rst_n) begin
        if (!local_rst_n) begin
            state <= IDLE; packet_state <= 0; packet_addr <= 0;
            packet_dwords <= 0; packet_beat <= 0; byte_offset <= 0;
            frame_number <= 0; frame_source_epoch <= 0; frame_source_config <= 0;
            frame_source_layout <= LAYOUT; buffered <= 0; host_hold <= 0; completion_hold <= 0;
            frame_done <= 0; producer <= 0; dropped_frames <= 0; busy <= 0;
            host_slot<=0; frame_host_slot<=0;
        end else begin
            frame_done <= 1'b0;
            busy <= (state != IDLE);
            if(state==IDLE && packet_state!=0) begin
                packet_state<=0; buffered<=0;
            end else if (state == IDLE) begin
                if (frame_accept) begin
                    frame_number <= frame_seq;
                    frame_source_epoch <= frame_epoch;
                    frame_source_config <= frame_config;
                    frame_source_layout <= frame_layout;
                    byte_offset  <= 0;
                    buffered<=0;
                    host_hold<=cfg_host_addr+host_slot*HOST_SLOT_BYTES;
                    frame_host_slot<=host_slot; completion_hold<=completion;
                    state        <= PAYLOAD;
                end
            end else case (packet_state)
                0: if(state==PAYLOAD && buffered<8) begin
                    if(data_valid && data_ready) begin
                        if(data_error) begin
                            // 尚未发出包头，可丢弃不完整包并拒绝该帧，
                            // 不发送元数据或提交值。
                            buffered<=0; state<=IDLE; frame_done<=1;
                            dropped_frames<=dropped_frames+1'b1;
                        end else begin
                            packet_buffer[buffered[2:0]]<=data;
                            buffered<=buffered+1'b1;
                        end
                    end
                end else if (link_ready) begin
                    packet_state <= 1;
                    packet_beat  <= 0;
                    case (state)
                        PAYLOAD: begin
                            packet_addr   <= host_hold + {41'd0, byte_offset};
                            packet_dwords <= 32;
                        end
                        METADATA: begin
                            packet_addr   <= completion_hold + {56'd0, producer[1:0], 6'b0} + 16;
                            packet_dwords <= RGB_METADATA ? 12 : 8;
                        end
                        COMMIT: begin
                            packet_addr   <= completion_hold + {56'd0, producer[1:0], 6'b0};
                            packet_dwords <= 1;
                        end
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
                                buffered <= 0;
                                byte_offset <= byte_offset + 23'd128;
                                if (byte_offset == PLANE_BYTES - 128) state <= METADATA;
                            end
                            METADATA: state <= COMMIT;
                            COMMIT: begin
                                state <= IDLE;
                                producer <= producer + 1'b1;
                                if(HOST_SLOT_BYTES!=0)
                                    host_slot<=host_slot==cfg_slot_count-1'b1 ? 0 : host_slot+1'b1;
                                frame_done <= 1'b1;
                            end
                            default: state <= IDLE;
                        endcase
                    end
                end
                default: packet_state <= 0;
            endcase
        end
    end
endmodule
