`timescale 1ns/1ps
// 三路突发仲裁：base 为基础通道，video 为完整视频，plane 为平面/小图支路。
// base/video 轮流优先；plane 通常只占空闲时隙，连续等待达到阈值后强制服务。
// plane_req 恒为零时，base/video 的逐拍选择须与原两路版本一致。
module video_dma_burst_mux #(
    parameter integer PLANE_STARVE_LIMIT = 8
)(
    input clk,
    input rst_n,
    input base_req,
    input [9:0] base_len,
    input [24:0] base_addr,
    input [255:0] base_data,
    output base_data_req,
    output base_finish,
    input video_req,
    input [24:0] video_addr,
    input [255:0] video_data,
    output video_data_req,
    output video_finish,
    // 第三路突发接口，实际用途由顶层接线决定。
    input plane_req,
    input [9:0] plane_len,
    input [24:0] plane_addr,
    input [255:0] plane_data,
    output plane_data_req,
    output plane_finish,
    output req,
    output [9:0] len,
    output [24:0] addr,
    output [255:0] data,
    input data_req,
    input finish
);
    localparam integer SEL_BASE = 0, SEL_VIDEO = 1, SEL_PLANE = 2;
    localparam [7:0] STARVE_LIM = PLANE_STARVE_LIMIT;

    reg [1:0] state;
    reg [1:0] selected;
    reg prefer_video;
    reg [9:0] held_len;
    reg [24:0] held_addr;
    reg [7:0] plane_starve;

    assign req  = state == 1;
    assign len  = held_len;
    assign addr = held_addr;
    assign data = (selected == SEL_VIDEO) ? video_data :
                  (selected == SEL_PLANE) ? plane_data : base_data;

    // 主控实际完成前保持事务所有权；
    // 本地超时不能取消 AXI，否则迟到数据会被送给其他通道。
    // 超时诊断由 AXI 主控的粘滞故障标志提供。
    assign base_data_req  = state == 1 && selected == SEL_BASE  && data_req;
    assign video_data_req = state == 1 && selected == SEL_VIDEO && data_req;
    assign plane_data_req = state == 1 && selected == SEL_PLANE && data_req;

    assign base_finish  = state == 1 && selected == SEL_BASE  && finish;
    assign video_finish = state == 1 && selected == SEL_VIDEO && finish;
    assign plane_finish = state == 1 && selected == SEL_PLANE && finish;

    // 选择组合逻辑：保持旧两路语义，plane 只在空闲或饿死超限时插入
    wire legacy_video = video_req && (!base_req || prefer_video);
    wire legacy_idle  = !base_req && !video_req;
    wire plane_forced = plane_req && (plane_starve >= STARVE_LIM);
    wire take_plane   = plane_req && (legacy_idle || plane_forced);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 0;
            selected <= SEL_BASE;
            prefer_video <= 0;
            held_len <= 0;
            held_addr <= 0;
            // plane_starve 由下方专用 always 复位/维护，此处不得重复赋值
        end else case (state)
            0: if (base_req || video_req || plane_req) begin
                if (take_plane) begin
                    selected <= SEL_PLANE;
                    held_len <= plane_len;
                    held_addr <= plane_addr;
                end else begin
                    selected <= legacy_video ? SEL_VIDEO : SEL_BASE;
                    held_len <= legacy_video ? 10'd16 : base_len;
                    held_addr <= legacy_video ? video_addr : base_addr;
                end
                state <= 1;
            end
            1: if (finish) begin
                prefer_video <= (selected == SEL_VIDEO) ? 1'b0 : 1'b1;
                state <= 2;
            end
            2: if (!(selected == SEL_VIDEO ? video_req :
                     selected == SEL_PLANE ? plane_req : base_req)) state <= 0;
            default: state <= 0;
        endcase
    end

    // 饿死计数：只要 plane 在等且本次没被服务，就 +1；被服务则清零
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) plane_starve <= 0;
        else if (state == 1 && finish) begin
            if (selected == SEL_PLANE) plane_starve <= 0;
            else if (plane_req && !plane_starve[7]) plane_starve <= plane_starve + 1'b1;
        end
    end
endmodule
