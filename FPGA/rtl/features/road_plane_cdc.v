`timescale 1ns/1ps
// 按整帧使能和抽帧；HDMI 边界可能与最后一个降采样像素重合，
// 因此延迟边界，等待该像素离开源流水线。
module road_plane_capture_gate (
    input clk, input rst_n, input enable, input [3:0] period_div,
    input boundary, input pixel_valid,
    input [31:0] source_sequence, input [31:0] source_epoch,
    input [31:0] source_config, input [2:0] source_layout,
    output valid, output sof,
    output reg [31:0] frame_sequence, output reg [31:0] frame_epoch,
    output reg [31:0] frame_config, output reg [2:0] frame_layout
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] enable_sync;
    (* ASYNC_REG = "TRUE" *) reg [3:0] period_s1, period_s2;
    reg [2:0] boundary_d;
    reg first, capture;
    reg [3:0] skipped;
    reg [31:0] boundary_sequence, boundary_epoch, boundary_config;
    reg [2:0] boundary_layout;
    wire take_frame = enable_sync[1] && skipped == 0;
    assign valid = pixel_valid && (first ? take_frame : capture);
    assign sof = valid && first;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable_sync<=0; period_s1<=1; period_s2<=1;
            boundary_d<=0; first<=0; capture<=0; skipped<=0;
            boundary_sequence<=0; boundary_epoch<=0; boundary_config<=0; boundary_layout<=0;
            frame_sequence<=0; frame_epoch<=0; frame_config<=0; frame_layout<=0;
        end else begin
            enable_sync <= {enable_sync[0],enable};
            period_s1<=period_div; period_s2<=period_s1;
            boundary_d<={boundary_d[1:0],boundary};
            // source_sequence 属于本边界开始的帧，与帧存储记录的自增前序号一致。
            // 延迟边界时保持此序号，
            // 使帧标识与像素流水线对齐。
            if (boundary) begin
                boundary_sequence<=source_sequence;
                boundary_epoch<=source_epoch;
                boundary_config<=source_config;
                boundary_layout<=source_layout;
            end
            if (boundary_d[2]) begin first<=1; capture<=0; end
            else if (first && pixel_valid) begin
                first<=0; capture<=take_frame;
                if (take_frame) begin
                    frame_sequence<=boundary_sequence;
                    frame_epoch<=boundary_epoch;
                    frame_config<=boundary_config;
                    frame_layout<=boundary_layout;
                end
                if (!enable_sync[1]) skipped<=0;
                else if (skipped!=0) skipped<=skipped-1'b1;
                else skipped <= (period_s2<=1) ? 0 : period_s2-1'b1;
            end
        end
    end
endmodule

module road_plane_counter_cdc (
    input rst_n, input src_clk, input dst_clk,
    input [31:0] count_in, output reg [31:0] count_out
);
    reg [31:0] gray;
    (* ASYNC_REG = "TRUE" *) reg [31:0] gs1,gs2;
    integer k;
    reg [31:0] decoded;
    always @* begin
        decoded[31]=gs2[31];
        for(k=30;k>=0;k=k-1) decoded[k]=decoded[k+1]^gs2[k];
    end
    always @(posedge src_clk or negedge rst_n)
        if(!rst_n) gray<=0; else gray<=count_in^(count_in>>1);
    always @(posedge dst_clk or negedge rst_n) begin
        if(!rst_n) begin gs1<=0; gs2<=0; count_out<=0; end
        else begin gs1<=gray; gs2<=gs1; count_out<=decoded; end
    end
endmodule

// 单项邮箱传递已完成 DDR 帧槽；发送完成或拒绝该帧后才释放，
// 期间描述信息保持不变。
// rst_n 复位存储侧所有权；单独复位 PCIe 只清空流 FIFO，
// DDR 帧槽保持占用，直到读取端排空。
module road_plane_cdc #(
    parameter integer FIFO_AW=6, LENGTH_BITS=16, METADATA_BITS=1,
    parameter integer ALLOW_DISCARD=0, BRAM_FIFO=0
)(
    input rst_n, input tx_rst_n, input mem_clk, input tx_clk,
    input frame_done, input [1:0] slot_index, input [31:0] frame_seq, input [31:0] frame_epoch,
    input [31:0] frame_config, input [2:0] frame_layout,
    input [24:0] cfg_base, input [LENGTH_BITS-1:0] cfg_slot_beats,
    input [METADATA_BITS-1:0] frame_metadata,
    output [METADATA_BITS-1:0] tx_metadata,
    input discard_frame,
    output reg slot_locked, output reg [1:0] locked_slot, output reg [31:0] skipped_frames,
    output reg reader_start, output reg [24:0] reader_base,
    input [127:0] reader_data, input reader_valid, input reader_error, input reader_busy,
    output reader_ready,
    output frame_valid, output [31:0] tx_frame_seq, output [31:0] tx_frame_epoch,
    output [31:0] tx_frame_config, output [2:0] tx_frame_layout,
    input frame_accept, input tx_frame_done,
    output [127:0] data, output data_valid, output data_error, input data_ready
);
    reg [31:0] seq_hold, epoch_hold, config_hold;
    reg [METADATA_BITS-1:0] metadata_hold;
    reg [2:0] layout_hold;
    reg offer_toggle, accept_toggle, release_toggle;
    (* ASYNC_REG = "TRUE" *) reg [1:0] offer_sync, accept_sync, release_sync;
    reg offer_seen, accept_seen, release_seen, pending, start_pending;
    reg tx_active, release_wait;
    (* ASYNC_REG = "TRUE" *) reg [2:0] mem_reset, tx_reset;
    (* ASYNC_REG = "TRUE" *) reg [2:0] fifo_mem_reset, fifo_tx_reset;
    (* ASYNC_REG = "TRUE" *) reg [1:0] tx_alive_sync;
    wire mem_rst_n=mem_reset[2], tx_local_rst_n=tx_reset[2];
    wire tx_async_rst_n=rst_n && tx_rst_n;
    wire fifo_async_rst_n=mem_rst_n && tx_local_rst_n;
    wire fifo_mem_rst_n=fifo_mem_reset[2], fifo_tx_rst_n=fifo_tx_reset[2];
    always @(posedge mem_clk or negedge rst_n)
        if(!rst_n) mem_reset<=0; else mem_reset<={mem_reset[1:0],1'b1};
    always @(posedge tx_clk or negedge tx_async_rst_n)
        if(!tx_async_rst_n) tx_reset<=0; else tx_reset<={tx_reset[1:0],1'b1};
    always @(posedge mem_clk or negedge fifo_async_rst_n)
        if(!fifo_async_rst_n) fifo_mem_reset<=0; else fifo_mem_reset<={fifo_mem_reset[1:0],1'b1};
    always @(posedge tx_clk or negedge fifo_async_rst_n)
        if(!fifo_async_rst_n) fifo_tx_reset<=0; else fifo_tx_reset<={fifo_tx_reset[1:0],1'b1};

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if(!mem_rst_n) begin
            slot_locked<=0; locked_slot<=0; reader_base<=0; seq_hold<=0; epoch_hold<=0; config_hold<=0; layout_hold<=0;
            skipped_frames<=0; metadata_hold<=0;
            offer_toggle<=0; reader_start<=0; accept_sync<=0; release_sync<=0;
            accept_seen<=0; release_seen<=0; start_pending<=0; release_wait<=0; tx_alive_sync<=0;
        end else begin
            tx_alive_sync<={tx_alive_sync[0],tx_local_rst_n};
            accept_sync<={accept_sync[0],accept_toggle};
            release_sync<={release_sync[0],release_toggle};
            reader_start<=0;
            if(!tx_alive_sync[1]) begin
                offer_toggle<=0;
                accept_seen<=accept_sync[1]; release_seen<=release_sync[1];
                start_pending<=0;
                if(slot_locked) release_wait<=1;
            end else if(frame_done && !slot_locked && !release_wait) begin
                locked_slot<=slot_index; seq_hold<=frame_seq; epoch_hold<=frame_epoch;
                config_hold<=frame_config; layout_hold<=frame_layout;
                metadata_hold<=frame_metadata;
                reader_base<=cfg_base+slot_index*cfg_slot_beats;
                slot_locked<=1; offer_toggle<=~offer_toggle;
            end
            else if(frame_done) begin
                skipped_frames<=skipped_frames+1'b1;
            end
            if(tx_alive_sync[1] && accept_sync[1]!=accept_seen) begin
                accept_seen<=accept_sync[1]; start_pending<=1;
            end
            // PCIe 复位后 DDR 读取端可能仍在排空旧突发；
            // 忙时不发出新启动脉冲，避免请求丢失。
            if(tx_alive_sync[1] && start_pending && !release_wait && !reader_busy) begin reader_start<=1; start_pending<=0; end
            if(tx_alive_sync[1] && release_sync[1]!=release_seen) begin
                release_seen<=release_sync[1]; release_wait<=1; start_pending<=0;
            end
            if(release_wait && !reader_busy) begin slot_locked<=0; release_wait<=0; end
        end
    end
    always @(posedge tx_clk or negedge tx_local_rst_n) begin
        if(!tx_local_rst_n) begin
            offer_sync<=0; offer_seen<=0; pending<=0;
            accept_toggle<=0; release_toggle<=0; tx_active<=0;
        end else begin
            offer_sync<={offer_sync[0],offer_toggle};
            if(offer_sync[1]!=offer_seen) begin
                offer_seen<=offer_sync[1]; pending<=1;
            end else if(ALLOW_DISCARD!=0 && discard_frame && pending) begin
                pending<=0; release_toggle<=~release_toggle;
            end else if(frame_accept) begin pending<=0; tx_active<=1; end
            if(frame_accept) accept_toggle<=~accept_toggle;
            if(tx_frame_done && tx_active) begin
                release_toggle<=~release_toggle; tx_active<=0;
            end
        end
    end
    assign frame_valid=pending;
    // seq_hold 在 offer 跨域前冻结，直到 release 才释放，
    // 覆盖 frame_accept 前任意时长的主机反压。
    assign tx_frame_seq=seq_hold;
    assign tx_frame_epoch=epoch_hold;
    assign tx_frame_config=config_hold;
    assign tx_frame_layout=layout_hold;
    assign tx_metadata=metadata_hold;

    // 首字直通双时钟 FIFO 仅同步格雷码指针；数据和结束错误标记
    // 保持到读握手完成。
    generate if(BRAM_FIFO!=0) begin: block_fifo
        wire ready;
        video_token_fifo_bram #(.WIDTH(129),.ADDR_BITS(FIFO_AW)) u_data (
            .wr_clk(mem_clk),.wr_rst_n(fifo_mem_rst_n),.wr_valid(reader_valid && tx_alive_sync[1]),
            .wr_ready(ready),.wr_data({reader_error,reader_data}),
            .rd_clk(tx_clk),.rd_rst_n(fifo_tx_rst_n),.rd_valid(data_valid),
            .rd_ready(data_ready),.rd_data({data_error,data})
        );
        assign reader_ready=ready && tx_alive_sync[1];
    end else begin: legacy_fifo
    reg [128:0] fifo_mem [0:(1<<FIFO_AW)-1];
    reg [FIFO_AW:0] wbin,wgray,rbin,rgray;
    (* ASYNC_REG = "TRUE" *) reg [FIFO_AW:0] rs1,rs2,ws1,ws2;
    wire [FIFO_AW:0] wnext=wbin+1'b1, rnext=rbin+1'b1;
    wire [FIFO_AW:0] wgnext=(wnext>>1)^wnext, rgnext=(rnext>>1)^rnext;
    wire full=wgnext=={~rs2[FIFO_AW:FIFO_AW-1],rs2[FIFO_AW-2:0]};
    wire empty=rgray==ws2;
    assign reader_ready=!full && fifo_mem_rst_n && tx_alive_sync[1];
    assign data_valid=!empty && fifo_tx_rst_n;
    assign {data_error,data}=fifo_mem[rbin[FIFO_AW-1:0]];
    always @(posedge mem_clk or negedge fifo_mem_rst_n) begin
        if(!fifo_mem_rst_n) begin wbin<=0; wgray<=0; rs1<=0; rs2<=0; end
        else begin
            rs1<=rgray; rs2<=rs1;
            if(reader_valid && reader_ready) begin
                fifo_mem[wbin[FIFO_AW-1:0]]<={reader_error,reader_data};
                wbin<=wnext; wgray<=wgnext;
            end
        end
    end
    always @(posedge tx_clk or negedge fifo_tx_rst_n) begin
        if(!fifo_tx_rst_n) begin rbin<=0; rgray<=0; ws1<=0; ws2<=0; end
        else begin
            ws1<=wgray; ws2<=ws1;
            if(data_valid && data_ready) begin rbin<=rnext; rgray<=rgnext; end
        end
    end
    end endgenerate
endmodule
