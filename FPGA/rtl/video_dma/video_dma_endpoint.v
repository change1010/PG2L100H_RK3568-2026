
// 原文件名：pango_pcie_dma_top.v

`timescale 1ns/1ps
module video_dma_endpoint #(
    parameter integer ENABLE_DARK = 0,
    parameter integer ENABLE_PLANE = 0,
    parameter integer ENABLE_ROAD_REF = 0
)
(
    // 时钟与复位
    input                       ref_clk_n       ,      // 参考时钟 100 MHz
    input                       ref_clk_p       ,      // 参考时钟 100 MHz
    input                       button_rst_n    ,
    input                       perst_n         ,
    // 串口接口
    output  wire                txd             ,
    input                       rxd             ,
    // PCIe 差分信号
    input           [3:0]       rxn             ,
    input           [3:0]       rxp             ,
    output  wire    [3:0]       txn             ,
    output  wire    [3:0]       txp             ,

    input [127:0] control_snapshot,
    input [31:0] control_generation,
    input video_reset_n,
    output video_clk,
    output video_core_reset_n,
    output video_run,
    input video_source_idle,
    input video_frame_valid,
    input [31:0] video_frame_seq,
    input [2:0] video_frame_mode,
    input [31:0] video_frame_epoch,
    input [31:0] video_frame_config_generation,
    input [127:0] video_frame_native_identity,
    output feature_reg_wr,
    output [31:0] feature_reg_addr,
    output [31:0] feature_reg_data,
    output feature_read_en,
    output [31:0] feature_read_addr,
    input [127:0] feature_read_data,
    output video_frame_accept,
    output video_frame_done,
    input video_data_valid,
    input [127:0] video_data,
    output video_data_ready,
    input [31:0] video_captured,
    input [31:0] video_dropped,
    // 平面及小图帧通过低优先级 MWR 接入发送通道。
    input                       plane_run,
    input                       plane_frame_valid,
    input  [31:0]               plane_frame_seq,
    input  [31:0]               plane_frame_epoch,
    input  [31:0]               plane_frame_config,
    input  [2:0]                plane_frame_layout,
    output                      plane_frame_accept,
    output                      plane_frame_done,
    input                       plane_data_valid,
    input                       plane_data_error,
    input  [127:0]              plane_data,
    output                      plane_data_ready,
    input  [24:0]               plane_cfg_base,
    input  [15:0]               plane_cfg_beats,
    input  [1:0]                plane_cfg_slots,
    input  [63:0]               plane_host_addr,
    input  [63:0]               plane_completion,
    input  [31:0]               plane_consumer,
    // 平面 TX 的两个计数（pclk_div2 域）。producer 是"已整帧提交到宿主内存"的
    // 计数，与完成记录环的 commit(=producer+1) 同源；寄存器 0x129 必须用它，
    // 不能再用 road_plane_store_960 的 DDR 写帧数（后者比提交超前，会使
    // ACQUIRE 取到尚未写回的槽）。
    output [31:0]               plane_producer,
    output [31:0]               plane_dropped_frames,
    input rgb_run, rgb_frame_valid,
    input [31:0] rgb_frame_seq, rgb_frame_epoch, rgb_frame_config,
    input [2:0] rgb_frame_layout,
    output rgb_frame_accept, rgb_frame_done,
    input rgb_data_valid, rgb_data_error,
    input [127:0] rgb_data, output rgb_data_ready,
    input [63:0] rgb_host_addr, rgb_completion,
    input [31:0] rgb_consumer,
    output [31:0] rgb_producer, rgb_dropped,
    output rgb_busy,
    input ref_run,ref_frame_valid,ref_source_idle,
    input [287:0] ref_frame_metadata,
    output ref_frame_accept,ref_frame_done,
    input ref_data_valid,ref_data_error,
    input [127:0] ref_data,output ref_data_ready,
    input [191:0] ref_host_addrs,input [63:0] ref_completion,
    input [31:0] ref_consumer,ref_session,ref_stop_cookie,
    output [31:0] ref_producer,ref_dropped,ref_stop_sent,
    output ref_busy,
    // 指示灯
    output reg                  ref_led         ,
    output reg                  pclk_led        ,
    output wire                 smlh_link_up    /* synthesis PAP_MARK_DEBUG=”true”*/,
    output wire                 rdlh_link_up    /* synthesis PAP_MARK_DEBUG=”true”*/
);


localparam      ADDR_WIDTH    = 'd32    ;  
localparam      RD_DATA_DELAY = 'd1     ;   

// 复位滤波及同步
wire            sync_button_rst_n       ;
wire            sync_perst_n            ;
wire            ref_core_rst_n          ;
wire            s_pclk_rstn             ;

// 内部信号
// 时钟与复位
wire            pclk_div2               /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            pclk                    ;
wire            ref_clk                 ;
wire            core_rst_n              ;
// AXIS 主接口
wire            axis_master_tvalid      /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_master_tready      /* synthesis PAP_MARK_DEBUG=”true”*/;
wire    [127:0] axis_master_tdata       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire    [3:0]   axis_master_tkeep       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_master_tlast       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire    [7:0]   axis_master_tuser       /* synthesis PAP_MARK_DEBUG=”true”*/;

// AXIS 从接口 0
wire            axis_slave0_tready      /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave0_tvalid      /* synthesis PAP_MARK_DEBUG=”true”*/;
wire    [127:0] axis_slave0_tdata       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave0_tlast       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave0_tuser       /* synthesis PAP_MARK_DEBUG=”true”*/;

// AXIS 从接口 1
wire            axis_slave1_tready      /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave1_tvalid      /* synthesis PAP_MARK_DEBUG=”true”*/;
wire    [127:0] axis_slave1_tdata       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave1_tlast       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave1_tuser       /* synthesis PAP_MARK_DEBUG=”true”*/;

// AXIS 从接口 2
wire            axis_slave2_tready      /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave2_tvalid      /* synthesis PAP_MARK_DEBUG=”true”*/;
wire    [127:0] axis_slave2_tdata       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave2_tlast       /* synthesis PAP_MARK_DEBUG=”true”*/;
wire            axis_slave2_tuser       /* synthesis PAP_MARK_DEBUG=”true”*/;

wire    [7:0]   cfg_pbus_num            ;
wire    [4:0]   cfg_pbus_dev_num        ;
wire    [2:0]   cfg_max_rd_req_size     ;
wire    [2:0]   cfg_max_payload_size    ;
wire            cfg_rcb                 ;

// 系统状态
wire    [4:0]   smlh_ltssm_state        /* synthesis PAP_MARK_DEBUG=”true”*/;

// 链路状态指示
// 100 MHz 参考时钟下，ref_led 约每 1.34 秒翻转。
reg     [26:0]  ref_led_cnt             ;
reg     [26:0]  pclk_led_cnt            ;

// 32 位串口转 APB 接口
wire            uart_p_sel              ;
wire    [3:0]   uart_p_strb             ;
wire    [15:0]  uart_p_addr             ;
wire    [31:0]  uart_p_wdata            ;
wire            uart_p_ce               ;
wire            uart_p_we               ;
wire            uart_p_rdy              ;
wire    [31:0]  uart_p_rdata            ;

// 复位滤波

hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_buttonrstn_debounce(
    .clk                (ref_clk            ),
    .rstn_in            (button_rst_n       ),
    .rstn_out           (sync_button_rst_n  )
);

hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_perstn_debounce(
    .clk                (ref_clk            ),
    .rstn_in            (perst_n            ),
    .rstn_out           (sync_perst_n       )
);

hsst_rst_sync_v1_0  u_ref_core_rstn_sync    (
    .clk                (ref_clk            ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (ref_core_rst_n     )
);

hsst_rst_sync_v1_0  u_pclk_core_rstn_sync   (
    .clk                (pclk               ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (s_pclk_rstn        )
);

// 时钟心跳指示
always @(posedge ref_clk or negedge sync_perst_n)
begin
    if (!sync_perst_n)
        ref_led_cnt    <= 27'd0;
    else
        ref_led_cnt    <= ref_led_cnt + 27'd1;
end

always @(posedge ref_clk or negedge sync_perst_n)
begin
    if (!sync_perst_n)
        ref_led        <= 1'b1;
    else if(&ref_led_cnt)
        ref_led        <= ~ref_led;
end

always @(posedge pclk or negedge s_pclk_rstn)
begin
    if (!s_pclk_rstn)
        pclk_led_cnt    <= 27'd0;
    else
        pclk_led_cnt    <= pclk_led_cnt + 27'd1;
end

always @(posedge pclk or negedge s_pclk_rstn)
begin
    if (!s_pclk_rstn)
        pclk_led        <= 1'b1;
    else if(&pclk_led_cnt)
        pclk_led        <= ~pclk_led;
end

// DMA 接口

wire                        cpld_finish        ; 
wire                        mwr_req_cmd        ;
wire                        mwr_tx_start       ;	
wire    [31:0]              rd_base_ram_addr   ;	
wire    [31:0]              wr_base_ram_addr   ;
wire    [10:0]              mem_length         ;

wire                        pio_wr_en          ;
wire    [15:0]              pio_wr_byte_en     ; 
wire    [ADDR_WIDTH-1:0]    pio_wr_addr        ;
wire    [127:0]             pio_wr_data        ;
wire                        pio_rd_en          ;
wire    [ADDR_WIDTH-1:0]    pio_rd_addr        ;
wire    [127:0]             pio_rd_data        ;
wire [127:0] pio_ram_data;
wire [31:0] pio_read_offset, pio_write_offset;
wire [31:0] video_register_data;
reg [31:0] video_read_data;
reg video_read_selected;
wire legacy_valid, legacy_ready, legacy_last, legacy_user;
wire [127:0] legacy_data;
wire stream_valid, stream_ready, stream_last;
wire [127:0] stream_data;
wire bus_master_enabled;
wire plane_valid, plane_last, plane_ready;
wire [127:0] plane_tx_data;

generate if (ENABLE_PLANE != 0) begin : g_plane_tx
road_plane_tx #(
    .PLANE_BYTES(32'd460800), .KIND(32'd3), .METADATA_VERSION(32'd3)
) u_road_plane_tx (
    .clk(pclk_div2), .rst_n(core_rst_n && video_reset_n),
    .link_ready(smlh_link_up && rdlh_link_up && bus_master_enabled),
    .requester_id({cfg_pbus_num, cfg_pbus_dev_num, 3'b0}),
    .cfg_run(plane_run),
    .cfg_base_beat(plane_cfg_base),
    .cfg_slot_beats(plane_cfg_beats),
    .cfg_slot_count(plane_cfg_slots),
    .cfg_host_addr(plane_host_addr),
    .frame_valid(plane_frame_valid), .frame_seq(plane_frame_seq),
    .frame_epoch(plane_frame_epoch), .frame_config(plane_frame_config), .frame_layout(plane_frame_layout),
    .frame_accept(plane_frame_accept), .frame_done(plane_frame_done),
    .data_valid(plane_data_valid), .data_error(plane_data_error), .data(plane_data), .data_ready(plane_data_ready),
    .tx_valid(plane_valid), .tx_data(plane_tx_data), .tx_last(plane_last), .tx_ready(plane_ready),
    .producer(plane_producer), .dropped_frames(plane_dropped_frames), .completion(plane_completion),
    .consumer(plane_consumer), .busy()
);

end else begin : g_no_plane_tx
    assign plane_valid=1'b0;
    assign plane_last=1'b0;
    assign plane_tx_data=128'd0;
    assign plane_frame_accept=1'b0;
    assign plane_frame_done=1'b0;
    assign plane_data_ready=1'b0;
    assign plane_producer=32'd0;
    assign plane_dropped_frames=32'd0;
end endgenerate

wire rgb_valid, rgb_last, rgb_ready;
wire [127:0] rgb_tx_data;
road_plane_tx #(
    .PLANE_BYTES(32'd691200),.KIND(32'd4),.METADATA_VERSION(32'd4),
    .RGB_METADATA(1),.HOST_SLOT_BYTES(32'd692224)
) u_rgb_small_tx (
    .clk(pclk_div2),.rst_n(core_rst_n && video_reset_n),
    .link_ready(smlh_link_up && rdlh_link_up && bus_master_enabled),
    .requester_id({cfg_pbus_num,cfg_pbus_dev_num,3'b0}),.cfg_run(rgb_run),
    .cfg_base_beat(25'd2621440),.cfg_slot_beats(16'd21600),.cfg_slot_count(2'd3),
    .cfg_host_addr(rgb_host_addr),.completion(rgb_completion),.consumer(rgb_consumer),
    .frame_valid(rgb_frame_valid),.frame_seq(rgb_frame_seq),.frame_epoch(rgb_frame_epoch),
    .frame_config(rgb_frame_config),.frame_layout(rgb_frame_layout),
    .frame_accept(rgb_frame_accept),.frame_done(rgb_frame_done),
    .data_valid(rgb_data_valid),.data_error(rgb_data_error),.data(rgb_data),.data_ready(rgb_data_ready),
    .tx_valid(rgb_valid),.tx_data(rgb_tx_data),.tx_last(rgb_last),.tx_ready(rgb_ready),
    .producer(rgb_producer),.dropped_frames(rgb_dropped),.busy(rgb_busy)
);
wire old_valid,old_last,old_user,old_ready,ref_valid,ref_last,ref_ready;
wire [127:0] old_data,ref_tx_data;
generate if (ENABLE_ROAD_REF != 0) begin : g_reference_tx
road_ref_tx u_road_ref_tx (
    .clk(pclk_div2),.rst_n(core_rst_n && video_reset_n),
    .link_ready(smlh_link_up && rdlh_link_up && bus_master_enabled),
    .requester_id({cfg_pbus_num,cfg_pbus_dev_num,3'b0}),.cfg_run(ref_run),
    .host_addrs(ref_host_addrs),.completion(ref_completion),
    .consumer(ref_consumer),.session(ref_session),.stop_cookie(ref_stop_cookie),.source_idle(ref_source_idle),
    .frame_valid(ref_frame_valid),.frame_metadata(ref_frame_metadata),
    .frame_accept(ref_frame_accept),.frame_done(ref_frame_done),
    .data_valid(ref_data_valid),.data_error(ref_data_error),.data(ref_data),.data_ready(ref_data_ready),
    .tx_valid(ref_valid),.tx_data(ref_tx_data),.tx_last(ref_last),.tx_ready(ref_ready),
    .producer(ref_producer),.dropped_frames(ref_dropped),.stop_sent(ref_stop_sent),.busy(ref_busy)
);
end else begin : g_no_reference_tx
    assign ref_valid=1'b0;
    assign ref_last=1'b0;
    assign ref_tx_data=128'd0;
    assign ref_frame_accept=1'b0;
    assign ref_frame_done=1'b0;
    assign ref_data_ready=1'b0;
    assign ref_producer=32'd0;
    assign ref_dropped=32'd0;
    assign ref_stop_sent=32'd0;
    assign ref_busy=1'b0;
end endgenerate
tx_packet_mux4 u_tx_frame_mux (
    .clk(pclk_div2),
    .rst_n(core_rst_n),
    .s_valid(stream_valid), .s_data(stream_data), .s_last(stream_last), .s_user(1'b0), .s_ready(stream_ready),
    .l_valid(legacy_valid), .l_data(legacy_data), .l_last(legacy_last), .l_user(legacy_user), .l_ready(legacy_ready),
    .p_valid(plane_valid),  .p_data(plane_tx_data), .p_last(plane_last), .p_user(1'b0),  .p_ready(plane_ready),
    .r_valid(rgb_valid), .r_data(rgb_tx_data), .r_last(rgb_last), .r_user(1'b0), .r_ready(rgb_ready),
    .m_valid(old_valid), .m_data(old_data), .m_last(old_last),
    .m_user(old_user), .m_ready(old_ready)
);
tx_packet_mux4 u_tx_reference_mux (
    .clk(pclk_div2),.rst_n(core_rst_n),
    .s_valid(old_valid),.s_data(old_data),.s_last(old_last),.s_user(old_user),.s_ready(old_ready),
    .l_valid(ref_valid),.l_data(ref_tx_data),.l_last(ref_last),.l_user(1'b0),.l_ready(ref_ready),
    .p_valid(1'b0),.p_data(128'd0),.p_last(1'b0),.p_user(1'b0),.p_ready(),
    .r_valid(1'b0),.r_data(128'd0),.r_last(1'b0),.r_user(1'b0),.r_ready(),
    .m_valid(axis_slave2_tvalid),.m_data(axis_slave2_tdata),.m_last(axis_slave2_tlast),
    .m_user(axis_slave2_tuser),.m_ready(axis_slave2_tready)
);
assign video_clk = pclk_div2;
assign video_core_reset_n = core_rst_n;
wire [127:0] legacy_pio_data=video_read_selected ? {96'd0,video_read_data} : pio_ram_data;
assign feature_reg_addr=pio_write_offset;
assign feature_read_addr=pio_read_offset;
feature_pio_mux #(.ENABLE_DARK(ENABLE_DARK)) u_feature_pio (
    .clk(pclk_div2),.rst_n(core_rst_n),.read_en(pio_rd_en),.read_addr(pio_rd_addr),
    .write_en(pio_wr_en),.write_byte_en(pio_wr_byte_en),.write_addr(pio_wr_addr),.write_data(pio_wr_data),
    .legacy_data(legacy_pio_data),.feature_data(feature_read_data),.feature_read_en(feature_read_en),
    .feature_write_en(feature_reg_wr),.feature_write_data(feature_reg_data),.result_data(pio_rd_data),
    .read_offset(pio_read_offset),.write_offset(pio_write_offset)
);
always @(posedge pclk_div2 or negedge core_rst_n) begin
    if (!core_rst_n) begin
        video_read_selected <= 0;
        video_read_data <= 0;
    end else begin
        if (pio_rd_en) begin
            video_read_selected <= pio_read_offset>=32'h100 && pio_read_offset<=32'h126;
            // 独立只读状态寄存器，保留视频 DMA 寄存器布局。
            case (pio_read_offset)
                9'h120: video_read_data <= 32'h53543143;
                9'h121: video_read_data <= 32'd1;
                9'h122: video_read_data <= control_generation;
                9'h123: video_read_data <= control_snapshot[31:0];
                9'h124: video_read_data <= control_snapshot[63:32];
                9'h125: video_read_data <= control_snapshot[95:64];
                9'h126: video_read_data <= control_snapshot[127:96];
                default: video_read_data <= video_register_data;
            endcase
        end
    end
end
    video_dma_tx #(.ENABLE_FEATURE_ID(1),.ENABLE_NATIVE_ID(1)) u_video_tx (
    .clk(pclk_div2), .rst_n(core_rst_n && video_reset_n),
    .link_ready(smlh_link_up && rdlh_link_up && bus_master_enabled),
    .requester_id({cfg_pbus_num, cfg_pbus_dev_num, 3'b0}),
    .reg_wr(pio_wr_en && (&pio_wr_byte_en[3:0]) && pio_write_offset>=32'h100 && pio_write_offset<=32'h119),
    .reg_addr(pio_write_offset[11:0]), .reg_data(pio_wr_data[31:0]),
    .read_addr(pio_read_offset[11:0]), .read_data(video_register_data),
    .run(video_run), .source_idle(video_source_idle), .frame_valid(video_frame_valid),
    .frame_seq(video_frame_seq), .frame_mode(video_frame_mode),
    .frame_epoch(video_frame_epoch),.frame_config_generation(video_frame_config_generation),
    .frame_native_identity(video_frame_native_identity),
    .frame_accept(video_frame_accept), .frame_done(video_frame_done),
    .data_valid(video_data_valid), .data(video_data), .data_ready(video_data_ready),
    .tx_valid(stream_valid), .tx_data(stream_data), .tx_last(stream_last), .tx_ready(stream_ready),
    .captured(video_captured), .dropped(video_dropped)
);

wire                        dma_wr_en          ;
wire    [15:0]              dma_wr_byte_en     ; 
wire    [ADDR_WIDTH-1:0]    dma_wr_addr        ;
wire    [127:0]             dma_wr_data        ;
wire                        dma_rd_en          ;
wire    [ADDR_WIDTH-1:0]    dma_rd_addr        ;
wire    [127:0]             dma_rd_data        ;
	
rt_pcie_dma #(
    .ADDR_WIDTH             (ADDR_WIDTH             ),
	.RD_DATA_DELAY          (RD_DATA_DELAY          )   // 读数据返回延迟支持 1、2 或 3 个时钟周期。
)
u_rt_pcie_dma
(
    .clk                    (pclk_div2              ),  // 用户时钟：Gen1 为 62.5 MHz，Gen2 为 125 MHz。
    .rst_n                  (core_rst_n             ),
    // 通道编号
    .i_cfg_pbus_num         (cfg_pbus_num           ),  
    .i_cfg_pbus_dev_num     (cfg_pbus_dev_num       ),  
    .i_cfg_max_rd_req_size  (cfg_max_rd_req_size    ),  
    .i_cfg_max_payload_size (cfg_max_payload_size   ),  
    
    // AXIS 主接口
    .i_axis_master_tvld     (axis_master_tvalid     ),  
    .o_axis_master_trdy     (axis_master_tready     ),  
    .i_axis_master_tdata    (axis_master_tdata      ),  
    .i_axis_master_tkeep    (axis_master_tkeep      ),  
    .i_axis_master_tlast    (axis_master_tlast      ),  
    .i_axis_master_tuser    (axis_master_tuser      ),  

    // AXIS 从接口 0
    .i_axis_slave0_trdy     (axis_slave0_tready     ),  
    .o_axis_slave0_tvld     (axis_slave0_tvalid     ),  
    .o_axis_slave0_tdata    (axis_slave0_tdata      ),  
    .o_axis_slave0_tlast    (axis_slave0_tlast      ),  
    .o_axis_slave0_tuser    (axis_slave0_tuser      ),  
    // AXIS 从接口 1
    .i_axis_slave1_trdy     (axis_slave1_tready     ),  
    .o_axis_slave1_tvld     (axis_slave1_tvalid     ),  
    .o_axis_slave1_tdata    (axis_slave1_tdata      ),  
    .o_axis_slave1_tlast    (axis_slave1_tlast      ),  
    .o_axis_slave1_tuser    (axis_slave1_tuser      ),  
    // AXIS 从接口 2
    .i_axis_slave2_trdy     (legacy_ready     ),  
    .o_axis_slave2_tvld     (legacy_valid     ),  
    .o_axis_slave2_tdata    (legacy_data      ),  
    .o_axis_slave2_tlast    (legacy_last      ),  
    .o_axis_slave2_tuser    (legacy_user      ),  
    
	
	.i_cpld_finish           (cpld_finish           ),  
    .o_mwr_req_cmd           (mwr_req_cmd           ),  
    .i_mwr_tx_start          (mwr_tx_start          ),  
    .o_rd_base_ram_addr      (rd_base_ram_addr      ),  
    .o_wr_base_ram_addr      (wr_base_ram_addr      ),  
    .o_mem_length            (mem_length            ),  
    
    // RAM 接口
	// PIO 写 RAM
	.o_pio_wr_en             (pio_wr_en             ),  
    .o_pio_wr_byte_en        (pio_wr_byte_en        ),  
    .o_pio_wr_addr           (pio_wr_addr           ),  
    .o_pio_wr_data           (pio_wr_data           ),  
	// PIO 读 RAM
	.o_pio_rd_en             (pio_rd_en             ),  
    .o_pio_rd_addr           (pio_rd_addr           ),  
    .i_pio_rd_data           (pio_rd_data           ),	
	
    // DMA 读完成包写入 RAM
	.o_dma_wr_en             (dma_wr_en             ),  
    .o_dma_wr_byte_en        (dma_wr_byte_en        ),  
    .o_dma_wr_addr           (dma_wr_addr           ),  
    .o_dma_wr_data           (dma_wr_data           ),  
	// DMA 写请求从 RAM 读取载荷
	.o_dma_rd_en             (dma_rd_en             ),  
    .o_dma_rd_addr           (dma_rd_addr           ),  
    .i_dma_rd_data           (dma_rd_data           )   
);

// PIO 数据 RAM
ram_128in_128out u_ram_128in_128out
(
    .wr_clk             (pclk_div2               ),  
    .wr_rst             (~core_rst_n             ),  
    .wr_en              (pio_wr_en               ),  
    .wr_byte_en         (pio_wr_byte_en          ),  
    .wr_addr            (pio_wr_addr[11:0]       ),  
    .wr_data            (pio_wr_data             ),  
    .rd_clk             (pclk_div2               ),  
    .rd_rst             (~core_rst_n             ),  
    .rd_clk_en          (pio_rd_en               ),  
    .rd_addr            (pio_rd_addr[11:0]       ),  
    .rd_data            (pio_ram_data             )   
);

// DMA 数据 RAM
wire            dma_ram_wr_en      ;
wire    [31:0]  dma_ram_wr_addr    ;
wire    [15:0]  dma_ram_wr_data    ;
wire    [1:0]   dma_ram_wr_byte_en ; 

wire            dma_ram_rd_en      ;
wire    [31:0]  dma_ram_rd_addr    ;
wire    [15:0]  dma_ram_rd_data    ;

rt_gs_dma_interface_conversion #(
    .ADDR_WIDTH              (ADDR_WIDTH          ),
	.RD_DATA_DELAY           (RD_DATA_DELAY       )   // 读数据返回延迟支持 1、2 或 3 个时钟周期。
)
u_rt_gs_dma_interface_conversion
(
    .clk                     (pclk_div2           ),  
    .rst_n                   (core_rst_n          ),  
	// 128 位写入
    .i_dma_wr_en             (dma_wr_en           ),  
    .i_dma_wr_byte_en        (dma_wr_byte_en      ),  
    .i_dma_wr_addr           (dma_wr_addr         ),  
    .i_dma_wr_data           (dma_wr_data         ),  
	// 16 位写入
    .o_dma_ram_wr_en         (dma_ram_wr_en       ),  
    .o_dma_ram_wr_byte_en    (dma_ram_wr_byte_en  ),  
    .o_dma_ram_wr_addr       (dma_ram_wr_addr     ),  
    .o_dma_ram_wr_data       (dma_ram_wr_data     ),  
    // 128 位读取
    .i_dma_rd_en             (dma_rd_en           ),  
    .i_dma_rd_addr           (dma_rd_addr         ),  
	.o_dma_rd_data           (dma_rd_data         ),  
    // 16 位读取
    .o_dma_ram_rd_en         (dma_ram_rd_en       ),  
    .o_dma_ram_rd_addr       (dma_ram_rd_addr     ),  
    .i_dma_ram_rd_data       (dma_ram_rd_data     ),  
	
	// 其他控制信号
	.o_cpld_finish           (cpld_finish          ),  
    .i_mwr_req_cmd           (mwr_req_cmd          ),  
    .o_mwr_tx_start          (mwr_tx_start         ),  
    .i_rd_base_ram_addr      (rd_base_ram_addr     ),  
    .i_wr_base_ram_addr      (wr_base_ram_addr     ),  
	.i_mem_length            (mem_length           )   
);

// DMA 数据 RAM
ram_16in_16out u_ram_16in_16out 
(
    .wr_clk             (pclk_div2            ),  
    .wr_rst             (~core_rst_n          ),  
    .wr_en              (dma_ram_wr_en        ),  
    .wr_byte_en         (dma_ram_wr_byte_en   ),  
    .wr_addr            (dma_ram_wr_addr[11:0]),  
    .wr_data            (dma_ram_wr_data      ),  
    .rd_clk             (pclk_div2            ),  
    .rd_rst             (~core_rst_n          ),  
    .rd_clk_en          (dma_ram_rd_en        ),  
    .rd_addr            (dma_ram_rd_addr[11:0]),  
    .rd_data            (dma_ram_rd_data      )   
);

// PCIe 核封装
// 封装内地址：HSSTLP 为 0x0000..0x6000，PCIe 配置基址为 0x7000。

PCIe_IP
u_ips2l_pcie_wrap
(
    .button_rst_n               (sync_button_rst_n      ),
    .power_up_rst_n             (sync_perst_n           ),
    .perst_n                    (sync_perst_n           ),
    // 时钟与复位
    .pclk                       (pclk                   ),      
    .pclk_div2                  (pclk_div2              ),      
    .ref_clk                    (ref_clk                ),      
    .ref_clk_n                  (ref_clk_n              ),      
    .ref_clk_p                  (ref_clk_p              ),      
    .core_rst_n                 (core_rst_n             ),      
    
    // 通过 APB 访问 DBI 配置
    //.p_clk                      (ref_clk                ),      //input
    .p_sel                      ('d0                    ),      
    .p_strb                     ('d0                    ),      
    .p_addr                     ('d0                    ),      
    .p_wdata                    ('d0                    ),      
    .p_ce                       ('d0                    ),      
    .p_we                       ('d0                    ),      
    .p_rdy                      (                       ),      
    .p_rdata                    (                       ),      
   
    // PHY 差分信号
    .rxn                        (rxn                    ),      
    .rxp                        (rxp                    ),      
    .txn                        (txn                    ),      
    .txp                        (txp                    ),      
    .pcs_nearend_loop           ({4{1'b0}}              ),      
    .pma_nearend_ploop          ({4{1'b0}}              ),      
    .pma_nearend_sloop          ({4{1'b0}}              ),      
    // AXIS 主接口
    .axis_master_tvalid         (axis_master_tvalid     ),      
    .axis_master_tready         (axis_master_tready     ),      
    .axis_master_tdata          (axis_master_tdata      ),      
    .axis_master_tkeep          (axis_master_tkeep      ),      
    .axis_master_tlast          (axis_master_tlast      ),      
    .axis_master_tuser          (axis_master_tuser      ),      
    
    // AXIS 从接口 0
    .axis_slave0_tready         (axis_slave0_tready     ),      
    .axis_slave0_tvalid         (axis_slave0_tvalid     ),      
    .axis_slave0_tdata          (axis_slave0_tdata      ),      
    .axis_slave0_tlast          (axis_slave0_tlast      ),      
    .axis_slave0_tuser          (axis_slave0_tuser      ),      
    
    // AXIS 从接口 1
    .axis_slave1_tready         (axis_slave1_tready     ),      
    .axis_slave1_tvalid         (axis_slave1_tvalid     ),      
    .axis_slave1_tdata          (axis_slave1_tdata      ),      
    .axis_slave1_tlast          (axis_slave1_tlast      ),      
    .axis_slave1_tuser          (axis_slave1_tuser      ),      
    // AXIS 从接口 2
    .axis_slave2_tready         (axis_slave2_tready     ),      
    .axis_slave2_tvalid         (axis_slave2_tvalid     ),      
    .axis_slave2_tdata          (axis_slave2_tdata      ),      
    .axis_slave2_tlast          (axis_slave2_tlast      ),      
    .axis_slave2_tuser          (axis_slave2_tuser      ),      
     
    .pm_xtlh_block_tlp          (                       ),      

	 // MSI 中断接口
    .ven_msi_req                (1'b0                   ),      
    .cfg_msi_en                 (                       ),      
    .ven_msi_grant              (                       ),      
    .ven_msi_tc                 (3'b0                   ),      
    .ven_msi_vector             (5'b0                   ),      
    .cfg_msi_pending            (32'b0                  ),      

	  
    .cfg_send_cor_err_mux       (                       ),      
    .cfg_send_nf_err_mux        (                       ),      
    .cfg_send_f_err_mux         (                       ),      
    .cfg_sys_err_rc             (                       ),      
    .cfg_aer_rc_err_mux         (                       ),      
    // 接收端超时状态
    .radm_cpl_timeout           (                       ),      
    
    // 配置状态信号
    .cfg_max_rd_req_size        (cfg_max_rd_req_size    ),      
    .cfg_bus_master_en          (bus_master_enabled),      
    .cfg_max_payload_size       (cfg_max_payload_size   ),      
    .cfg_ext_tag_en             (                       ),      
    .cfg_rcb                    (cfg_rcb                ),      
    .cfg_mem_space_en           (                       ),      
    .cfg_pm_no_soft_rst         (                       ),      
    .cfg_crs_sw_vis_en          (                       ),      
    .cfg_no_snoop_en            (                       ),      
    .cfg_relax_order_en         (                       ),      
    .cfg_tph_req_en             (                       ),      
    .cfg_pf_tph_st_mode         (                       ),      
    .rbar_ctrl_update           (                       ),      
    .cfg_atomic_req_en          (                       ),      
    
    .cfg_pbus_num               (cfg_pbus_num           ),      
    .cfg_pbus_dev_num           (cfg_pbus_dev_num       ),      
    
    // 调试信号
    .radm_idle                  (                       ),      
    .radm_q_not_empty           (                       ),      
    .radm_qoverflow             (                       ),      
    .diag_ctrl_bus              (2'b0                   ),      
    .cfg_link_auto_bw_mux       (                       ),      // 链路自动带宽中断合并输出
    .cfg_bw_mgt_mux             (                       ),      // 带宽管理中断合并输出
    .cfg_pme_mux                (                       ),      // 电源管理中断合并输出
    .app_ras_des_sd_hold_ltssm  (1'b0                   ),      
    .app_ras_des_tba_ctrl       (2'b0                   ),      
    
    .dyn_debug_info_sel         (4'b0                   ),      
    .debug_info_mux             (                       ),      
    
    // 系统状态
    .smlh_link_up               (smlh_link_up           ),      
    .rdlh_link_up               (rdlh_link_up           ),      
    .smlh_ltssm_state           (smlh_ltssm_state       )       
);

endmodule
