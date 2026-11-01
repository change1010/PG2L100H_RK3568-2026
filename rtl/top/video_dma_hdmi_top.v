`timescale 1ns / 1ps

// 原提供方：Meyesemi
// 原作者：Will

// 原创建时间：2023-03-17

// 目标器件：Pango

`define UD #1
// 双摄像头、原生 HDMI 和以太网输入共用 DDR；公共画布供显示及 PCIe 采集。

module video_dma_hdmi_top #(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        =  32/8
)(
    input                                sys_clk              ,// 系统时钟 27 MHz
    input                                clk_p ,
    input                                clk_n ,
    input                                pcie_ref_clk_p,
    input                                pcie_ref_clk_n,
    input                                pcie_perst_n,
    input      [3:0]                     pcie_rxp,
    input      [3:0]                     pcie_rxn,
    output     [3:0]                     pcie_txp,
    output     [3:0]                     pcie_txn,
    input                                key0                 ,
    input                                key1                 ,
    input                                key2                 ,
    input                                key3                 ,
    input                                key4                 ,
//OV5640
    output  [1:0]                        cmos_init_done       ,//OV5640寄存器初始化完成
    // 第一路摄像头
    inout                                cmos1_scl            ,// 第一路摄像头 I2C
    inout                                cmos1_sda            ,// 第一路摄像头 I2C
    input                                cmos1_vsync          ,// 第一路摄像头场同步
    input                                cmos1_href           ,// 第一路摄像头行有效
    input                                cmos1_pclk           ,// 第一路摄像头像素时钟
    input   [7:0]                        cmos1_data           ,// 第一路摄像头数据
    output                               cmos1_reset          ,// 第一路摄像头复位
    // 第二路摄像头
    inout                                cmos2_scl            ,// 第二路摄像头 I2C
    inout                                cmos2_sda            ,// 第二路摄像头 I2C
    input                                cmos2_vsync          ,// 第二路摄像头场同步
    input                                cmos2_href           ,// 第二路摄像头行有效
    input                                cmos2_pclk           ,// 第二路摄像头像素时钟
    input   [7:0]                        cmos2_data           ,// 第二路摄像头数据
    output                               cmos2_reset          ,// 第二路摄像头复位
// DDR 接口
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
//MS72xx       
    output                               rstn_out                  ,
    output                               iic_tx_scl                ,
    inout                                iic_tx_sda                ,
    output                               hdmi_int_led              ,//MS7200(HDMI RX)初始化完成
// HDMI 输入引脚沿用官方例程；C22 在本板用作 key0，不引入例程的 rst_in。
    input                                pixclk_in                 ,//HDMI RX 像素时钟
    input                                vs_in                     ,//场同步
    input                                hs_in                     ,//行同步
    input                                de_in                     ,//数据有效
    input      [7:0]                     r_in                      ,
    input      [7:0]                     g_in                      ,
    input      [7:0]                     b_in                      ,
input  wire        rgmii_rxc,
    input  wire        rgmii_rx_ctl,
    input  wire [3:0]  rgmii_rxd,
    output wire        rgmii_txc,
    output wire        rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd,
    // HDMI 输出
    output                               pix_clk                   ,// 输出像素时钟
    output     reg                       vs_out                    , 
    output     reg                       hs_out                    , 
    output     reg                       de_out                    ,
    output     reg[7:0]                  r_out                     , 
    output     reg[7:0]                  g_out                     , 
    output     reg[7:0]                  b_out         
);

// DDR 参数
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
    parameter TH_1S = 27'd125000000;// core_clk 为 125 MHz 时对应 1 秒

// CMOS 和以太网每通道轮转 4 个 960x540 RGB565 帧槽；HDMI 使用独立 1080p 区域。
// 地址以 256 位拍为单位，每帧 32400 拍，恰为 2025 个 16 拍突发。

//   历史地址问题（2026-09-28，保留数值供回查）：
//     JC_pds 用 1,036,800 作为槽步进，即实际帧长(32,400) 的 32 倍。
//     ch2/ch3 的最高槽是 11,404,800，经 aq_axi_master_256 的 {addr,5'd0} 左移 5 位后
//     = 364,953,600，超过 ddr3_test 的 28 位地址口上限(268,435,455)，
//     会被 axi_awaddr = m_axi_awaddr[27:0] 截断并回绕，踩到 ch0/ch1 的帧槽；
//     若该口的单位其实是 32bit 字，则 4 倍后为 1.46GB，又超出板上 1GB DDR3。
//     两种解释都会失效，所以不能照搬。
//     改为按真实帧长做步进后，最高槽 = 486,000，左移 5 位 = 15,552,000，
//     在 28 位地址口与 1GB DDR3 两种口径下都安全；12 个槽合计约 12.4MB。

    localparam [24:0] FRAME_BEATS = 25'd32400;   // 一帧的 beat 数
    localparam [24:0] FRAME_LEN   = 25'd32400;   // 读写长度
    localparam [24:0] CH0_BASE    = 25'd0;       // 第一个 960x540 通道起点
    localparam [24:0] CH1_BASE    = 25'd129600;  // 第二个 960x540 通道起点
    localparam [24:0] HDMI_NATIVE_BASE = 25'd2097152; // 字节地址 0x04000000，预留四个 1080p 帧槽
    localparam [24:0] CH3_BASE    = 25'd388800;  // 第四个 960x540 通道起点

    // 帧槽地址显式限定为 25 位，避免端口连接时隐式截位。
    // FRAME_BEATS/FRAME_LEN 仅用于 CMOS 和以太网的 960x540 帧缓存。
    localparam [24:0] CH0_ADDR_0 = CH0_BASE + 25'd0 * FRAME_BEATS;
    localparam [24:0] CH0_ADDR_1 = CH0_BASE + 25'd1 * FRAME_BEATS;
    localparam [24:0] CH0_ADDR_2 = CH0_BASE + 25'd2 * FRAME_BEATS;
    localparam [24:0] CH0_ADDR_3 = CH0_BASE + 25'd3 * FRAME_BEATS;
    localparam [24:0] CH1_ADDR_0 = CH1_BASE + 25'd0 * FRAME_BEATS;
    localparam [24:0] CH1_ADDR_1 = CH1_BASE + 25'd1 * FRAME_BEATS;
    localparam [24:0] CH1_ADDR_2 = CH1_BASE + 25'd2 * FRAME_BEATS;
    localparam [24:0] CH1_ADDR_3 = CH1_BASE + 25'd3 * FRAME_BEATS;
    localparam [24:0] CH3_ADDR_0 = CH3_BASE + 25'd0 * FRAME_BEATS;
    localparam [24:0] CH3_ADDR_1 = CH3_BASE + 25'd1 * FRAME_BEATS;
    localparam [24:0] CH3_ADDR_2 = CH3_BASE + 25'd2 * FRAME_BEATS;
    localparam [24:0] CH3_ADDR_3 = CH3_BASE + 25'd3 * FRAME_BEATS;

// 旧 Y/C 平面支路的编译期开关，当前为 0，不生成对应采集和传输实例。
// 当前道路检测直接使用原始 HDMI RGB，不经过此支路。

    localparam integer PLANE_ENABLE = 0;


    reg  [15:0]                 rstn_1ms            ;
    wire                        cmos_scl            ;// 摄像头 I2C 时钟
    wire                        cmos_sda            ;// 摄像头 I2C 数据
    wire                        cmos_vsync          ;// 摄像头场同步
    wire                        cmos_href           ;// 摄像头行有效
    wire                        cmos_pclk           ;// 摄像头像素时钟
    wire   [7:0]                cmos_data           ;// 摄像头数据
    wire                        cmos_reset          ;// 摄像头复位
    wire                        initial_en          ;
    wire[15:0]                  cmos1_d_16bit       ;
    wire                        cmos1_href_16bit    ;
    reg [7:0]                   cmos1_d_d0          ;
    reg                         cmos1_href_d0       ;
    reg                         cmos1_vsync_d0      ;
    wire                        cmos1_capture_clk;
    wire                        cmos1_pixel_href;
    wire                        cmos1_pixel_vsync;
    wire[15:0]                  cmos2_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    reg [7:0]                   cmos2_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_capture_clk;
    wire                        cmos2_pixel_href;
    wire                        cmos2_pixel_vsync;
    wire[15:0]                  o_rgb565            ;
    wire[15:0]                  cam0_pixel          /*synthesis PAP_MARK_DEBUG="1"*/;
    wire[15:0]                  cam1_pixel          /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cam0_valid          /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cam1_valid          /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        pix_rstn            /*synthesis PAP_MARK_DEBUG="1"*/;
    wire [2:0] display_sel_mode_r;
    wire [2:0] display_process_mode_r;
    wire [4:0] key_level_n;

    reg [2:0]                   pix_rstn_sync = 3'b000 /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cfg_clk;
    wire                        clk_25M;
    wire                        locked;
    wire                        init_over_rx;
    wire                        core_clk;
    wire                        pll_lock;
    wire                        phy_pll_lock;
    wire                        gpll_lock;
    wire                        rst_gpll_lock;
    wire                        ddrphy_cpd_lock;
    wire                        vs_o;
    wire                        hs_o;
    wire                        de_o;
    wire                        de_re               ;
// AXI 总线
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_awvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_arvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   /* synthesis syn_keep = 1 */;
    wire                        axi_rvalid                  /* synthesis syn_keep = 1 */;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;

// HDMI 输入在 pixclk_in 域寄存一拍，供原生采集及道路检测使用。

    reg  [7:0]  r_in_d0  /*synthesis PAP_MARK_DEBUG="1"*/;
    reg  [7:0]  g_in_d0  /*synthesis PAP_MARK_DEBUG="1"*/;
    reg  [7:0]  b_in_d0  /*synthesis PAP_MARK_DEBUG="1"*/;
    reg         vs_in_d0 /*synthesis PAP_MARK_DEBUG="1"*/;
    reg         hs_in_d0 /*synthesis PAP_MARK_DEBUG="1"*/;
    reg         de_in_d0 /*synthesis PAP_MARK_DEBUG="1"*/;

// 锁相环

    pll u_pll (
        .clkin1   (  sys_clk    ),// 系统时钟 27 MHz
        .clkout0  (  pix_clk    ),// 1080p60 像素时钟 148.5 MHz
        .clkout1  (  cfg_clk    ),// 配置时钟 10 MHz
        .clkout2  (  clk_25M    ),//24.75MHz（请求 25MHz，VCO 限制取最近值）
        .lock (  locked     )
    );

//配置7210
wire init_over_tx;
    ms72xx_ctl ms72xx_ctl(
        .clk             (  cfg_clk        ), 
        .rst_n           (  rstn_out       ), 
        .init_over    (  init_over_tx   ), 
        .init_over_rx    (  init_over_rx   ), 
//        .iic_tx_scl      (  iic_tx_scl     ), //output      iic_scl,
//        .iic_tx_sda      (  iic_tx_sda     ), //inout       iic_sda
        .iic_scl         (  iic_tx_scl        ), 
        .iic_sda         (  iic_tx_sda        )  
    );
//   assign    hdmi_int_led    =    init_over_tx; 
    
    // HDMI RX 输入寄存级（pixclk_in 域）。H4 的降采样从这组寄存器取数。
    always @(posedge pixclk_in)
    begin
        r_in_d0  <= r_in ;
        g_in_d0  <= g_in ;
        b_in_d0  <= b_in ;
        vs_in_d0 <= vs_in;
        hs_in_d0 <= hs_in;
        de_in_d0 <= de_in;
    end

    always @(posedge cfg_clk)
    begin
    	if(!locked)
    	    rstn_1ms <= 16'd0;
    	else
    	begin
    		if(rstn_1ms == 16'h2710)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    
    assign rstn_out = (rstn_1ms == 16'h2710);

//配置CMOS///////////////////////////////////////////////////////////////////////////////////
// OV5640 寄存器配置使能
    power_on_delay	power_on_delay_inst(
    	.clk_sys                 (sys_clk        ),// 输入时钟 27 MHz
    	.reset_n                 (locked         ),// 锁相环失锁后重新复位
    	.camera1_rstn            (cmos1_reset    ),
    	.camera2_rstn            (cmos2_reset    ),
    	.camera_pwnd             (               ),
    	.initial_en              (initial_en     ) 
    );
// 第一路摄像头配置
    reg_config	coms1_reg_config(
    	.clk_25M                 (clk_25M            ),
    	.camera_rstn             (cmos1_reset        ),
    	.initial_en              (initial_en         ),
    	.i2c_sclk                (cmos1_scl          ),
    	.i2c_sdat                (cmos1_sda          ),
    	.reg_conf_done           (cmos_init_done[0]  ),
    	.reg_index               (                   ),
    	.clock_20k               (                   ) 
    );

// 第二路摄像头配置
    reg_config	coms2_reg_config(
    	.clk_25M                 (clk_25M            ),
    	.camera_rstn             (cmos2_reset        ),
    	.initial_en              (initial_en         ),
    	.i2c_sclk                (cmos2_scl          ),
    	.i2c_sdat                (cmos2_sda          ),
    	.reg_conf_done           (cmos_init_done[1]  ),
    	.reg_index               (                   ),
    	.clock_20k               (                   ) 
    );
//CMOS 8bit转16bit///////////////////////////////////////////////////////////////////////////////////
// 第一路摄像头采集
    GTP_CLKBUFG u_cmos1_capture_bufg (
        .CLKIN(cmos1_pclk), .CLKOUT(cmos1_capture_clk));
    GTP_CLKBUFG u_cmos2_capture_bufg (
        .CLKIN(cmos2_pclk), .CLKOUT(cmos2_capture_clk));
    always@(posedge cmos1_capture_clk)
        begin
            cmos1_d_d0        <= cmos1_data    ;
            cmos1_href_d0     <= cmos1_href    ;
            cmos1_vsync_d0    <= cmos1_vsync   ;
        end

    cmos_8_16bit cmos1_8_16bit(
    	.pclk           (cmos1_capture_clk),
    	.rst_n          (cmos_init_done[0]),
    	.pdata_i        (cmos1_d_d0       ),
    	.de_i           (cmos1_href_d0    ),
    	.vs_i           (cmos1_vsync_d0    ),
    	
    	.pixel_clk      (                 ),// 兼容输出，仍为原始像素时钟
        .pixel_href     (cmos1_pixel_href ),
        .pixel_vsync    (cmos1_pixel_vsync),
    	.pdata_o        (cmos1_d_16bit    ),
    	.de_o           (cmos1_href_16bit ) 
    );
// 第二路摄像头采集
    always@(posedge cmos2_capture_clk)
        begin
            cmos2_d_d0        <= cmos2_data    ;
            cmos2_href_d0     <= cmos2_href    ;
            cmos2_vsync_d0    <= cmos2_vsync   ;
        end

    cmos_8_16bit cmos2_8_16bit(
    	.pclk           (cmos2_capture_clk),
    	.rst_n          (cmos_init_done[1]),
    	.pdata_i        (cmos2_d_d0       ),
    	.de_i           (cmos2_href_d0    ),
    	.vs_i           (cmos2_vsync_d0    ),
    	
    	.pixel_clk      (                 ),
        .pixel_href     (cmos2_pixel_href ),
        .pixel_vsync    (cmos2_pixel_vsync),
    	.pdata_o        (cmos2_d_16bit    ),
    	.de_o           (cmos2_href_16bit ) 
    );
// 按 JC_pds 的显示输出思路：先把 1280x720 中心裁剪成 960x540，再写入帧缓存。
// 帧缓存因此存 960x540，显示侧再 2 倍放大到 1920x1080。
wire [15:0] cam0_tailor_data;
wire [15:0] cam1_tailor_data;
wire        cam0_tailor_valid;
wire        cam1_tailor_valid;

cmos_tailor #(
    .INPUT_WDITH  (1280), .INPUT_HIGHT  (720),
    .OUTPUT_WDITH (960),  .OUTPUT_HIGHT (540)
) cmos1_tailor (
    .rst_n            (cmos_init_done[0]),
    .cam_pclk         (cmos1_capture_clk),
    .cam_vsync        (cmos1_pixel_vsync),
    .cam_href         (cmos1_pixel_href ),
    .cam_data         (cmos1_d_16bit    ),
    .cam_data_valid   (cmos1_href_16bit ),
    .cmos_frame_valid (cam0_tailor_valid),
    .cmos_frame_data  (cam0_tailor_data )
);

cmos_tailor #(
    .INPUT_WDITH  (1280), .INPUT_HIGHT  (720),
    .OUTPUT_WDITH (960),  .OUTPUT_HIGHT (540)
) cmos2_tailor (
    .rst_n            (cmos_init_done[1]),
    .cam_pclk         (cmos2_capture_clk),
    .cam_vsync        (cmos2_pixel_vsync),
    .cam_href         (cmos2_pixel_href ),
    .cam_data         (cmos2_d_16bit    ),
    .cam_data_valid   (cmos2_href_16bit ),
    .cmos_frame_valid (cam1_tailor_valid),
    .cmos_frame_data  (cam1_tailor_data )
);

// 裁剪后的 960x540 RGB565 流（保持原厂字节序）
assign cam0_pixel = {cam0_tailor_data[4:0],cam0_tailor_data[10:5],cam0_tailor_data[15:11]};
assign cam1_pixel = {cam1_tailor_data[4:0],cam1_tailor_data[10:5],cam1_tailor_data[15:11]};
assign cam0_valid = cam0_tailor_valid;
assign cam1_valid = cam1_tailor_valid;

    key_debounce u_key0 (.clk(pix_clk), .rst_n(pix_rstn), .key_n(key0),
        .press(), .level(key_level_n[0]));
    key_debounce u_key1 (.clk(pix_clk), .rst_n(pix_rstn), .key_n(key1),
        .press(), .level(key_level_n[1]));
    key_debounce u_key2 (.clk(pix_clk), .rst_n(pix_rstn), .key_n(key2),
        .press(), .level(key_level_n[2]));
    key_debounce u_key3 (.clk(pix_clk), .rst_n(pix_rstn), .key_n(key3),
        .press(), .level(key_level_n[3]));
    key_debounce u_key4 (.clk(pix_clk), .rst_n(pix_rstn), .key_n(key4),
        .press(), .level(key_level_n[4]));

    wire drive_state /*synthesis PAP_MARK_DEBUG="1"*/;
    wire [1:0] turn_request /*synthesis PAP_MARK_DEBUG="1"*/;
    wire alarm_reset /*synthesis PAP_MARK_DEBUG="1"*/;
    wire [31:0] alarm_reset_seq;
    wire [31:0] key_event_seq /*synthesis PAP_MARK_DEBUG="1"*/;
    key_state_ctrl u_key_state_ctrl (
        .clk(pix_clk), .rst_n(pix_rstn), .key_level_n(key_level_n),
        .timing_vs(vs_o), .timing_de(de_re),
        .source_mode(display_sel_mode_r),
        .process_mode(display_process_mode_r),
        .drive_state(drive_state), .turn_request(turn_request),
        .alarm_reset(alarm_reset), .event_seq(key_event_seq),
        .alarm_reset_seq(alarm_reset_seq)
    );

    // C19：MS7200(HDMI RX) 初始化完成指示。
    //   原来是 ddr_init_done，与 C18(LED1) 完全重复、不承载信息；
    //   官方 HDMI_IN 例程就是把 C19 接 init_over_rx，零引脚成本换来 bring-up 观测点。
    //   判据：C19 亮 = MS7200 已通过 I2C 配置（不要求插着 HDMI 源）。
    assign hdmi_int_led = init_over_rx;

    always @(posedge pix_clk) begin
        pix_rstn_sync <= {pix_rstn_sync[1:0], ddr_init_done};
    end

    assign pix_rstn = pix_rstn_sync[2];

    // 四路帧缓存共用读写仲裁及唯一 AXI 主控。
    // ch0/1 为摄像头，ch2 为原生 HDMI，ch3 为以太网，各使用四个帧槽。
    // 单画面和四画面共享这些输入，由 display_sel_mode_r 选择显示布局。
    
    wire         wr_burst_req, wr_burst_data_req, wr_burst_finish;
    wire [9:0]   wr_burst_len;
    wire [24:0]  wr_burst_addr;
    wire [255:0] wr_burst_data;
    wire         rd_burst_req, rd_burst_data_valid, rd_burst_finish;
    wire [9:0]   rd_burst_len;
    wire [24:0]  rd_burst_addr;
    wire [255:0] rd_burst_data;

    // ---- 通道 P：写请求/读请求 用户侧握手 ----
    wire [1:0]   ch0_write_addr_index, ch0_read_addr_index;
    wire         ch0_write_req, ch0_write_req_ack;
    wire         ch0_read_req, ch0_read_req_ack, ch0_read_en;
    wire [15:0]  ch0_read_data;
    wire [1:0]   ch1_write_addr_index, ch1_read_addr_index;
    wire         ch1_write_req, ch1_write_req_ack;
    wire         ch1_read_req, ch1_read_req_ack, ch1_read_en;
    wire [15:0]  ch1_read_data;
    wire         ch2_read_req, ch2_read_req_ack, ch2_read_en;
    wire [15:0]  ch2_read_data;
    wire [1:0]  ch3_read_addr_index;
    wire         ch3_read_req, ch3_read_req_ack, ch3_read_en;
    wire [15:0]  ch3_read_data;
    wire [15:0] ch3_read_data_raw;
    wire        eth_display_valid;
    reg  [2:0]  eth_display_valid_sync;
    always @(posedge pix_clk or negedge ddr_init_done)
        if (!ddr_init_done) eth_display_valid_sync <= 3'd0;
        else                eth_display_valid_sync <= {eth_display_valid_sync[1:0], eth_display_valid};
    assign ch3_read_data = eth_display_valid_sync[2] ? ch3_read_data_raw : 16'd0;

    // ---- 通道 P：与仲裁器之间的突发接口 ----
    wire         ch0_wr_burst_req, ch0_wr_burst_data_req, ch0_wr_burst_finish;
    wire [9:0]   ch0_wr_burst_len;
    wire [24:0]  ch0_wr_burst_addr;
    wire [255:0] ch0_wr_burst_data;
    wire         ch0_rd_burst_req, ch0_rd_burst_data_valid, ch0_rd_burst_finish;
    wire [9:0]   ch0_rd_burst_len;
    wire [24:0]  ch0_rd_burst_addr;
    wire [255:0] ch0_rd_burst_data;
    wire         ch1_wr_burst_req, ch1_wr_burst_data_req, ch1_wr_burst_finish;
    wire [9:0]   ch1_wr_burst_len;
    wire [24:0]  ch1_wr_burst_addr;
    wire [255:0] ch1_wr_burst_data;
    wire         ch1_rd_burst_req, ch1_rd_burst_data_valid, ch1_rd_burst_finish;
    wire [9:0]   ch1_rd_burst_len;
    wire [24:0]  ch1_rd_burst_addr;
    wire [255:0] ch1_rd_burst_data;
    wire         ch2_wr_burst_req, ch2_wr_burst_data_req, ch2_wr_burst_finish;
    wire [9:0]   ch2_wr_burst_len;
    wire [24:0]  ch2_wr_burst_addr;
    wire [255:0] ch2_wr_burst_data;
    wire         ch2_rd_burst_req, ch2_rd_burst_data_valid, ch2_rd_burst_finish;
    wire [9:0]   ch2_rd_burst_len;
    wire [24:0]  ch2_rd_burst_addr;
    wire [255:0] ch2_rd_burst_data;
    wire         ch3_wr_burst_req, ch3_wr_burst_data_req, ch3_wr_burst_finish;
    wire [9:0]   ch3_wr_burst_len;
    wire [24:0]  ch3_wr_burst_addr;
    wire [255:0] ch3_wr_burst_data;
    wire         ch3_rd_burst_req, ch3_rd_burst_data_valid, ch3_rd_burst_finish;
    wire [9:0]   ch3_rd_burst_len;
    wire [24:0]  ch3_rd_burst_addr;
    wire [255:0] ch3_rd_burst_data;

    wire hdmi_native_frame_valid, hdmi_native_source_live;
    wire native_canvas_invalid;
    wire [31:0] hdmi_native_committed, hdmi_native_rejected, hdmi_capture_rejected;
    wire [31:0] hdmi_native_underflows, hdmi_native_missed;
    wire hdmi_input_boundary;
    wire [31:0] hdmi_source_sequence,hdmi_source_epoch;
    wire [63:0] hdmi_source_timestamp;
    wire [127:0] hdmi_pixel_identity,hdmi_bus_identity;
    wire [127:0] hdmi_left_check,hdmi_bus_left_check,hdmi_bus_left_counts;
    wire [63:0] hdmi_left_counts;
    reg native_vs_d;
    wire native_frame_start = native_vs_d && !vs_o;
    // 旧 Y/C 三帧槽紧接 RGB 快照区，地址以 32 字节为单位：
    // RGB 区间为 [524288,913088)，Y/C 区间为 [913088,956288)。
    localparam [15:0] ROAD_YC_PLANE_BEATS = 16'd7200;
    localparam [15:0] ROAD_YC_SLOT_BEATS = 16'd14400;
    // 保留的公共 RGB565 画布平面支路。
    wire        plane_enable;
    wire [3:0]  plane_period_div;
    wire [24:0] plane_cfg_base;
    wire [15:0] plane_cfg_slot_beats;
    wire [1:0]  plane_cfg_slot_count;
    wire [15:0] plane_geom_w, plane_geom_h;
    wire [3:0]  plane_format;
    wire [63:0] plane_host_addr, plane_completion;
    wire [31:0] plane_consumer;
    wire        plane_gray_valid, plane_gray_sof;
    wire [7:0]  plane_gray;
    wire [7:0]  plane_chroma;
    wire        plane_wr_req, plane_wr_data_req, plane_wr_finish, plane_wr_data_valid;
    wire [9:0]  plane_wr_len;
    wire [24:0] plane_wr_addr;
    wire        plane_rd_req, plane_rd_data_req, plane_rd_finish;
    wire [24:0] plane_rd_addr;
    wire        plane_rd_done, plane_rd_error, plane_rd_stream_error, plane_reader_busy;
    wire        plane_tx_data_error, plane_slot_locked;
    wire [31:0] plane_cdc_skipped;
    wire [1:0]  plane_locked_slot;
    wire [24:0] plane_reader_base, plane_completed_base;
    wire [31:0] plane_tx_frame_seq, plane_tx_frame_epoch, plane_tx_frame_config, plane_skipped_sync;
    wire [2:0]  plane_tx_frame_layout;
    wire        plane_frame_pending;
    wire [127:0] plane_reader_data;
    wire        plane_reader_valid, plane_reader_ready;
    wire [15:0] plane_reader_bursts;
    wire plane_axi_rd_fault, plane_axi_wr_fault;
    wire video_tx_clk, video_pcie_reset_n, video_run, video_source_idle;
    wire plane_reset_n=ddr_init_done && pix_rstn && video_pcie_reset_n;
    (* ASYNC_REG = "TRUE" *) reg [2:0] plane_mem_run_sync;
    always @(posedge core_clk or negedge plane_reset_n) begin
        if(!plane_reset_n) plane_mem_run_sync<=0;
        else plane_mem_run_sync<={plane_mem_run_sync[1:0],1'b1};
    end
    wire        plane_tx_data_valid, plane_tx_data_ready;
    wire [127:0] plane_tx_data;
    wire        plane_tx_frame_accept, plane_tx_frame_done;
    wire        plane_rd_start;
    wire [255:0] plane_wr_data;
    wire        plane_frame_done, plane_overflow, plane_src_lost;
    wire [1:0]  plane_slot_index;
    wire [31:0] plane_frame_count, plane_completed_sequence, plane_completed_epoch, plane_completed_config;
    wire [2:0]  plane_completed_layout;
    // 平面支路共享画布帧标识；信号须在生成块前声明，
    // 避免隐式网络把多位标识截为一位。
    wire feature_boundary,feature_begin,feature_end,feature_invalid;
    wire [31:0] feature_sequence,feature_epoch,feature_capture_config;
    wire [31:0] video_frame_epoch,video_frame_config;
    wire [127:0] video_frame_native_identity;
    wire feature_reg_wr,feature_read_en;
    wire [31:0] feature_reg_addr,feature_reg_data,feature_read_addr;
    wire [127:0] feature_read_data;
    // 先声明并驱动复位释放信号，
    // 供下方公共画布平面生成块使用。
    wire dark_reset_n = ddr_init_done && pix_rstn && video_pcie_reset_n;
    (* ASYNC_REG = "TRUE" *) reg [15:0] dark_reset_release;
    wire dark_capture_reset_n = dark_reset_release[15];
    always @(posedge pix_clk or negedge dark_reset_n) begin
        if (!dark_reset_n) dark_reset_release <= 0;
        else dark_reset_release <= {dark_reset_release[14:0],1'b1};
    end
    // ---- 方案 A：0x129 改取 TX 的 producer ----
    // ⚠ 必须声明在此处（首次使用之前）。本文件已经吃过一次亏：shared_rd_data
    //   曾在 817 行被引用、1051 行才声明，被综合器当成 1 位隐式线
    //   （W: Verilog-2057），255 位平面读数据被截断成 1 位。
    wire [31:0] plane_producer;           // TX 已整帧提交到宿主内存的帧数（pclk_div2 域）
    wire [31:0] plane_dropped_frames;     // 发送端拒绝帧数，位于 pclk_div2 域
    wire [7:0]  plane_overflow_row;       // store 溢出时的行号（pixclk_in 域）
    wire [7:0]  plane_overflow_row_sync;  // 两拍同步后的版本（video_tx_clk 域）
    wire plane_frame_done_toggle; // 旧存储诊断信号；邮箱使用 frame_done。

    // ch3 为独立以太网输入。

    // 单画面、四画面及公共画布信号须先声明后连接，避免生成一位隐式线。
    // 历史漏声明使 quad_vout_data 截位并导致 240 条悬空连接。
    wire         quad_display_mode;
    wire         single_ch0_read_req, single_ch0_read_en;
    wire         single_ch1_read_req, single_ch1_read_en;
    wire         single_ch2_read_req, single_ch2_read_en;
    wire single_ch3_read_req, single_ch3_read_en;
    wire         quad_ch0_read_req, quad_ch0_read_en;
    wire         quad_ch1_read_req, quad_ch1_read_en;
    wire         quad_ch2_read_req, quad_ch2_read_en;
    wire         quad_ch3_read_req, quad_ch3_read_en;
    wire         quad_hs, quad_vs, quad_de;
    wire [15:0]  quad_vout_data;
    wire         disp_hs, disp_vs, disp_de;
    wire [15:0]  disp_vout_data;
    wire         color_canvas_hs, color_canvas_vs, color_canvas_de;
    wire [15:0]  color_canvas_data;
    wire [2:0]   color_canvas_source;
    assign native_canvas_invalid = (color_canvas_source == 3'd2 || color_canvas_source == 3'd4) &&
                                   !hdmi_native_frame_valid;
    wire [31:0]  color_canvas_generation, color_canvas_request;
    wire         dark_control_boundary, dark_sample_enable;
    wire [7:0]   dark_sample_threshold, dark_sample_strength;
    wire [31:0]  dark_sample_generation, dark_sample_request;
    localparam integer DARK_CANVAS_STAGES = 13;
    reg [2:0] process_mode_pipe [0:DARK_CANVAS_STAGES-1];
    integer process_stage;
    wire         display_canvas_hs, display_canvas_vs, display_canvas_de;
    wire [15:0]  display_canvas_data;

    // 共享 DDR 突发接口须在首次使用前声明，尤其要保持读数据为 256 位。
    // 历史隐式线问题仅报 Verilog-2057，仍会截断数据。
    wire         shared_wr_req, shared_wr_data_req, shared_wr_finish;
    wire         shared_rd_req, shared_rd_data_valid, shared_rd_finish;
    wire [9:0]   shared_wr_len, shared_rd_len;
    wire [24:0]  shared_wr_addr, shared_rd_addr;
    wire [255:0] shared_wr_data, shared_rd_data;
    assign rd_burst_data = shared_rd_data;

    wire rgb_enable, rgb_armed, rgb_busy, rgb_pending;
    wire [3:0] rgb_period;
    wire [63:0] rgb_host_addr, rgb_completion;
    wire [31:0] rgb_consumer, rgb_producer, rgb_dropped;
    wire [127:0] rgb_read_data, rgb_mem_status, rgb_pix_status;
    wire rgb_wr_req, rgb_wr_data_req, rgb_wr_finish;
    wire [9:0] rgb_wr_len;
    wire [24:0] rgb_wr_addr, rgb_rd_addr;
    wire [255:0] rgb_wr_data;
    wire rgb_rd_req, rgb_rd_valid, rgb_rd_finish;
    wire rgb_frame_accept, rgb_frame_done, rgb_data_valid, rgb_data_error, rgb_data_ready;
    wire [31:0] rgb_frame_seq, rgb_frame_epoch, rgb_frame_config;
    wire [2:0] rgb_frame_layout;
    wire [127:0] rgb_data;
    wire small_rd_req, small_rd_valid, small_rd_finish;
    wire [9:0] small_rd_len;
    wire [24:0] small_rd_addr;
    wire rgb_read = feature_read_addr>=32'h0c0 && feature_read_addr<=32'h0df;
    wire rgb_write = feature_reg_addr>=32'h0c0 && feature_reg_addr<=32'h0df;
    rgb_small_ctl u_rgb_small_ctl (
        .clk(video_tx_clk),.rst_n(plane_reset_n),
        .reg_wr(feature_reg_wr && rgb_write),.reg_addr(feature_reg_addr),.reg_data(feature_reg_data),
        .read_en(feature_read_en && rgb_read),.read_addr(feature_read_addr),.read_data(rgb_read_data),
        .enable(rgb_enable),.armed(rgb_armed),.period_div(rgb_period),
        .host_addr(rgb_host_addr),.completion(rgb_completion),.consumer(rgb_consumer),
        .producer(rgb_producer),.tx_dropped(rgb_dropped),.mem_status(rgb_mem_status),.pix_status(rgb_pix_status),
        .pending(rgb_pending),.tx_busy(rgb_busy)
    );
    rgb_small_channel u_rgb_small_channel (
        .rst_n(ddr_init_done && pix_rstn),.tx_rst_n(plane_reset_n),
        .pix_clk(pix_clk),.mem_clk(core_clk),.tx_clk(video_tx_clk),
        .boundary(feature_boundary),.de(color_canvas_de),.pixel(color_canvas_data),.invalid(native_canvas_invalid),
        .source_sequence(feature_sequence),.source_epoch(feature_epoch),.source_config(feature_capture_config),
        .image_generation(color_canvas_generation),.source_layout(color_canvas_source),
        .enable(rgb_enable),.period_div(rgb_period),
        .wr_req(rgb_wr_req),.wr_len(rgb_wr_len),.wr_addr(rgb_wr_addr),.wr_data(rgb_wr_data),
        .wr_data_req(rgb_wr_data_req),.wr_finish(rgb_wr_finish),.wr_fault(plane_axi_wr_fault),
        .rd_req(rgb_rd_req),.rd_addr(rgb_rd_addr),.rd_valid(rgb_rd_valid),
        .rd_finish(rgb_rd_finish),.rd_fault(plane_axi_rd_fault),.rd_data(shared_rd_data),
        .frame_valid(rgb_pending),.frame_seq(rgb_frame_seq),.frame_epoch(rgb_frame_epoch),
        .frame_config(rgb_frame_config),.frame_layout(rgb_frame_layout),
        .frame_accept(rgb_frame_accept),.frame_done(rgb_frame_done),
        .data(rgb_data),.data_valid(rgb_data_valid),.data_error(rgb_data_error),.data_ready(rgb_data_ready),
        .mem_status(rgb_mem_status),.pix_status(rgb_pix_status)
    );

    // 旧参考图像传输已停用，不产生帧、DDR 请求或 DMA 写入。
    wire ref_enable=1'b0, ref_pending=1'b0, ref_idle=1'b1, ref_busy;
    wire [191:0] ref_host_addrs=192'd0;
    wire [63:0] ref_completion=64'd0;
    wire [31:0] ref_consumer=32'd0,ref_producer,ref_dropped,ref_session=32'd0;
    wire [31:0] ref_stop_cookie=32'd0,ref_stop_sent;
    wire [127:0] ref_read_data,ref_data=128'd0;
    wire [287:0] ref_frame_metadata=288'd0;
    wire ref_wr_req,ref_wr_data_req,ref_wr_finish,ref_rd_req,ref_rd_valid,ref_rd_finish;
    wire [9:0] ref_wr_len;
    wire [24:0] ref_wr_addr,ref_rd_addr;
    wire [255:0] ref_wr_data;
    wire ref_frame_accept,ref_frame_done,ref_data_valid,ref_data_error,ref_data_ready;
    wire ref_read=feature_read_addr>=32'h0e0 && feature_read_addr<=32'h0ff;
    wire ref_write=feature_reg_addr>=32'h0e0 && feature_reg_addr<=32'h0ff;
    assign ref_wr_req=1'b0;
    assign ref_wr_len=10'd0;
    assign ref_wr_addr=25'd0;
    assign ref_wr_data=256'd0;
    assign ref_rd_req=1'b0;
    assign ref_rd_addr=25'd0;
    assign ref_data_valid=1'b0;
    assign ref_data_error=1'b0;
    fpga_road_channel u_fpga_road (
        .rst_n(plane_reset_n),.pix_clk(pixclk_in),.bus_clk(video_tx_clk),
        .boundary(hdmi_input_boundary),.de(de_in_d0),.rgb({r_in_d0,g_in_d0,b_in_d0}),
        .source_epoch(hdmi_source_epoch),.source_sequence(hdmi_source_sequence),
        .source_timestamp(hdmi_source_timestamp),
        .read_en(feature_read_en && ref_read),.read_addr(feature_read_addr),
        .reg_wr(feature_reg_wr && ref_write),.reg_addr(feature_reg_addr),.reg_data(feature_reg_data),
        .read_data(ref_read_data)
    );
    stage1_status_cdc u_native_identity (
        .rst_n(plane_reset_n),.src_clk(pix_clk),.dst_clk(video_tx_clk),
        .source_state(hdmi_pixel_identity),.state_snapshot(hdmi_bus_identity),.generation()
    );
    stage1_status_cdc u_native_left_check (
        .rst_n(plane_reset_n),.src_clk(pix_clk),.dst_clk(video_tx_clk),
        .source_state(hdmi_left_check),.state_snapshot(hdmi_bus_left_check),.generation()
    );
    stage1_status_cdc u_native_left_counts (
        .rst_n(plane_reset_n),.src_clk(pix_clk),.dst_clk(video_tx_clk),
        .source_state({64'd0,hdmi_left_counts}),.state_snapshot(hdmi_bus_left_counts),.generation()
    );

    wire [31:0]  m_axi_awaddr, m_axi_araddr;
    wire [7:0]   m_axi_awlen,  m_axi_arlen;
    wire [0:0]   m_axi_awid,   m_axi_arid, m_axi_rid;
    wire [255:0] m_axi_wdata,  m_axi_rdata;
    wire [31:0]  m_axi_wstrb;
    wire         m_axi_awvalid, m_axi_awready, m_axi_wvalid, m_axi_wready;
    wire         m_axi_bready, m_axi_arvalid, m_axi_arready, m_axi_rready;
    // 显式声明摄像头写地址索引，避免隐式网络。
    wire         m_axi_rvalid;

    wire         single_hs, single_vs, single_de;
    wire [15:0]  single_vout_data;

    // ---- ch0 写请求生成（按 cmos1 场同步翻转帧地址索引）----
    cmos_write_req_gen u_cmos_write_req_gen0 (
        .rst               (~cmos_init_done[0]    ),   // 与摄像头初始化对齐（JC_pds: ~cmos_inital_done）
        .pclk              (cmos1_capture_clk   ),
        .cmos_vsync        (cmos1_pixel_vsync   ),
        .write_req         (ch0_write_req        ),
        .write_addr_index  (ch0_write_addr_index ),
        .read_addr_index   (ch0_read_addr_index  ),
        .write_req_ack     (ch0_write_req_ack    )
    );

    // ---- ch0 帧缓存：960x540，用户侧 16bit，存储侧 256bit，4 个帧槽轮转 ----
    frame_read_write #(
        .MEM_DATA_BITS      (256), .READ_DATA_BITS(16), .WRITE_DATA_BITS(16),
        .ADDR_BITS          (25 ), .BUSRT_BITS    (10), .BURST_SIZE     (16)
    ) frame_read_write_m0 (
        .rst                    (~ddr_init_done         ),
        .mem_clk                (core_clk               ),
        .rd_burst_req           (ch0_rd_burst_req       ),
        .rd_burst_len           (ch0_rd_burst_len       ),
        .rd_burst_addr          (ch0_rd_burst_addr      ),
        .rd_burst_data_valid    (ch0_rd_burst_data_valid),
        .rd_burst_data          (ch0_rd_burst_data      ),
        .rd_burst_finish        (ch0_rd_burst_finish    ),
        .read_clk               (pix_clk                ),
        .read_req               (ch0_read_req           ),
        .read_req_ack           (ch0_read_req_ack       ),
        .read_finish            (                       ),
        .read_addr_0            (CH0_ADDR_0)              ,
        .read_addr_1            (CH0_ADDR_1)              ,
        .read_addr_2            (CH0_ADDR_2)              ,
        .read_addr_3            (CH0_ADDR_3)              ,
        .read_addr_index        (ch0_read_addr_index    ),
        .read_len               (FRAME_LEN              ),
        .read_en                (ch0_read_en            ),
        .read_data              (ch0_read_data          ),
        .wr_burst_req           (ch0_wr_burst_req       ),
        .wr_burst_len           (ch0_wr_burst_len       ),
        .wr_burst_addr          (ch0_wr_burst_addr      ),
        .wr_burst_data_req      (ch0_wr_burst_data_req  ),
        .wr_burst_data          (ch0_wr_burst_data      ),
        .wr_burst_finish        (ch0_wr_burst_finish    ),
        .write_clk              (cmos1_capture_clk      ),
        .write_req              (ch0_write_req          ),
        .write_req_ack          (ch0_write_req_ack      ),
        .write_finish           (                       ),
        .write_addr_0           (CH0_ADDR_0)              ,
        .write_addr_1           (CH0_ADDR_1)              ,
        .write_addr_2           (CH0_ADDR_2)              ,
        .write_addr_3           (CH0_ADDR_3)              ,
        .write_addr_index       (ch0_write_addr_index   ),
        .write_len              (FRAME_LEN              ),
        .write_en               (cam0_valid               ),
        .write_data             (cam0_pixel              )
    );

    // ch1：第二路摄像头帧缓存。
    
    cmos_write_req_gen u_cmos_write_req_gen1 (
        .rst               (~cmos_init_done[1]    ),
        .pclk              (cmos2_capture_clk    ),
        .cmos_vsync        (cmos2_pixel_vsync    ),
        .write_req         (ch1_write_req         ),
        .write_addr_index  (ch1_write_addr_index  ),
        .read_addr_index   (ch1_read_addr_index   ),
        .write_req_ack     (ch1_write_req_ack     )
    );

    frame_read_write #(
        .MEM_DATA_BITS      (256), .READ_DATA_BITS(16), .WRITE_DATA_BITS(16),
        .ADDR_BITS          (25 ), .BUSRT_BITS    (10), .BURST_SIZE     (16)
    ) frame_read_write_m1 (
        .rst                    (~ddr_init_done         ),
        .mem_clk                (core_clk               ),
        .rd_burst_req           (ch1_rd_burst_req       ),
        .rd_burst_len           (ch1_rd_burst_len       ),
        .rd_burst_addr          (ch1_rd_burst_addr      ),
        .rd_burst_data_valid    (ch1_rd_burst_data_valid),
        .rd_burst_data          (ch1_rd_burst_data      ),
        .rd_burst_finish        (ch1_rd_burst_finish    ),
        .read_clk               (pix_clk                ),
        .read_req               (ch1_read_req           ),
        .read_req_ack           (ch1_read_req_ack       ),
        .read_finish            (                       ),
        .read_addr_0            (CH1_ADDR_0)              ,
        .read_addr_1            (CH1_ADDR_1)              ,
        .read_addr_2            (CH1_ADDR_2)              ,
        .read_addr_3            (CH1_ADDR_3)              ,
        .read_addr_index        (ch1_read_addr_index    ),
        .read_len               (FRAME_LEN              ),
        .read_en                (ch1_read_en            ),
        .read_data              (ch1_read_data          ),
        .wr_burst_req           (ch1_wr_burst_req       ),
        .wr_burst_len           (ch1_wr_burst_len       ),
        .wr_burst_addr          (ch1_wr_burst_addr      ),
        .wr_burst_data_req      (ch1_wr_burst_data_req  ),
        .wr_burst_data          (ch1_wr_burst_data      ),
        .wr_burst_finish        (ch1_wr_burst_finish    ),
        .write_clk              (cmos2_capture_clk      ),
        .write_req              (ch1_write_req          ),
        .write_req_ack          (ch1_write_req_ack      ),
        .write_finish           (                       ),
        .write_addr_0           (CH1_ADDR_0)              ,
        .write_addr_1           (CH1_ADDR_1)              ,
        .write_addr_2           (CH1_ADDR_2)              ,
        .write_addr_3           (CH1_ADDR_3)              ,
        .write_addr_index       (ch1_write_addr_index   ),
        .write_len              (FRAME_LEN              ),
        .write_en               (cam1_valid               ),
        .write_data             (cam1_pixel              )
    );

    // ch2 保存原生 RGB565；单画面和四画面共用四个 DDR 帧槽。
    // 切离 HDMI 时也在显示帧边界释放读取占用。
    
    always @(posedge pix_clk or negedge pix_rstn) begin
        if (!pix_rstn) native_vs_d <= 0;
        else native_vs_d <= vs_o;
    end

    // Y/C 平面与 RGB565 快照读取同一 color_canvas。
    // 1920x1080 转为 640x360，先存 Y8 后存 C8，每槽 460800 字节。
    // ⚠ PLANE_ENABLE=0 时本节整体不进入网表（见参数区的说明）。关断分支只把
    //   会被 generate 外部引用到的信号拉成常量，保证 4 路基础通路完全独立。
    
generate
if (PLANE_ENABLE != 0) begin : g_plane
    wire plane_input_valid, plane_input_sof;
    wire [31:0] plane_input_sequence, plane_input_epoch, plane_input_config;
    wire [2:0] plane_input_layout;
    road_plane_capture_gate u_plane_capture_gate (
        .clk(pix_clk), .rst_n(dark_capture_reset_n), .enable(plane_enable),
        .period_div(plane_period_div), .boundary(feature_boundary),
        .pixel_valid(color_canvas_de && !native_canvas_invalid),
        .source_sequence(feature_sequence), .source_epoch(feature_epoch),
        .source_config(feature_capture_config), .source_layout(color_canvas_source),
        .valid(plane_input_valid), .sof(plane_input_sof),
        .frame_sequence(plane_input_sequence), .frame_epoch(plane_input_epoch),
        .frame_config(plane_input_config), .frame_layout(plane_input_layout)
    );
    road_lane_plane_canvas_1920 #(.IN_W(1920), .IN_H(1080), .OUT_W(640), .OUT_H(360)) u_road_lane_plane_canvas (
        .clk        (pix_clk                       ),
        .rst_n      (dark_capture_reset_n          ),
        .in_valid   (plane_input_valid),
        .in_pixel   (color_canvas_data             ),
        .in_sof     (plane_input_sof               ),
        .out_valid  (plane_gray_valid              ),
        .out_gray   (plane_gray                    ),
        .out_chroma (plane_chroma                  ),
        .out_x      (                             ),
        .out_y      (                             ),
        .out_sof    (plane_gray_sof                ),
        .out_eof    (                             ),
        .out_ready  (1'b1                          ),
        .stat_pixels(                             ),
        .stat_outputs(                            ),
        .stat_frames(                             ),
        .stat_dropped(                            )
    );

    road_yc_plane_store #(
        .ROW_WORDS(20), .PLANE_BEATS(ROAD_YC_PLANE_BEATS), .SLOT_COUNT(3)
    ) u_road_plane_store (
        .rst_n        (ddr_init_done               ),
        .pix_clk      (pix_clk                      ),
        .pix_valid    (plane_gray_valid && !native_canvas_invalid),
        .pix_y8       (plane_gray                  ),
        .pix_c8       (plane_chroma                ),
        .pix_sof      (plane_gray_sof              ),
        .pix_sequence (plane_input_sequence),
        .pix_epoch    (plane_input_epoch),
        .pix_config   (plane_input_config),
        .pix_layout   (plane_input_layout),
        .mem_clk      (core_clk                    ),
        .cfg_base     (plane_cfg_base              ),
        .wr_req       (plane_wr_req                ),
        .wr_len       (plane_wr_len                ),
        .wr_addr      (plane_wr_addr               ),
        .wr_data_req  (plane_wr_data_req           ),
        .wr_data      (plane_wr_data               ),
        .wr_data_valid(plane_wr_data_valid         ),
        .wr_finish    (plane_wr_finish             ),
        .wr_fault(plane_axi_wr_fault), .slot_locked(plane_slot_locked), .locked_slot(plane_locked_slot),
        .frame_done   (plane_frame_done            ),
        .frame_done_toggle(plane_frame_done_toggle ),
        .slot_index   (plane_slot_index            ),
        .frame_count  (plane_frame_count           ),
        .completed_sequence(plane_completed_sequence),
        .completed_epoch(plane_completed_epoch),
        .completed_config(plane_completed_config),
        .completed_layout(plane_completed_layout),
        .completed_base(plane_completed_base),
        .overflow     (plane_overflow              ),
        .overflow_row (plane_overflow_row          ),
        .src_lost     (plane_src_lost              )
    );

    // 平面帧读流: 由 road_plane_tx 每接受一帧发起一次 DDR 顺序读
    road_plane_reader #(.TOTAL_BEATS(ROAD_YC_SLOT_BEATS), .BURST_BEATS(10'd16)) u_road_plane_reader (
        .mem_clk      (core_clk                   ),
        .rst_n        (ddr_init_done              ),
        .abort        (~plane_mem_run_sync[2]      ),
        .busy         (plane_reader_busy          ),
        .start        (plane_rd_start             ),
        .cfg_base_beat(plane_reader_base          ),
        .cfg_slot_beats(ROAD_YC_SLOT_BEATS         ),
        .slot_idx     (2'd0                      ),
        .rd_req       (plane_rd_req               ),
        .rd_addr      (plane_rd_addr              ),
        .rd_valid     (plane_rd_data_req          ),
        .rd_data      (shared_rd_data             ),
        .rd_finish    (plane_rd_finish            ),
        .data         (plane_reader_data          ),
        .data_valid   (plane_reader_valid         ),
        .data_error   (plane_rd_stream_error      ),
        .rd_fault     (plane_axi_rd_fault         ),
        .data_ready   (plane_reader_ready         ),
        .done         (plane_rd_done              ),
        .bursts_done  (plane_reader_bursts        ),
        .error        (plane_rd_error             )
    );

    road_plane_cdc u_plane_cdc (
        .rst_n(ddr_init_done), .tx_rst_n(plane_reset_n), .mem_clk(core_clk), .tx_clk(video_tx_clk),
        .frame_done(plane_frame_done), .slot_index(plane_slot_index),
        .frame_seq(plane_completed_sequence), .frame_epoch(plane_completed_epoch),
        .frame_config(plane_completed_config), .frame_layout(plane_completed_layout),
        .cfg_base(plane_completed_base), .cfg_slot_beats(ROAD_YC_SLOT_BEATS),
        .slot_locked(plane_slot_locked), .locked_slot(plane_locked_slot), .skipped_frames(plane_cdc_skipped),
        .reader_start(plane_rd_start), .reader_base(plane_reader_base),
        .reader_data(plane_reader_data), .reader_valid(plane_reader_valid),
        .reader_error(plane_rd_stream_error), .reader_busy(plane_reader_busy), .reader_ready(plane_reader_ready),
        .frame_valid(plane_frame_pending), .tx_frame_seq(plane_tx_frame_seq),
        .tx_frame_epoch(plane_tx_frame_epoch), .tx_frame_config(plane_tx_frame_config),
        .tx_frame_layout(plane_tx_frame_layout),
        .frame_accept(plane_tx_frame_accept), .tx_frame_done(plane_tx_frame_done),
        .data(plane_tx_data), .data_valid(plane_tx_data_valid),
        .data_error(plane_tx_data_error), .data_ready(plane_tx_data_ready)
    );

end else begin : g_no_plane
    // 平面特性关闭：三个实例都不生成。下列信号被 generate 外部（写/读仲裁、
    // 控制寄存器组、PCIe 端点）引用，这里给常量，平面通道等效于永久空闲。
    assign plane_frame_count  = 32'd0;
    assign plane_completed_sequence = 32'd0;
    assign plane_completed_epoch = 32'd0;
    assign plane_completed_config = 32'd0;
    assign plane_completed_layout = 3'd0;
    assign plane_slot_index   = 2'd0;
    assign plane_frame_done   = 1'b0;
    assign plane_frame_done_toggle = 1'b0;
    assign plane_overflow     = 1'b0;
    assign plane_overflow_row = 8'd0;   // A-3 新增信号：关闭分支无驱动源，必须给常量
    assign plane_frame_pending=1'b0;
    assign plane_tx_frame_seq=32'd0;
    assign plane_tx_frame_epoch=32'd0;
    assign plane_tx_frame_config=32'd0;
    assign plane_tx_frame_layout=3'd0;
    assign plane_tx_data_error=1'b0;
    assign plane_rd_error=1'b0;
    assign plane_rd_done=1'b0;
    assign plane_reader_busy=1'b0;
    assign plane_cdc_skipped=32'd0;
    assign plane_rd_start=1'b0;
    assign plane_slot_locked=1'b0;
    assign plane_src_lost     = 1'b0;
    assign plane_wr_req       = 1'b0;
    assign plane_wr_len       = 10'd0;
    assign plane_wr_addr      = 25'd0;
    assign plane_wr_data      = 256'd0;
    assign plane_wr_data_valid= 1'b0;
    assign plane_completed_base = 25'd0;
    assign plane_reader_base  = 25'd0;
    assign plane_locked_slot  = 2'd0;
    assign plane_reader_bursts= 16'd0;
    assign plane_rd_req       = 1'b0;
    assign plane_rd_addr      = 25'd0;
    assign plane_tx_data      = 128'd0;
    assign plane_tx_data_valid= 1'b0;
end
endgenerate

    hdmi_native_source #(.BASE(HDMI_NATIVE_BASE)) u_hdmi_native (
        .rst_n(ddr_init_done), .in_clk(pixclk_in), .mem_clk(core_clk), .pix_clk(pix_clk),
        .in_vs(vs_in_d0), .in_de(de_in_d0),
        .in_pixel({r_in_d0[7:3],g_in_d0[7:2],b_in_d0[7:3]}),
        .frame_start(native_frame_start),
        .display_enable(display_sel_mode_r == 3'd2 || display_sel_mode_r == 3'd4),
        .quad_mode(quad_display_mode), .read_en(ch2_read_en), .read_data(ch2_read_data),
        .frame_valid(hdmi_native_frame_valid), .source_live(hdmi_native_source_live),
        .wr_req(ch2_wr_burst_req), .wr_len(ch2_wr_burst_len), .wr_addr(ch2_wr_burst_addr),
        .wr_data(ch2_wr_burst_data), .wr_data_req(ch2_wr_burst_data_req),
        .wr_finish(ch2_wr_burst_finish), .wr_fault(plane_axi_wr_fault),
        .rd_req(ch2_rd_burst_req), .rd_len(ch2_rd_burst_len), .rd_addr(ch2_rd_burst_addr),
        .rd_valid(ch2_rd_burst_data_valid), .rd_data(ch2_rd_burst_data),
        .rd_finish(ch2_rd_burst_finish), .rd_fault(plane_axi_rd_fault),
        .committed_frames(hdmi_native_committed), .rejected_frames(hdmi_native_rejected),
        .capture_rejected(hdmi_capture_rejected), .underflow_frames(hdmi_native_underflows),
        .missed_requests(hdmi_native_missed),
        .frame_boundary(hdmi_input_boundary),.source_sequence(hdmi_source_sequence),
        .source_epoch(hdmi_source_epoch),.source_timestamp_ticks(hdmi_source_timestamp),.native_identity(hdmi_pixel_identity)
        ,.native_left_check(hdmi_left_check),.native_left_counts(hdmi_left_counts)
    );
    // 旧显示模块保留请求端口；原生 HDMI 用 native_frame_start 管理占用，
    // 包含切离 HDMI 的帧。
    assign ch2_read_req_ack = ch2_read_req;

    // ch3 接收以太网桌面图像；保留读通路，关闭旧 HDMI 写通路。
    eth_ch3_input u_eth_ch3_input (
        .core_clk         (core_clk         ),
        .rst_n            (ddr_init_done    ),
        .rgmii_rxc        (rgmii_rxc        ),
        .rgmii_rx_ctl     (rgmii_rx_ctl     ),
        .rgmii_rxd        (rgmii_rxd        ),
        .rgmii_txc        (rgmii_txc        ),
        .rgmii_tx_ctl     (rgmii_tx_ctl     ),
        .rgmii_txd        (rgmii_txd        ),
        .read_req_ack     (ch3_read_req_ack ),
        .read_index       (ch3_read_addr_index),
        .display_valid    (eth_display_valid),
        .wr_burst_req     (ch3_wr_burst_req ),
        .len              (ch3_wr_burst_len ),
        .addr             (ch3_wr_burst_addr),
        .data             (ch3_wr_burst_data),
        .wr_burst_data_req(ch3_wr_burst_data_req),
        .wr_burst_finish  (ch3_wr_burst_finish)
    );

    frame_read_write #(
        .MEM_DATA_BITS      (256), .READ_DATA_BITS(16), .WRITE_DATA_BITS(16),
        .ADDR_BITS          (25 ), .BUSRT_BITS    (10), .BURST_SIZE     (16)
    ) frame_read_write_m3 (
        .rst                    (~ddr_init_done         ),
        .mem_clk                (core_clk               ),
        .rd_burst_req           (ch3_rd_burst_req       ),
        .rd_burst_len           (ch3_rd_burst_len       ),
        .rd_burst_addr          (ch3_rd_burst_addr      ),
        .rd_burst_data_valid    (ch3_rd_burst_data_valid),
        .rd_burst_data          (ch3_rd_burst_data      ),
        .rd_burst_finish        (ch3_rd_burst_finish    ),
        .read_clk               (pix_clk                ),
        .read_req               (ch3_read_req           ),
        .read_req_ack           (ch3_read_req_ack       ),
        .read_finish            (                       ),
        .read_addr_0            (CH3_ADDR_0)              ,
        .read_addr_1            (CH3_ADDR_1)              ,
        .read_addr_2            (CH3_ADDR_2)              ,
        .read_addr_3            (CH3_ADDR_3)              ,
        .read_addr_index        (ch3_read_addr_index    ),
        .read_len               (FRAME_LEN              ),
        .read_en                (ch3_read_en            ),
        .read_data              (ch3_read_data_raw      ),
        .wr_burst_req           (                       ),
        .wr_burst_len           (                       ),
        .wr_burst_addr          (                       ),
        .wr_burst_data_req      (1'b0                   ),
        .wr_burst_data          (                       ),
        .wr_burst_finish        (1'b0                   ),
        .write_clk              (core_clk               ),
        .write_req              (1'b0                   ),
        .write_req_ack          (                       ),
        .write_finish           (                       ),
        .write_addr_0           (CH3_ADDR_0)              ,
        .write_addr_1           (CH3_ADDR_1)              ,
        .write_addr_2           (CH3_ADDR_2)              ,
        .write_addr_3           (CH3_ADDR_3)              ,
        .write_addr_index       (2'd0                   ),
        .write_len              (FRAME_LEN              ),
        .write_en               (1'b0                   ),
        .write_data             (16'd0                  )
    );

    // 四路写仲裁：ch0/1 为摄像头，ch2 为原生 HDMI，ch3 为以太网。
    mem_write_arbi #(.MEM_DATA_BITS(256), .ADDR_BITS(25), .BUSRT_BITS(10)) u_mem_write_arbi (
        .rst_n                  (ddr_init_done            ),
        .mem_clk                (core_clk                 ),
        .ch0_wr_burst_req       (ch0_wr_burst_req         ),
        .ch0_wr_burst_len       (ch0_wr_burst_len         ),
        .ch0_wr_burst_addr      (ch0_wr_burst_addr        ),
        .ch0_wr_burst_data_req  (ch0_wr_burst_data_req    ),
        .ch0_wr_burst_data      (ch0_wr_burst_data        ),
        .ch0_wr_burst_finish    (ch0_wr_burst_finish      ),
        .ch1_wr_burst_req       (ch1_wr_burst_req         ),
        .ch1_wr_burst_len       (ch1_wr_burst_len         ),
        .ch1_wr_burst_addr      (ch1_wr_burst_addr        ),
        .ch1_wr_burst_data_req  (ch1_wr_burst_data_req    ),
        .ch1_wr_burst_data      (ch1_wr_burst_data        ),
        .ch1_wr_burst_finish    (ch1_wr_burst_finish      ),
        .ch2_wr_burst_req       (ch2_wr_burst_req         ),
        .ch2_wr_burst_len       (ch2_wr_burst_len         ),
        .ch2_wr_burst_addr      (ch2_wr_burst_addr        ),
        .ch2_wr_burst_data_req  (ch2_wr_burst_data_req    ),
        .ch2_wr_burst_data      (ch2_wr_burst_data        ),
        .ch2_wr_burst_finish    (ch2_wr_burst_finish      ),
        .ch3_wr_burst_req       (ch3_wr_burst_req         ),
        .ch3_wr_burst_len       (ch3_wr_burst_len         ),
        .ch3_wr_burst_addr      (ch3_wr_burst_addr        ),
        .ch3_wr_burst_data_req  (ch3_wr_burst_data_req    ),
        .ch3_wr_burst_data      (ch3_wr_burst_data        ),
        .ch3_wr_burst_finish    (ch3_wr_burst_finish      ),
        // 采集门控只停止新帧；即使软件在帧中途关闭采集，
        // 已接受的写事务也必须排空。
        .ch4_wr_burst_req       (plane_wr_req              ),
        .ch4_wr_burst_len       (plane_wr_len             ),
        .ch4_wr_burst_addr      (plane_wr_addr            ),
        .ch4_wr_burst_data_req  (plane_wr_data_req        ),
        .ch4_wr_burst_data      (plane_wr_data            ),
        .ch4_wr_burst_finish    (plane_wr_finish          ),
        .wr_burst_req           (wr_burst_req            ),
        .wr_burst_len           (wr_burst_len            ),
        .wr_burst_addr          (wr_burst_addr           ),
        .wr_burst_data_req      (wr_burst_data_req       ),
        .wr_burst_data          (wr_burst_data           ),
        .wr_burst_finish        (wr_burst_finish         )
    );

    // ---- 4 路读仲裁 ----
    mem_read_arbi #(.MEM_DATA_BITS(256), .ADDR_BITS(25), .BUSRT_BITS(10)) u_mem_read_arbi (
        .rst_n                  (ddr_init_done            ),
        .mem_clk                (core_clk                 ),
        .ch0_rd_burst_req       (ch0_rd_burst_req         ),
        .ch0_rd_burst_len       (ch0_rd_burst_len         ),
        .ch0_rd_burst_addr      (ch0_rd_burst_addr        ),
        .ch0_rd_burst_data_valid(ch0_rd_burst_data_valid  ),
        .ch0_rd_burst_data      (ch0_rd_burst_data        ),
        .ch0_rd_burst_finish    (ch0_rd_burst_finish      ),
        .ch1_rd_burst_req       (ch1_rd_burst_req         ),
        .ch1_rd_burst_len       (ch1_rd_burst_len         ),
        .ch1_rd_burst_addr      (ch1_rd_burst_addr        ),
        .ch1_rd_burst_data_valid(ch1_rd_burst_data_valid  ),
        .ch1_rd_burst_data      (ch1_rd_burst_data        ),
        .ch1_rd_burst_finish    (ch1_rd_burst_finish      ),
        .ch2_rd_burst_req       (ch2_rd_burst_req         ),
        .ch2_rd_burst_len       (ch2_rd_burst_len         ),
        .ch2_rd_burst_addr      (ch2_rd_burst_addr        ),
        .ch2_rd_burst_data_valid(ch2_rd_burst_data_valid  ),
        .ch2_rd_burst_data      (ch2_rd_burst_data        ),
        .ch2_rd_burst_finish    (ch2_rd_burst_finish      ),
        .ch3_rd_burst_req       (ch3_rd_burst_req         ),
        .ch3_rd_burst_len       (ch3_rd_burst_len         ),
        .ch3_rd_burst_addr      (ch3_rd_burst_addr        ),
        .ch3_rd_burst_data_valid(ch3_rd_burst_data_valid  ),
        .ch3_rd_burst_data      (ch3_rd_burst_data        ),
        .ch3_rd_burst_finish    (ch3_rd_burst_finish      ),
        .rd_burst_req           (rd_burst_req            ),
        .rd_burst_len           (rd_burst_len            ),
        .rd_burst_addr          (rd_burst_addr           ),
        .rd_burst_data_valid    (rd_burst_data_valid     ),
        .rd_burst_data          (rd_burst_data           ),
        .rd_burst_finish        (rd_burst_finish         )
    );

    // 端点短复位后，先排空旧像素和版本信息，再恢复采集及帧关联。
    // 本地 HDMI 流水线保持运行。
    wire video_frame_valid, video_frame_accept, video_frame_done;
    wire [31:0] video_frame_seq, video_captured, video_dropped;
    wire [2:0] video_frame_mode;
    wire video_data_valid, video_data_ready;
    wire [127:0] video_dma_data;
    wire video_wr_req, video_wr_data_req, video_wr_finish;
    wire video_rd_req, video_rd_data_valid, video_rd_finish;
    wire [24:0] video_wr_addr, video_rd_addr;
    wire [255:0] video_wr_data;
    // 暗光输出供完整视频 DMA、YOLO 小图和本地显示使用；关闭时逐像素旁路。
    // 道路检测另取原始 HDMI。PCIe 复位不停止本地画布；硬件复位后默认关闭增强。
    dark_enhance_canvas_pipeline u_dark_canvas (
        .rst_n(pix_rstn), .pix_clk(pix_clk),
        .timing_hs(disp_hs), .timing_vs(disp_vs), .timing_de(disp_de),
        .pixel_in(disp_vout_data), .source_in(display_sel_mode_r),
        .sample_enable(dark_sample_enable), .sample_threshold(dark_sample_threshold),
        .sample_strength(dark_sample_strength), .sample_generation(dark_sample_generation),
        .sample_request(dark_sample_request), .control_boundary(dark_control_boundary),
        .timing_hs_out(color_canvas_hs), .timing_vs_out(color_canvas_vs),
        .timing_de_out(color_canvas_de), .pixel_out(color_canvas_data),
        .source_out(color_canvas_source), .image_generation_out(color_canvas_generation),
        .image_request_out(color_canvas_request)
    );
    // KEY4 模式与对应输入像素一起延迟 13 拍。
    always @(posedge pix_clk or negedge pix_rstn) begin
        if (!pix_rstn) begin
            for (process_stage=0; process_stage<DARK_CANVAS_STAGES; process_stage=process_stage+1)
                process_mode_pipe[process_stage] <= 0;
        end else begin
            process_mode_pipe[0] <= display_process_mode_r;
            for (process_stage=1; process_stage<DARK_CANVAS_STAGES; process_stage=process_stage+1)
                process_mode_pipe[process_stage] <= process_mode_pipe[process_stage-1];
        end
    end
    display_process u_display_process (
        .clk(pix_clk), .rst(~pix_rstn), .process_mode(process_mode_pipe[DARK_CANVAS_STAGES-1]),
        .quad_mode(color_canvas_source==3'd4), .timing_hs(color_canvas_hs), .timing_vs(color_canvas_vs),
        .timing_de(color_canvas_de), .color_canvas_data(color_canvas_data),
        .display_hs(display_canvas_hs), .display_vs(display_canvas_vs),
        .display_de(display_canvas_de), .display_canvas_data(display_canvas_data)
    );
    video_dma_frame_store #(.TRACK_IMAGE_GENERATION(1),.CHECK_PIX_INVALID(1),.TRACK_NATIVE_IDENTITY(1)) u_video_frame_store (
        .rst_n(dark_capture_reset_n),
        .pix_clk(pix_clk), .pix_vs(color_canvas_vs), .pix_de(color_canvas_de),
        .pix_data(color_canvas_data), .pix_mode(color_canvas_source),
        .pix_invalid(native_canvas_invalid),
        .pix_config_generation(feature_capture_config),
        .pix_image_generation(color_canvas_generation),
        .capture_boundary(feature_boundary),.capture_begin(feature_begin),.capture_end(feature_end),
        .capture_invalid(feature_invalid),.capture_sequence(feature_sequence),.capture_epoch(feature_epoch),
        .mem_clk(core_clk), .wr_req(video_wr_req), .wr_addr(video_wr_addr),
        .wr_data_req(video_wr_data_req), .wr_data(video_wr_data), .wr_finish(video_wr_finish),
        .rd_req(video_rd_req), .rd_addr(video_rd_addr), .rd_valid(video_rd_data_valid),
        .rd_data(shared_rd_data), .rd_finish(video_rd_finish),
        .tx_clk(video_tx_clk), .run(video_run), .source_idle(video_source_idle),
        .frame_valid(video_frame_valid), .frame_seq(video_frame_seq), .frame_mode(video_frame_mode),
        .frame_epoch(video_frame_epoch),.frame_config_generation(video_frame_config),
        .pix_native_identity((color_canvas_source == 3'd4 || color_canvas_source == 3'd2) ? hdmi_pixel_identity : 128'd0),
        .frame_native_identity(video_frame_native_identity),
        .frame_accept(video_frame_accept), .frame_done(video_frame_done),
        .data_valid(video_data_valid), .data(video_dma_data), .data_ready(video_data_ready),
        .captured(video_captured), .dropped(video_dropped)
    );
    // 平面溢出行号从 pix_clk 域两拍采样到 video_tx_clk，仅用于诊断。
    // 多位采样不保证原子一致，不能据此控制传输。
    reg [7:0] plane_overflow_row_d0, plane_overflow_row_d1;
    always @(posedge video_tx_clk or negedge dark_capture_reset_n) begin
        if (!dark_capture_reset_n) begin
            plane_overflow_row_d0 <= 8'd0;
            plane_overflow_row_d1 <= 8'd0;
        end else begin
            plane_overflow_row_d0 <= plane_overflow_row;
            plane_overflow_row_d1 <= plane_overflow_row_d0;
        end
    end
    assign plane_overflow_row_sync = plane_overflow_row_d1;

    // 诊断计数器寄存为格雷码后跨域；
    // 传输控制不依赖这些采样状态。
    reg [31:0] plane_read_burst_total, plane_read_error_total;
    reg [31:0] plane_accept_total, plane_finish_total;
    wire [31:0] plane_store_count_sync, plane_read_count_sync, plane_error_count_sync;
    (* ASYNC_REG = "TRUE" *) reg [4:0] plane_flags_s1, plane_flags_s2;
    always @(posedge core_clk or negedge plane_reset_n) begin
        if(!plane_reset_n) begin plane_read_burst_total<=0; plane_read_error_total<=0; end
        else begin
            if(plane_rd_finish) plane_read_burst_total<=plane_read_burst_total+1'b1;
            if(plane_rd_done && plane_rd_error) plane_read_error_total<=plane_read_error_total+1'b1;
        end
    end
    always @(posedge video_tx_clk or negedge plane_reset_n) begin
        if(!plane_reset_n) begin
            plane_accept_total<=0; plane_finish_total<=0; plane_flags_s1<=0; plane_flags_s2<=0;
        end else begin
            if(plane_tx_frame_accept) plane_accept_total<=plane_accept_total+1'b1;
            if(plane_tx_frame_done) plane_finish_total<=plane_finish_total+1'b1;
            plane_flags_s1<={plane_rd_req,plane_slot_locked,plane_rd_error,plane_axi_wr_fault,plane_axi_rd_fault};
            plane_flags_s2<=plane_flags_s1;
        end
    end
    road_plane_counter_cdc u_plane_skipped_count (.rst_n(plane_reset_n),.src_clk(core_clk),
        .dst_clk(video_tx_clk),.count_in(plane_cdc_skipped),.count_out(plane_skipped_sync));
    road_plane_counter_cdc u_plane_store_count (.rst_n(plane_reset_n),.src_clk(core_clk),
        .dst_clk(video_tx_clk),.count_in(plane_frame_count),.count_out(plane_store_count_sync));
    road_plane_counter_cdc u_plane_read_count (.rst_n(plane_reset_n),.src_clk(core_clk),
        .dst_clk(video_tx_clk),.count_in(plane_read_burst_total),.count_out(plane_read_count_sync));
    road_plane_counter_cdc u_plane_error_count (.rst_n(plane_reset_n),.src_clk(core_clk),
        .dst_clk(video_tx_clk),.count_in(plane_read_error_total),.count_out(plane_error_count_sync));
    wire [191:0] plane_diagnostics={
        24'd0,plane_tx_data_ready,plane_tx_data_valid,plane_frame_pending,plane_flags_s2,
        plane_finish_total,plane_accept_total,plane_error_count_sync,plane_read_count_sync,plane_store_count_sync};

    wire [127:0] native_status_data, legacy_feature_read_data;
    // 保留退役源控制窗口；仅 0x17d..0x17f 的高位字提供原生诊断，
    // 低位字返回零。
    wire native_status_read = feature_read_addr >= 32'h177 && feature_read_addr <= 32'h17f;
    reg native_status_selected, rgb_read_selected,ref_read_selected;
    always @(posedge video_tx_clk or negedge dark_capture_reset_n)
        if (!dark_capture_reset_n) native_status_selected <= 0;
        else if (feature_read_en) native_status_selected <= native_status_read;
    always @(posedge video_tx_clk or negedge dark_capture_reset_n)
        if (!dark_capture_reset_n) rgb_read_selected <= 0;
        else if (feature_read_en) rgb_read_selected <= rgb_read;
    always @(posedge video_tx_clk or negedge dark_capture_reset_n)
        if (!dark_capture_reset_n) ref_read_selected <= 0;
        else if (feature_read_en) ref_read_selected <= ref_read;
    assign feature_read_data = ref_read_selected ? ref_read_data : rgb_read_selected ? rgb_read_data :
        native_status_selected ? native_status_data : legacy_feature_read_data;
    hdmi_native_status u_hdmi_native_status (
        .rst_n(dark_capture_reset_n), .in_clk(pixclk_in), .mem_clk(core_clk),
        .pix_clk(pix_clk), .bus_clk(video_tx_clk),
        .capture_rejected(hdmi_capture_rejected), .committed(hdmi_native_committed),
        .rejected(hdmi_native_rejected), .underflows(hdmi_native_underflows), .missed(hdmi_native_missed),
        .frame_valid(hdmi_native_frame_valid), .source_live(hdmi_native_source_live),
        .read_en(feature_read_en && native_status_read), .read_addr(feature_read_addr), .read_data(native_status_data)
    );
    feature_control_mux #(.ENABLE_PLANE(0)) u_feature_transport (
        .rst_n(dark_capture_reset_n),.pix_clk(pix_clk),.bus_clk(video_tx_clk),.run(video_run),
        .control_boundary(dark_control_boundary),
        .capture_boundary(feature_boundary),.capture_begin(feature_begin),.capture_end(feature_end),
        .capture_invalid(feature_invalid),.capture_sequence(feature_sequence),.capture_epoch(feature_epoch),
        .capture_source(color_canvas_source),.pixel_de(color_canvas_de),.pixel_rgb565(color_canvas_data),
        .image_generation(color_canvas_generation),.image_request(color_canvas_request),
        .sample_enable(dark_sample_enable),.sample_threshold(dark_sample_threshold),
        .sample_strength(dark_sample_strength),.sample_generation(dark_sample_generation),
        .sample_request(dark_sample_request),
        .capture_config_generation(feature_capture_config),.reg_wr(feature_reg_wr),.reg_addr(feature_reg_addr),
        .reg_data(feature_reg_data),.read_en(feature_read_en),.read_addr(feature_read_addr),.read_data(legacy_feature_read_data),
        .plane_enable(plane_enable),.plane_period_div(plane_period_div),
        .plane_base(plane_cfg_base),.plane_slot_beats(plane_cfg_slot_beats),.plane_slot_count(plane_cfg_slot_count),
        .plane_geom_w(plane_geom_w),.plane_geom_h(plane_geom_h),.plane_format(plane_format),
        .plane_host_addr(plane_host_addr),.plane_completion(plane_completion),
        .plane_consumer(plane_consumer), .plane_diagnostics(plane_diagnostics),
        // 0x129 返回 TX 已提交帧数，0x13a 单独返回 DDR 完成帧数。
        .plane_frame_count(plane_producer),.plane_color_gen(color_canvas_generation),
        .plane_slot_index(plane_slot_index),.plane_busy(plane_wr_req),
        .plane_overflow(plane_overflow),.plane_src_lost(plane_src_lost),
        .plane_overflow_row(plane_overflow_row_sync),.plane_dropped(plane_dropped_frames+plane_skipped_sync)
    );
    wire aux_wr_req,aux_wr_data_req,aux_wr_finish;
    wire [9:0] aux_wr_len;
    wire [24:0] aux_wr_addr;
    wire [255:0] aux_wr_data;
    video_dma_burst_mux u_aux_write_mux (
        .clk(core_clk),.rst_n(ddr_init_done),
        .base_req(rgb_wr_req),.base_len(rgb_wr_len),.base_addr(rgb_wr_addr),.base_data(rgb_wr_data),
        .base_data_req(rgb_wr_data_req),.base_finish(rgb_wr_finish),
        .video_req(1'b0),.video_addr(25'd0),.video_data(256'd0),.video_data_req(),.video_finish(),
        .plane_req(ref_wr_req),.plane_len(ref_wr_len),.plane_addr(ref_wr_addr),.plane_data(ref_wr_data),
        .plane_data_req(ref_wr_data_req),.plane_finish(ref_wr_finish),
        .req(aux_wr_req),.len(aux_wr_len),.addr(aux_wr_addr),.data(aux_wr_data),
        .data_req(aux_wr_data_req),.finish(aux_wr_finish)
    );
    video_dma_burst_mux u_video_write_mux (
        .clk(core_clk), .rst_n(ddr_init_done),
        .base_req(wr_burst_req), .base_len(wr_burst_len), .base_addr(wr_burst_addr),
        .base_data(wr_burst_data), .base_data_req(wr_burst_data_req), .base_finish(wr_burst_finish),
        .video_req(video_wr_req), .video_addr(video_wr_addr), .video_data(video_wr_data),
        .video_data_req(video_wr_data_req), .video_finish(video_wr_finish),
        .plane_req(aux_wr_req), .plane_len(aux_wr_len), .plane_addr(aux_wr_addr), .plane_data(aux_wr_data),
        .plane_data_req(aux_wr_data_req),.plane_finish(aux_wr_finish),
        .req(shared_wr_req), .len(shared_wr_len), .addr(shared_wr_addr), .data(shared_wr_data),
        .data_req(shared_wr_data_req), .finish(shared_wr_finish)
    );
    video_dma_burst_mux u_small_read_mux (
        .clk(core_clk),.rst_n(ddr_init_done),
        .base_req(plane_rd_req),.base_len(10'd16),.base_addr(plane_rd_addr),.base_data(256'd0),
        .base_data_req(plane_rd_data_req),.base_finish(plane_rd_finish),
        .video_req(rgb_rd_req),.video_addr(rgb_rd_addr),.video_data(256'd0),
        .video_data_req(rgb_rd_valid),.video_finish(rgb_rd_finish),
        .plane_req(ref_rd_req),.plane_len(10'd16),.plane_addr(ref_rd_addr),.plane_data(256'd0),
        .plane_data_req(ref_rd_valid),.plane_finish(ref_rd_finish),
        .req(small_rd_req),.len(small_rd_len),.addr(small_rd_addr),.data(),
        .data_req(small_rd_valid),.finish(small_rd_finish)
    );
    video_dma_burst_mux u_video_read_mux (
        .clk(core_clk), .rst_n(ddr_init_done),
        .base_req(rd_burst_req), .base_len(rd_burst_len), .base_addr(rd_burst_addr),
        .base_data(256'd0), .base_data_req(rd_burst_data_valid), .base_finish(rd_burst_finish),
        .video_req(video_rd_req), .video_addr(video_rd_addr), .video_data(256'd0),
        .video_data_req(video_rd_data_valid), .video_finish(video_rd_finish),
        .plane_req(small_rd_req), .plane_len(small_rd_len), .plane_addr(small_rd_addr),
        .plane_data(256'd0), .plane_data_req(small_rd_valid), .plane_finish(small_rd_finish),
        .req(shared_rd_req), .len(shared_rd_len), .addr(shared_rd_addr), .data(),
        .data_req(shared_rd_data_valid), .finish(shared_rd_finish)
    );

    // ---- AXI 主控（唯一）：作为 ddr3_test 的唯一主设备 ----
    aq_axi_master_256 u_aq_axi_master (
        .ARESETN            (ddr_init_done   ),
        .ACLK               (core_clk        ),
        .RD_FAULT(plane_axi_rd_fault), .WR_FAULT(plane_axi_wr_fault),
        .M_AXI_AWID         (m_axi_awid      ),
        .M_AXI_AWADDR       (m_axi_awaddr    ),
        .M_AXI_AWLEN        (m_axi_awlen     ),
        .M_AXI_AWSIZE       (                ),
        .M_AXI_AWBURST      (                ),
        .M_AXI_AWLOCK       (                ),
        .M_AXI_AWCACHE      (                ),
        .M_AXI_AWPROT       (                ),
        .M_AXI_AWQOS        (                ),
        .M_AXI_AWUSER       (                ),
        .M_AXI_AWVALID      (m_axi_awvalid   ),
        .M_AXI_AWREADY      (m_axi_awready   ),
        .M_AXI_WDATA        (m_axi_wdata     ),
        .M_AXI_WSTRB        (m_axi_wstrb     ),
        .M_AXI_WLAST        (                ),
        .M_AXI_WUSER        (                ),
        .M_AXI_WVALID       (m_axi_wvalid    ),
        .M_AXI_WREADY       (m_axi_wready    ),
        .M_AXI_BID          (1'b0            ),
        .M_AXI_BRESP        (2'b00           ),
        .M_AXI_BUSER        (1'b0            ),
        .M_AXI_BVALID       (1'b1            ),   // ddr3_test 无写响应通道，与 50K 工程一致
        .M_AXI_BREADY       (m_axi_bready    ),
        .M_AXI_ARID         (m_axi_arid      ),
        .M_AXI_ARADDR       (m_axi_araddr    ),
        .M_AXI_ARLEN        (m_axi_arlen     ),
        .M_AXI_ARSIZE       (                ),
        .M_AXI_ARBURST      (                ),
        .M_AXI_ARLOCK       (                ),
        .M_AXI_ARCACHE      (                ),
        .M_AXI_ARPROT       (                ),
        .M_AXI_ARQOS        (                ),
        .M_AXI_ARUSER       (                ),
        .M_AXI_ARVALID      (m_axi_arvalid   ),
        .M_AXI_ARREADY      (m_axi_arready   ),
        .M_AXI_RID          (m_axi_rid       ),
        .M_AXI_RDATA        (m_axi_rdata     ),
        .M_AXI_RRESP        (                ),
        // RLAST 驱动读状态机结束突发并产生 shared_rd_finish，必须连接。
        // 历史悬空使结束状态不可达、读链被优化，表现为同步正常但画面全黑。
        .M_AXI_RLAST        (axi_rlast      ),
        .M_AXI_RUSER        (                ),
        .M_AXI_RVALID       (m_axi_rvalid    ),
        .M_AXI_RREADY       (m_axi_rready    ),
        .MASTER_RST         (1'b0            ),
        .WR_START           (shared_wr_req        ),
        .WR_ADRS            ({shared_wr_addr,5'd0}),
        .WR_LEN             ({shared_wr_len ,5'd0}),
        .WR_READY           (                    ),
        .WR_FIFO_RE         (shared_wr_data_req   ),
        .WR_FIFO_EMPTY      (1'b0                ),
        .WR_FIFO_AEMPTY     (1'b0                ),
        .WR_FIFO_DATA       (shared_wr_data       ),
        .WR_DONE            (shared_wr_finish     ),
        .RD_START           (shared_rd_req        ),
        .RD_ADRS            ({shared_rd_addr,5'd0}),
        .RD_LEN             ({shared_rd_len ,5'd0}),
        .RD_READY           (                    ),
        .RD_FIFO_WE         (shared_rd_data_valid ),
        .RD_FIFO_FULL       (1'b0                ),
        .RD_FIFO_AFULL      (1'b0                ),
        .RD_FIFO_DATA       (shared_rd_data       ),
        .RD_DONE            (shared_rd_finish     ),
        .DEBUG              (                    )
    );

    // AXI 主控与 DDR IP 之间的地址和数据位宽适配。
    assign axi_awaddr    = m_axi_awaddr[CTRL_ADDR_WIDTH-1:0];  // 32 -> 28
    assign axi_awlen     = m_axi_awlen[3:0];                   //  8 -> 4
    assign axi_awuser_id = {3'b0, m_axi_awid[0]};              //  1 -> 4
    assign axi_awvalid   = m_axi_awvalid;
    assign m_axi_awready = axi_awready;
    assign axi_wdata     = m_axi_wdata;
    assign axi_wstrb     = m_axi_wstrb;
    assign m_axi_wready  = axi_wready;
    assign m_axi_bready  = 1'b1;
    assign axi_araddr    = m_axi_araddr[CTRL_ADDR_WIDTH-1:0];  // 32 -> 28
    assign axi_arlen     = m_axi_arlen[3:0];                   //  8 -> 4
    assign axi_aruser_id = {3'b0, m_axi_arid[0]};              //  1 -> 4
    assign axi_arvalid   = m_axi_arvalid;
    assign m_axi_arready = axi_arready;
    assign m_axi_rdata   = axi_rdata;
    assign m_axi_rid     = {3'b0, axi_rid[0]};
    assign m_axi_rvalid  = axi_rvalid;
    // axi_rlast 已直接接到 u_aq_axi_master 的 .M_AXI_RLAST，不需要再走中转线

    // 单画面、四画面显示及模式选择。
    //   display_sel_mode_r = 0 -> 单路 CMOS1(ch0)     1 -> 单路 CMOS2(ch1)
    //                      = 2 -> HDMI(ch2)          3 -> ETH(ch3)
    //                      = 4 -> 四画面(ch0/ch1/ch2/ch3)
    //   两套模块都在 pix_clk 域、共用 sync_vg 的时序；只有被选中的那套的
    //   read_req/read_en 会放行到帧缓存，未选中模块的请求不参与读取。
    
    assign quad_display_mode = (display_sel_mode_r == 3'd4);

    assign ch0_read_req = quad_display_mode ? quad_ch0_read_req : single_ch0_read_req;
    assign ch0_read_en  = quad_display_mode ? quad_ch0_read_en  : single_ch0_read_en;
    assign ch1_read_req = quad_display_mode ? quad_ch1_read_req : single_ch1_read_req;
    assign ch1_read_en  = quad_display_mode ? quad_ch1_read_en  : single_ch1_read_en;
    assign ch2_read_req = quad_display_mode ? quad_ch2_read_req : single_ch2_read_req;
    assign ch2_read_en  = quad_display_mode ? quad_ch2_read_en  : single_ch2_read_en;
    assign ch3_read_req = quad_display_mode ? quad_ch3_read_req : single_ch3_read_req;
    assign ch3_read_en  = quad_display_mode ? quad_ch3_read_en  : single_ch3_read_en;

    assign disp_hs        = quad_display_mode ? quad_hs        : single_hs;
    assign disp_vs        = quad_display_mode ? quad_vs        : single_vs;
    assign disp_de        = quad_display_mode ? quad_de        : single_de;
    assign disp_vout_data = quad_display_mode ? quad_vout_data : single_vout_data;

    single_channel_fullres_display u_single_channel_fullres_display (
        .display_enable    (~quad_display_mode  ),
        .video_clk          (pix_clk             ),
        .rst                (~pix_rstn           ),
        .i_display_sel_mode (display_sel_mode_r  ),
        .timing_hs          (hs_o                ),
        .timing_vs          (vs_o                ),
        .timing_de          (de_re               ),
        .ch0_read_req       (single_ch0_read_req ),
        .ch0_read_req_ack   (ch0_read_req_ack    ),
        .ch0_read_en        (single_ch0_read_en  ),
        .ch0_read_data      (ch0_read_data     ),
        .ch1_read_req       (single_ch1_read_req ),
        .ch1_read_req_ack   (ch1_read_req_ack    ),
        .ch1_read_en        (single_ch1_read_en  ),
        .ch1_read_data      (ch1_read_data     ),
        .ch2_read_req       (single_ch2_read_req ),
        .ch2_read_req_ack   (ch2_read_req_ack    ),
        .ch2_read_en        (single_ch2_read_en  ),
        .ch2_read_data      (ch2_read_data     ),
        .ch3_read_req       (single_ch3_read_req ),
        .ch3_read_req_ack   (ch3_read_req_ack    ),
        .ch3_read_en        (single_ch3_read_en  ),
        .ch3_read_data      (ch3_read_data     ),
        .o_display_sel      (                    ),
        .hs                 (single_hs           ),
        .vs                 (single_vs           ),
        .de                 (single_de           ),
        .vout_data          (single_vout_data    )
    );

    // 四画面：2x2 各 960x540，正好铺满 1920x1080
    // （帧缓存尺寸与窗口尺寸一致，1:1 不缩放，与 JC_pds 的参数相同）
    quad_channel_display u_quad_channel_display (
        .display_enable    (quad_display_mode   ),
        .video_clk          (pix_clk             ),
        .rst                (~pix_rstn           ),
        .timing_hs          (hs_o                ),
        .timing_vs          (vs_o                ),
        .timing_de          (de_re               ),
        .ch0_read_req       (quad_ch0_read_req   ),
        .ch0_read_req_ack   (ch0_read_req_ack    ),
        .ch0_read_en        (quad_ch0_read_en    ),
        .ch0_read_data      (ch0_read_data     ),
        .ch1_read_req       (quad_ch1_read_req   ),
        .ch1_read_req_ack   (ch1_read_req_ack    ),
        .ch1_read_en        (quad_ch1_read_en    ),
        .ch1_read_data      (ch1_read_data     ),
        .ch2_read_req       (quad_ch2_read_req   ),
        .ch2_read_req_ack   (ch2_read_req_ack    ),
        .ch2_read_en        (quad_ch2_read_en    ),
        .ch2_read_data      (ch2_read_data     ),
        .ch3_read_req       (quad_ch3_read_req   ),
        .ch3_read_req_ack   (ch3_read_req_ack    ),
        .ch3_read_en        (quad_ch3_read_en    ),
        .ch3_read_data      (ch3_read_data     ),
        .hs                 (quad_hs             ),
        .vs                 (quad_vs             ),
        .de                 (quad_de             ),
        .vout_data          (quad_vout_data      )
    );

    always @(posedge pix_clk or negedge pix_rstn) begin
        if (!pix_rstn) begin
            r_out <= 8'd0;
            g_out <= 8'd0;
            b_out <= 8'd0;
            vs_out <= 1'b0;
            hs_out <= 1'b0;
            de_out <= 1'b0;
        end else begin
            r_out <= {display_canvas_data[15:11], 3'b0};
            g_out <= {display_canvas_data[10:5], 2'b0};
            b_out <= {display_canvas_data[4:0], 3'b0};
            vs_out <= display_canvas_vs;
            hs_out <= display_canvas_hs;
            de_out <= display_canvas_de;
        end
    end

//产生visa时序 
     sync_vg sync_vg(                            
        .clk            (  pix_clk              ),
        .rstn           (  pix_rstn                  ),
        .vs_out         (  vs_o                 ),
        .hs_out         (  hs_o                 ),
        .de_out         (  de_re                     ),
        .de_re          (                  )    
    );  

// DDR 接口
/*        ddr3_100h ddr3_100h (
             .ref_clk                   (sys_clk            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output
             .ddrphy_clkin              (core_clk           ),// output
             .pll_lock                  (pll_lock           ),// output

             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug
             .debug_data                (                   ),// output [135:0]
             .debug_slice_state         (                   ),// output [51:0]
             .debug_calib_ctrl          (                   ),// output [21:0]
             .ck_dly_set_bin            (                   ),// output [7:0]
             .force_ck_dly_en           (1'b0               ),// input
             .force_ck_dly_set_bin      (8'h05              ),// input [7:0]
             .dll_step                  (                   ),// output [7:0]
             .dll_lock                  (                   ),// output
             .init_read_clk_ctrl        (2'b0               ),// input [1:0]
             .init_slip_step            (4'b0               ),// input [3:0]
             .force_read_clk_ctrl       (1'b0               ),// input
             .ddrphy_gate_update_en     (1'b0               ),// input
             .update_com_val_err_flag   (                   ),// output [3:0]
             .rd_fake_stop              (1'b0               ) // input
       );*/
wire clk_125Mhz ;

GTP_INBUFGDS #(
    .IOSTANDARD("DEFAULT"),
    .TERM_DIFF("ON")
) u_gtp (
    .O(clk_125Mhz), 
    .I(clk_p), 
    .IB(clk_n) 
);
 ddr3_test u_ddr3_test_h(
             .ref_clk                   (clk_125Mhz            ),
             .resetn                    (rstn_out           ),
             .ddr_init_done             (ddr_init_done      ),

             .pll_lock                  (pll_lock           ),

             .core_clk                  (core_clk),                                  

             .phy_pll_lock              (phy_pll_lock),                          
             .gpll_lock                 (gpll_lock),                                
             .rst_gpll_lock             (rst_gpll_lock),                        
             .ddrphy_cpd_lock           (ddrphy_cpd_lock),                    
             //.ddr_init_done             (ddr_init_done),                        // output

             .axi_awaddr                (axi_awaddr         ),
             .axi_awuser_ap             (1'b0               ),
             .axi_awuser_id             (axi_awuser_id      ),
             .axi_awlen                 (axi_awlen          ),
             .axi_awready               (axi_awready        ),
             .axi_awvalid               (axi_awvalid        ),
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),
             .axi_wready                (axi_wready         ),
             .axi_wusero_id             (                   ),
             .axi_wusero_last           (axi_wusero_last    ),
             .axi_araddr                (axi_araddr         ),
             .axi_aruser_ap             (1'b0               ),
             .axi_aruser_id             (axi_aruser_id      ),
             .axi_arlen                 (axi_arlen          ),
             .axi_arready               (axi_arready        ),
             .axi_arvalid               (axi_arvalid        ),
             .axi_rdata                 (axi_rdata          ),
             .axi_rid                   (axi_rid            ),
             .axi_rlast                 (axi_rlast          ),
             .axi_rvalid                (axi_rvalid         ),

             .apb_clk                   (1'b0               ),
             .apb_rst_n                 (1'b1               ),
             .apb_sel                   (1'b0               ),
             .apb_enable                (1'b0               ),
             .apb_addr                  (8'b0               ),
             .apb_write                 (1'b0               ),
             .apb_ready                 (                   ), 
             .apb_wdata                 (16'b0              ),
             .apb_rdata                 (                   ),
//             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),
             .mem_ck                    (mem_ck             ),
             .mem_ck_n                  (mem_ck_n           ),
             .mem_cke                   (mem_cke            ),
             .mem_cs_n                  (mem_cs_n           ),
             .mem_ras_n                 (mem_ras_n          ),
             .mem_cas_n                 (mem_cas_n          ),
             .mem_we_n                  (mem_we_n           ),
             .mem_odt                   (mem_odt            ),
             .mem_a                     (mem_a              ),
             .mem_ba                    (mem_ba             ),
             .mem_dqs                   (mem_dqs            ),
             .mem_dqs_n                 (mem_dqs_n          ),
             .mem_dq                    (mem_dq             ),
             .mem_dm                    (mem_dm             ),
             // 调试接口

  .dbg_gate_start(1'b0),                      
  .dbg_cpd_start(1'b0),                        
  .dbg_ddrphy_rst_n(1'b1),                  
  .dbg_gpll_scan_rst(1'b0),                
  .samp_position_dyn_adj(1'b0),        
  .init_samp_position_even(32'd0),    
  .init_samp_position_odd(32'd0),      
  .wrcal_position_dyn_adj(1'b0),      
  .init_wrcal_position(32'd0),            
  .force_read_clk_ctrl(1'b0),            
  .init_slip_step(16'd0),                      
  .init_read_clk_ctrl(12'd0),              
  .debug_calib_ctrl(),                  
  .dbg_slice_status(),                  
  .dbg_slice_state(),                    
  .debug_data(),                              
  .dbg_dll_upd_state(),                
  .debug_gpll_dps_phase(),          
  .dbg_rst_dps_state(),                
  .dbg_tran_err_rst_cnt(),          
  .dbg_ddrphy_init_fail(),          
  .debug_cpd_offset_adj(1'b0),          
  .debug_cpd_offset_dir(1'b0),          
  .debug_cpd_offset(10'd0),                  
  .debug_dps_cnt_dir0(),              
  .debug_dps_cnt_dir1(),              
  .ck_dly_en(1'b0),                                
  .init_ck_dly_step(8'h0),                  
  .ck_dly_set_bin(),                      
  .align_error(),                            
  .debug_rst_state(),                    
  .debug_cpd_state()                     
       );
//心跳信号
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end

     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
                 

    reg [31:0] canvas_seq;
    reg canvas_vs_d;
    always @(posedge pix_clk or negedge pix_rstn) begin
        if (!pix_rstn) begin canvas_seq<=0; canvas_vs_d<=0; end
        else begin
            canvas_vs_d<=color_canvas_vs;
            if (color_canvas_vs && !canvas_vs_d) canvas_seq<=canvas_seq + 1'b1;
        end
    end
    wire [127:0] control_snapshot;
    wire [31:0] control_generation;
    stage1_status_cdc u_status_cdc (
        .src_clk(pix_clk), .dst_clk(video_tx_clk),
        .rst_n(pix_rstn && video_pcie_reset_n),
        .source_state({canvas_seq,alarm_reset_seq,key_event_seq,
            23'd0,turn_request,drive_state,display_process_mode_r,display_sel_mode_r}),
        .state_snapshot(control_snapshot), .generation(control_generation)
    );
    video_dma_endpoint #(.ENABLE_DARK(1)) u_pcie_ram_endpoint (
        .control_snapshot(control_snapshot), .control_generation(control_generation),
        .ref_clk_p      (pcie_ref_clk_p),
        .ref_clk_n      (pcie_ref_clk_n),
        .button_rst_n   (1'b1),
        .perst_n        (pcie_perst_n),
        .rxd            (1'b1),
        .txd            (),
        .rxp            (pcie_rxp),
        .rxn            (pcie_rxn),
        .txp            (pcie_txp),
        .txn            (pcie_txn),
        .video_reset_n  (ddr_init_done && pix_rstn),
        .video_clk      (video_tx_clk),
        .video_core_reset_n(video_pcie_reset_n),
        .video_run      (video_run),
        .video_source_idle(video_source_idle),
        .video_frame_valid(video_frame_valid),
        .video_frame_seq(video_frame_seq),
        .video_frame_mode(video_frame_mode),
        .video_frame_epoch(video_frame_epoch),.video_frame_config_generation(video_frame_config),
        .video_frame_native_identity(video_frame_native_identity),
        .feature_reg_wr(feature_reg_wr),.feature_reg_addr(feature_reg_addr),.feature_reg_data(feature_reg_data),
        .feature_read_en(feature_read_en),.feature_read_addr(feature_read_addr),.feature_read_data(feature_read_data),
        .video_frame_accept(video_frame_accept),
        .video_frame_done(video_frame_done),
        .video_data_valid(video_data_valid),
        .video_data     (video_dma_data),
        .video_data_ready(video_data_ready),
        .video_captured (video_captured),
        .video_dropped  (video_dropped),
        .plane_run           (plane_enable),
        // 邮箱描述信息保持在 video_tx_clk 域，载荷经异步 FIFO 到达。
        .plane_frame_valid   (plane_frame_pending),
        .plane_frame_seq     (plane_tx_frame_seq),
        .plane_frame_epoch   (plane_tx_frame_epoch),
        .plane_frame_config  (plane_tx_frame_config),
        .plane_frame_layout  (plane_tx_frame_layout),
        .plane_frame_accept  (plane_tx_frame_accept),
        .plane_frame_done    (plane_tx_frame_done),
        .plane_data_valid    (plane_tx_data_valid),
        .plane_data_error    (plane_tx_data_error),
        .plane_data          (plane_tx_data),
        .plane_data_ready    (plane_tx_data_ready),
        .plane_cfg_base      (plane_cfg_base),
        .plane_cfg_beats     (plane_cfg_slot_beats),
        .plane_cfg_slots     (plane_cfg_slot_count),
        .plane_host_addr     (plane_host_addr),
        .plane_completion    (plane_completion),
        .plane_consumer(plane_consumer),
        // 方案 A：把 TX 的两个计数接出来，供 0x129 / 0x131 使用
        .plane_producer (plane_producer),
        .plane_dropped_frames(plane_dropped_frames),
        .rgb_run(rgb_armed),.rgb_frame_valid(rgb_pending),
        .rgb_frame_seq(rgb_frame_seq),.rgb_frame_epoch(rgb_frame_epoch),.rgb_frame_config(rgb_frame_config),
        .rgb_frame_layout(rgb_frame_layout),.rgb_frame_accept(rgb_frame_accept),.rgb_frame_done(rgb_frame_done),
        .rgb_data_valid(rgb_data_valid),.rgb_data_error(rgb_data_error),.rgb_data(rgb_data),.rgb_data_ready(rgb_data_ready),
        .rgb_host_addr(rgb_host_addr),.rgb_completion(rgb_completion),.rgb_consumer(rgb_consumer),
        .rgb_producer(rgb_producer),.rgb_dropped(rgb_dropped),.rgb_busy(rgb_busy),
        .ref_run(ref_enable),.ref_frame_valid(ref_pending),.ref_source_idle(ref_idle),
        .ref_frame_metadata(ref_frame_metadata),.ref_frame_accept(ref_frame_accept),.ref_frame_done(ref_frame_done),
        .ref_data_valid(ref_data_valid),.ref_data_error(ref_data_error),.ref_data(ref_data),.ref_data_ready(ref_data_ready),
        .ref_host_addrs(ref_host_addrs),.ref_completion(ref_completion),.ref_consumer(ref_consumer),
        .ref_session(ref_session),.ref_stop_cookie(ref_stop_cookie),
        .ref_producer(ref_producer),.ref_dropped(ref_dropped),.ref_stop_sent(ref_stop_sent),.ref_busy(ref_busy),
        .ref_led        (),
        .pclk_led       (),
        .smlh_link_up   (),
        .rdlh_link_up   ()
    );

endmodule
