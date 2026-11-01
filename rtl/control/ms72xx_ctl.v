`timescale 1ns / 1ps

// 原提供方：Myminieye
// 原作者：Nill

// 原创建时间：2019-09-16 19:46

// 原模块名：ms7210_ctl
// 原工程名：hdmi_tx
// 目标器件：Pg2l100H

// 原版本：v1.0

`define UD #1
module ms72xx_ctl(
    input       clk,
    input       rst_n,
    
    output      init_over_rx,
    output      init_over,
    output      iic_scl,
    inout       iic_sda
);
    reg rstn_temp1,rstn_temp2;
    reg rstn;
    always @(posedge clk or negedge rst_n)
    begin
    	if(!rst_n)
    	    rstn_temp1 <= 1'b0;
    	else
    	    rstn_temp1 <= rst_n;
    end
    
    always @(posedge clk)
    begin
    	rstn_temp2 <= rstn_temp1;
    	rstn <= rstn_temp2;
    end
    
    // RX 配置完成标志必须由 MS7200 状态机驱动，不能直接置一。
    wire         init_over_rx;

    // RX/TX 共用 K22/K23 上的 I2C 总线，正常先配置 RX 再配置 TX。
    // RX 无响应时等待 0.3 秒后允许 TX 配置，避免 HDMI 输出始终处于复位。
    // 原等待量级依据：RX 约 300 次事务、400 kHz，配置约 10 ms。
    
    localparam [23:0] RX_WAIT_TIMEOUT = 24'd3_000_000;   // 配置时钟 10 MHz 下等待 0.3 秒
    reg  [23:0] rx_wait_cnt;
    wire        rx_timeout;
    wire        tx_rstn;
    wire        tx_sel;         // 1 = 当前 I2C 总线归 ms7210_ctl(TX)，0 = 归 ms7200_ctl(RX)

    always @(posedge clk or negedge rst_n)
    begin
        if(!rst_n)
            rx_wait_cnt <= 24'd0;
        else if(!init_over_rx && !rx_timeout)
            rx_wait_cnt <= rx_wait_cnt + 24'd1;
    end

    assign rx_timeout = (rx_wait_cnt == RX_WAIT_TIMEOUT);
    assign tx_rstn    = init_over_rx | rx_timeout;
    // 总线归属：(RX 已配完 或 已超时) 且 TX 自己还没配完 -> 归 TX。
    assign tx_sel     = (init_over_rx | rx_timeout) & ~init_over;
    wire   [7:0] device_id_rx;
    wire         iic_trig_rx ;
    wire         w_r_rx      ;
    wire  [15:0] addr_rx     ;
    wire  [ 7:0] data_in_rx  ;
    wire         busy_rx     ;
    wire  [ 7:0] data_out_rx ;
    wire         byte_over_rx;
    
    wire   [7:0] device_id_tx;
    wire         iic_trig_tx ;
    wire         w_r_tx      ;
    wire  [15:0] addr_tx     ;
    wire  [ 7:0] data_in_tx  ;
    wire         busy_tx     ;
    wire  [ 7:0] data_out_tx ;
    wire         byte_over_tx;
    
    wire   [7:0] device_id   ;
    wire         iic_trig    ;
    wire         w_r         ;
    wire  [15:0] addr        /*synthesis PAP_MARK_DEBUG="true"*/;
    wire  [ 7:0] data_in     ;
    wire         busy        ;
    wire  [ 7:0] data_out    /*synthesis PAP_MARK_DEBUG="true"*/;
    wire         byte_over   /*synthesis PAP_MARK_DEBUG="true"*/;

    ms7200_ctl U1_ms7200_ctl(
        .clk             (  clk           ),
        .rstn            (  rstn          ),
                              
        .init_over       (  init_over_rx  ),
        .device_id       (                ),
        .iic_trig        (  iic_trig_rx   ),
        .w_r             (  w_r_rx        ),
        .addr            (  addr_rx       ),
        .data_in         (  data_in_rx    ),
        .busy            (  busy_rx       ),
        .data_out        (  data_out_rx   ),
        .byte_over       (  byte_over_rx  ) 
    );
    
    ms7210_ctl U2_ms7210_ctl(
        .clk             (  clk           ),
        .rstn            (  tx_rstn  ),
                              
        .init_over       (  init_over     ),
        .device_id       (                ),
        .iic_trig        (  iic_trig_tx   ),
        .w_r             (  w_r_tx        ),
        .addr            (  addr_tx       ),
        .data_in         (  data_in_tx    ),
        .busy            (  busy_tx       ),
        .data_out        (  data_out_tx   ),
        .byte_over       (  byte_over_tx  ) 
    );
    
    // 两套配置状态机分时复用同一个 I2C 驱动，tx_sel 选定总线归属。
    assign device_id    = tx_sel ? 8'hB2       : 8'h56;
    assign iic_trig     = tx_sel ? iic_trig_tx : iic_trig_rx;
    assign w_r          = tx_sel ? w_r_tx      : w_r_rx;
    assign addr         = tx_sel ? addr_tx     : addr_rx;
    assign data_in      = tx_sel ? data_in_tx  : data_in_rx;
    assign busy_tx      = tx_sel ? busy        : 1'b0;
    assign data_out_tx  = tx_sel ? data_out    : 8'd0;
    assign byte_over_tx = tx_sel ? byte_over   : 1'b0;

    assign busy_rx      = tx_sel ? 1'b0        : busy;
    assign data_out_rx  = tx_sel ? 8'd0        : data_out;
    assign byte_over_rx = tx_sel ? 1'b0        : byte_over;

    wire         sda_in/*synthesis PAP_MARK_DEBUG="true"*/;
    wire         sda_out/*synthesis PAP_MARK_DEBUG="true"*/;
    wire         sda_out_en/*synthesis PAP_MARK_DEBUG="true"*/;  
    iic_dri #(
        .CLK_FRE        (  27'd10_000_000  ),//= clk 实际频率(cfg_clk = 10MHz)
        .IIC_FREQ       (  20'd400_000     ),
        .T_WR           (  10'd1           ),
        .ADDR_BYTE      (  2'd2            ),
        .LEN_WIDTH      (  8'd3            ),
        .DATA_BYTE      (  2'd1            ) 
    )iic_dri(                       
        .clk            (  clk             ),
        .rstn           (  rstn            ),
        .device_id      (  device_id       ),
        .pluse          (  iic_trig        ),
        .w_r            (  w_r             ),
        .byte_len       (  4'd1            ),
                   
        .addr           (  addr            ),
        .data_in        (  data_in         ),
                     
        .busy           (  busy            ),
        
        .byte_over      (  byte_over       ),
        .data_out       (  data_out        ),
                                           
        .scl            (  iic_scl         ),
        .sda_in         (  sda_in          ),
        .sda_out        (  sda_out         ),
        .sda_out_en     (  sda_out_en      ) 
    );
    
    assign iic_sda = sda_out_en ? sda_out : 1'bz;
    assign sda_in = iic_sda;
    
endmodule
