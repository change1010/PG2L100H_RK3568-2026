

//////////////////////////////////////////////////////////////////////////////
module rt02_dma_tx #(
    parameter                           ADDR_WIDTH     = 'd32,
    parameter                           RD_DATA_DELAY  = 'd1
)
(
    input                               clk                     ,   //gen1:62.5MHz,gen2:125MHz
    input                               rst_n                   ,
    input           [7:0]               i_cfg_pbus_num          ,
    input           [4:0]               i_cfg_pbus_dev_num      ,
    input           [2:0]               i_cfg_max_rd_req_size   ,
    input           [2:0]               i_cfg_max_payload_size  ,
    //**********************************************************************
    //axis_slave0 interface
    input                               i_axis_slave0_trdy      ,
    input                               i_axis_slave0_tvld      ,
    input                               i_axis_slave0_tlast     ,
    //axis_slave1 interface
    input                               i_axis_slave1_trdy      ,
    input                               i_axis_slave1_tvld      ,
    input                               i_axis_slave1_tlast     ,
    //axis_slave2 interface
    input                               i_axis_slave2_trdy      ,
    input                               i_axis_slave2_tvld      ,
    input                               i_axis_slave2_tlast     ,
	
    //**********************************************************************	
	//other ctrl
	input                               i_cpld_finish           ,   //cpld写入16bit RAM结束(1clk)
	
    output	wire                        o_mwr_req_cmd           ,   //mwr写指令来后，需要去开始读16in_16out_ram
    input                               i_mwr_tx_start          ,   //这个信号表示数据已经搬到16in_128out ram.可以开始mwr组包
	//**********************************************************************
    //PIO to tx
    input   [2:0]                       i_bar_sel               ,
    input   [2:0]                       i_mrd_tc                ,
    input   [2:0]                       i_mrd_attr              ,
    input   [9:0]                       i_mrd_length            ,
    input   [15:0]                      i_mrd_id                ,
    input   [7:0]                       i_mrd_tag               ,
    input   [63:0]                      i_mrd_addr              ,
	
    input                               i_mrd_req_vld           ,
	output  wire                        o_mrd_req_rdy           ,
	//**********************************************************************
    //mwr
    input                               i_mwr32_req             ,
    input                               i_mwr64_req             ,
    input                               i_mrd32_req             ,
    input                               i_mrd64_req             ,
    output  wire                        o_mwr32_req_ack         ,
    output  wire                        o_mwr64_req_ack         ,
    output  wire                        o_mrd32_req_ack         ,
    output  wire                        o_mrd64_req_ack         ,

    input           [10:0]              i_mem_length            ,
    input           [63:0]              i_mem_addr              ,
	
    //to mwr	
	input                               i_mwr_data_ready        ,
	output  wire                        o_mwr_data_valid        ,
    output  wire    [127:0]             o_mwr_data              ,
    output  wire                        o_mwr_last_data_en      ,
    output  wire    [9:0]               o_mwr_length            ,
    output  wire    [1:0]               o_mwr_at                ,
    output  wire    [2:0]               o_mwr_attr              ,
    output  wire                        o_mwr_ep                ,
    output  wire                        o_mwr_td                ,
    output  wire                        o_mwr_th                ,
    output  wire    [2:0]               o_mwr_tc                ,
    output  wire    [4:0]               o_mwr_tlp_type          ,
    output  wire    [2:0]               o_mwr_fmt               ,
    output  wire    [7:0]               o_mwr_dwbe              ,
    output  wire     [7:0]              o_mwr_tag               ,
    output  wire    [15:0]              o_mwr_requester_id      ,
    output  wire     [63:0]             o_mwr_addr           	,
    //to mrd	
    output  wire                        o_tx_mrd_en             ,	
    output  wire    [9:0]               o_mrd_length            ,
    output  wire    [1:0]               o_mrd_at                ,
    output  wire    [2:0]               o_mrd_attr              ,
    output  wire                        o_mrd_ep                ,
    output  wire                        o_mrd_td                ,
    output  wire                        o_mrd_th                ,
    output  wire    [2:0]               o_mrd_tc                ,
    output  wire    [4:0]               o_mrd_tlp_type          ,
    output  wire    [2:0]               o_mrd_fmt               ,
    output  wire    [7:0]               o_mrd_dwbe              ,
    output  wire     [7:0]              o_mrd_tag               ,
    output  wire    [15:0]              o_mrd_requester_id      ,
    output  wire     [63:0]             o_mrd_addr           	,

	//cpld	
	input                               i_cpld_tx_data_ready    ,
	output  wire                        o_cpld_tx_data_valid    ,
    output  wire    [127:0]             o_cpld_tx_data          ,
    output  wire                        o_cpld_tx_last_data_en  ,

    output wire     [9:0]               o_cpld_length           ,
    output wire     [1:0]               o_cpld_at               ,
    output wire     [2:0]               o_cpld_attr             ,
    output wire                         o_cpld_ep               ,
    output wire                         o_cpld_td               ,
    output wire                         o_cpld_th               ,
    output wire     [2:0]               o_cpld_tc               ,
    output wire     [4:0]               o_cpld_tlp_type         ,
    output wire     [2:0]               o_cpld_fmt              ,
	
    output wire     [11:0]              o_cpld_byte_cnt         ,	
    output wire                         o_cpld_bcm              ,
    output wire     [2:0]               o_cpld_compl_status     ,
    output wire     [15:0]              o_cpld_completer_id     ,
    output wire     [6:0]               o_cpld_low_addr         ,
    output wire     [7:0]               o_cpld_tag              ,
    output wire     [15:0]              o_cpld_requester_id     ,	
	
    //**********************************************************************
    output  wire                        o_pio_rd_en             ,
    output  wire    [63:0]              o_pio_rd_addr           ,
    input           [127:0]             i_pio_rd_data           ,
    output  wire                        o_dma_rd_en             ,
    output  wire    [ADDR_WIDTH-1:0]    o_dma_rd_addr           ,
    input           [127:0]             i_dma_rd_data             
);

//mwr
wire                mwr_rd_en             ;
wire      [9:0]     mwr_rd_length         ;
wire                mwr_tx_busy           ;
wire                mwr_tx_hold           ;

wire                mwr_gen_tlp_start     ;
wire      [127:0]   mwr_rd_data           ;
wire                mwr_last_data         ;

//cpld
wire                cpld_rd_en            ;
wire      [9:0]     cpld_rd_length        ;
wire      [63:0]    cpld_rd_addr          ;
wire                cpld_tx_hold          ;

wire                cpld_gen_tlp_start    ;
wire      [127:0]   cpld_rd_data          ;
wire                cpld_last_data        ;

wire      [2:0]     o_bar_sel             ;
wire      [127:0]   pio_rd_data           ;


rt0201_dma_mwr_ctrl u_rt0201_dma_mwr_ctrl
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_cfg_pbus_num             (i_cfg_pbus_num             ),  //input [7:0]
    .i_cfg_pbus_dev_num         (i_cfg_pbus_dev_num         ),  //input [4:0]
    .i_cfg_max_payload_size     (i_cfg_max_payload_size     ),  //input [2:0]
    //**********************************************************************
	//axis_slave interface
    .i_axis_slave2_trdy         (i_axis_slave2_trdy         ),
    .i_axis_slave2_tvld         (i_axis_slave2_tvld         ),
    .i_axis_slave2_tlast        (i_axis_slave2_tlast        ),
	
    .i_mwr32_req                (i_mwr32_req                ),
    .i_mwr64_req                (i_mwr64_req                ),
    .o_mwr32_req_ack            (o_mwr32_req_ack            ),
    .o_mwr64_req_ack            (o_mwr64_req_ack            ),
    .i_mem_length               (i_mem_length               ),
    .i_mem_addr                 (i_mem_addr                 ),

    //**********************************************************************
    .o_mwr_req_cmd              (o_mwr_req_cmd              ),  //mwr写指令来后，需要去开始读16in_16out_ram
    .i_mwr_tx_start             (i_mwr_tx_start             ),  //这个信号表示数据已经搬到16in_128out ram.可以开始mwr组包
    //
    .o_rd_en                    (mwr_rd_en                  ),
    .o_rd_length                (mwr_rd_length              ),
    .o_mwr_tx_busy              (mwr_tx_busy                ),
    .o_mwr_tx_hold              (mwr_tx_hold                ),	
    //	
    .i_data_valid               (mwr_gen_tlp_start          ),
    .i_data                     (mwr_rd_data                ),
    .i_last_data_en             (mwr_last_data              ),
	.o_data_valid               (o_mwr_data_valid           ),
    .o_data                     (o_mwr_data                 ),
    .o_last_data_en             (o_mwr_last_data_en         ),
	
    .o_mwr_length               (o_mwr_length               ),
    .o_mwr_at                   (o_mwr_at                   ),
    .o_mwr_attr                 (o_mwr_attr                 ),
    .o_mwr_ep                   (o_mwr_ep                   ),
    .o_mwr_td                   (o_mwr_td                   ),
    .o_mwr_th                   (o_mwr_th                   ),
    .o_mwr_tc                   (o_mwr_tc                   ),
    .o_mwr_tlp_type             (o_mwr_tlp_type             ),
    .o_mwr_fmt                  (o_mwr_fmt                  ),
    .o_mwr_dwbe                 (o_mwr_dwbe                 ),
    .o_mwr_tag                  (o_mwr_tag                  ),
    .o_mwr_requester_id         (o_mwr_requester_id         ),
    .o_mwr_addr                 (o_mwr_addr                 )


);

rt0202_dma_mrd_ctrl u_rt0202_dma_mrd_ctrl
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_cfg_pbus_num             (i_cfg_pbus_num             ),  //input [7:0]
    .i_cfg_pbus_dev_num         (i_cfg_pbus_dev_num         ),  //input [4:0]
    .i_cfg_max_rd_req_size      (i_cfg_max_rd_req_size      ),  //input [2:0]
    .i_cfg_max_payload_size     (i_cfg_max_payload_size     ),  //input [2:0]
    //**********************************************************************
	//axis_slave interface
    .i_axis_slave1_trdy         (i_axis_slave1_trdy         ),
    .i_axis_slave1_tvld         (i_axis_slave1_tvld         ),
    .i_axis_slave1_tlast        (i_axis_slave1_tlast        ),
	
    //from dma controller
    .i_mrd32_req                (i_mrd32_req                ),
    .i_mrd64_req                (i_mrd64_req                ),
    .o_mrd32_req_ack            (o_mrd32_req_ack            ),
    .o_mrd64_req_ack            (o_mrd64_req_ack            ),
    .i_mem_length               (i_mem_length               ),
    .i_mem_addr                 (i_mem_addr                 ),
    //**********************************************************************
    .o_tx_mrd_en                (o_tx_mrd_en                ),
	
    .o_mrd_length               (o_mrd_length               ),
    .o_mrd_at                   (o_mrd_at                   ),
    .o_mrd_attr                 (o_mrd_attr                 ),
    .o_mrd_ep                   (o_mrd_ep                   ),
    .o_mrd_td                   (o_mrd_td                   ),
    .o_mrd_th                   (o_mrd_th                   ),
    .o_mrd_tc                   (o_mrd_tc                   ),
    .o_mrd_tlp_type             (o_mrd_tlp_type             ),
    .o_mrd_fmt                  (o_mrd_fmt                  ),
    .o_mrd_dwbe                 (o_mrd_dwbe                 ),
    .o_mrd_tag                  (o_mrd_tag                  ),
    .o_mrd_requester_id         (o_mrd_requester_id         ),
    .o_mrd_addr                 (o_mrd_addr                 )

);

rt0203_dma_rd_ram #(
    .ADDR_WIDTH                 (ADDR_WIDTH                 ),
	.RD_DATA_DELAY              (RD_DATA_DELAY              )   //receive rd_data delay clk cyle, support 1、2、3 clk cyle.
)                                                          
u_rt0203_dma_rd_ram                                            
(                                                          
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    //*********************************************************************
    //frome rt0201_mwr_ctrl                                
    .i_rd_en                    (mwr_rd_en                  ),
    .i_rd_length                (mwr_rd_length              ),
    .i_mwr_tx_busy              (mwr_tx_busy                ),
    .i_mwr_tx_hold              (mwr_tx_hold                ),
    .i_mwr_tlp_tx               (i_mwr_data_ready           ),

    //read ram                                             
    .o_bar_rd_clk_en            (o_dma_rd_en                ),
    .o_bar_rd_addr              (o_dma_rd_addr              ),
    .i_bar_rd_data              (i_dma_rd_data              ),
    //to rt0201_mwr_ctrl                                   
	.o_gen_tlp_start            (mwr_gen_tlp_start          ),
    .o_rd_data                  (mwr_rd_data                ),
    .o_last_data                (mwr_last_data              )
);


rt0204_pio_cpld_ctrl u_rt0204_pio_cpld_ctrl
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_cfg_pbus_num             (i_cfg_pbus_num             ),  //input [7:0]
    .i_cfg_pbus_dev_num         (i_cfg_pbus_dev_num         ),  //input [4:0]
    .i_cfg_max_payload_size     (i_cfg_max_payload_size     ),  //input [4:0]
    //**********************************************************************
	//axis_slave interface
    .i_axis_slave0_trdy         (i_axis_slave0_trdy         ),
    .i_axis_slave0_tvld         (i_axis_slave0_tvld         ),
    .i_axis_slave0_tlast        (i_axis_slave0_tlast        ),

	.i_bar_sel                  (i_bar_sel                  ),	
    .i_mrd_tc                   (i_mrd_tc                   ),
    .i_mrd_attr                 (i_mrd_attr                 ),
    .i_mrd_length               (i_mrd_length               ),
    .i_mrd_id                   (i_mrd_id                   ),
    .i_mrd_tag                  (i_mrd_tag                  ),
    .i_mrd_addr                 (i_mrd_addr                 ),		
	.i_cpld_req_vld             (i_mrd_req_vld              ),	
    .o_cpld_req_rdy             (o_mrd_req_rdy              ),

    .o_rd_en                    (cpld_rd_en                 ),
    .o_rd_length                (cpld_rd_length             ),
    .o_rd_addr                  (cpld_rd_addr               ),
    .o_cpld_tx_hold             (cpld_tx_hold               ),	
    //	
    .i_data_valid               (cpld_gen_tlp_start         ),
    .i_data                     (cpld_rd_data               ),
    .i_last_data_en             (cpld_last_data             ),
	.o_data_valid               (o_cpld_tx_data_valid       ),
    .o_data                     (o_cpld_tx_data             ),
    .o_last_data_en             (o_cpld_tx_last_data_en     ),
	
    //**********************************************************************
    .o_bar_sel                  (o_bar_sel                  ),
	
    .o_cpld_length              (o_cpld_length              ),
    .o_cpld_at                  (o_cpld_at                  ),
    .o_cpld_attr                (o_cpld_attr                ),
    .o_cpld_ep                  (o_cpld_ep                  ),
    .o_cpld_td                  (o_cpld_td                  ),
    .o_cpld_th                  (o_cpld_th                  ),
    .o_cpld_tc                  (o_cpld_tc                  ),
    .o_cpld_tlp_type            (o_cpld_tlp_type            ),
    .o_cpld_fmt                 (o_cpld_fmt                 ),
    .o_cpld_byte_cnt            (o_cpld_byte_cnt            ),	
    .o_cpld_bcm                 (o_cpld_bcm                 ),
    .o_cpld_compl_status        (o_cpld_compl_status        ),
    .o_cpld_completer_id        (o_cpld_completer_id        ),
    .o_cpld_low_addr            (o_cpld_low_addr            ),
    .o_cpld_tag                 (o_cpld_tag                 ),
    .o_cpld_requester_id  	    (o_cpld_requester_id        )

);

rt0205_pio_rd_ram #(
    .ADDR_WIDTH                 (ADDR_WIDTH                 ),
	.RD_DATA_DELAY              (RD_DATA_DELAY              )   //receive rd_data delay clk cyle, support 1、2、3 clk cyle.
)                                                          
u_rt0205_pio_rd_ram                                          
(                                                          
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    //*********************************************************************
                              
    .i_rd_en                    (cpld_rd_en                 ),
    .i_rd_length                (cpld_rd_length             ),
    .i_rd_addr                  (cpld_rd_addr               ),
    .i_cpld_tx_hold             (cpld_tx_hold               ),
    .i_cpld_tlp_tx              (i_cpld_tx_data_ready       ),

    //read ram                                             
    .o_bar_rd_clk_en            (o_pio_rd_en                ),
    .o_bar_rd_addr              (o_pio_rd_addr              ),
    .i_bar_rd_data              (pio_rd_data                ),

	.o_gen_tlp_start            (cpld_gen_tlp_start         ),
    .o_rd_data                  (cpld_rd_data               ),
    .o_last_data                (cpld_last_data             )
);

rt0206_data_mux #(
    .ADDR_WIDTH                 (ADDR_WIDTH                 ),
	.RD_DATA_DELAY              (RD_DATA_DELAY              )   //receive rd_data delay clk cyle, support 1、2、3 clk cyle.
)                                                          
u_rt0206_data_mux                                        
(                                                          
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    //*********************************************************************
                              
    .i_cpld_finish              (i_cpld_finish              ),
    .i_mwr_tx_busy              (mwr_tx_busy                ),
    .i_bar_sel                  (o_bar_sel                  ),
    .i_rd_clk_en                (o_pio_rd_en                ),
    .i_rd_addr                  (o_pio_rd_addr              ),                                          
    .i_rd_data                  (i_pio_rd_data              ),
    .o_rd_data                  (pio_rd_data                )
);

endmodule
