

//////////////////////////////////////////////////////////////////////////////
module rt_dma #(
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
    output  wire                        o_axis_master_trdy      ,
    input                               i_axis_slave0_trdy      ,
    input                               i_axis_slave0_tvld      ,
    input                               i_axis_slave0_tlast     ,
    input                               i_axis_slave1_trdy      ,
    input                               i_axis_slave1_tvld      ,
    input                               i_axis_slave1_tlast     ,
    input                               i_axis_slave2_trdy      ,
    input                               i_axis_slave2_tvld      ,
    input                               i_axis_slave2_tlast     ,
    //**********************************************************************
	//other ctrl
    input                               i_cpld_finish           ,   //is cpld write to ram finish
    output  wire                        o_mwr_req_cmd           ,	//mwr写指令来后，开始读16in_16out_ram
    input                               i_mwr_tx_start          ,	//这个信号表示数据已经搬到16in_128out ram.可以开始mwr组包
	output  wire    [10:0]              o_mem_length            , 
	//ram addr                          
	output  wire    [31:0]              o_mwr_ram_addr          ,  //存储器写操作。从这个地址读取ram_16in_16out
	output  wire    [31:0]              o_mrd_ram_addr          ,  //存储器读操作。接收CplD数据，写入ram_16in_16out       

    //*********************************************************************RX
    input        [9:0]                 i_tlp_common_length       ,
    input        [1:0]                 i_tlp_common_at           ,
	input        [2:0]                 i_tlp_common_attr         ,
	input                              i_tlp_common_ep           ,
	input                              i_tlp_common_td           ,
	input                              i_tlp_common_th           ,
    input        [2:0]                 i_tlp_common_tc           ,
    input        [4:0]                 i_tlp_common_type         ,
    input        [2:0]                 i_tlp_common_fmt          ,
   
    input        [7:0]                 i_tlp_req_dwbe            ,
    input        [7:0]                 i_tlp_req_tag             ,
    input        [15:0]                i_tlp_req_requester_id    ,	
    input        [63:0]                i_tlp_req_addr            ,
   
    input        [11:0]                i_tlp_cpld_byte_cnt       ,   //还要获得多少数据才能完成全部数据传递
    input                              i_tlp_cpld_bcm            ,   	
    input        [2:0]                 i_tlp_cpld_status         ,
    input        [15:0]                i_tlp_cpld_completer_id   ,
    input        [6:0]                 i_tlp_cpld_low_addr       ,   //TLP中数据的起始地址
    input        [7:0]                 i_tlp_cpld_tag            ,	
    input        [15:0]                i_tlp_cpld_requester_id   ,
    //************************************************************
	input        [2:0]                 i_bar_sel                 ,	   
    input                              i_mrd_req_en              ,   //mrd_en 数据使能	
    input                              i_mwr_req_en              ,   //mwr_en 数据使能
    input        [127:0]               i_mwr_req_data            ,
    input        [3:0]                 i_mwr_req_dw_vld          ,
    input                              i_mwr_req_en_flag         ,
    input                              i_cpld_en                 ,
    input        [127:0]               i_cpld_data               ,	
    input        [3:0]                 i_cpld_dw_vld             ,
    input                              i_cpld_en_flag            ,
    input                              i_cpld_multi_flag         , 
	
	//**********************************************************************TX
    //to mwr	
	input                               i_mwr_data_ready            ,
	output  wire                        o_mwr_data_valid            ,
    output  wire    [127:0]             o_mwr_data                  ,
    output  wire                        o_mwr_last_data_en          ,
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
    output  wire    [7:0]               o_mrd_tag               ,
    output  wire    [15:0]              o_mrd_requester_id      ,
    output  wire    [63:0]              o_mrd_addr           	,

	//cpld
	input                               i_cpld_tx_data_ready    ,	
    output  wire                        o_cpld_tx_data_valid    ,
    output  wire     [127:0]            o_cpld_tx_data          ,
    output  wire                        o_cpld_tx_last_data_en  ,
	
    output  wire     [9:0]              o_cpld_length           ,
    output  wire     [1:0]              o_cpld_at               ,
    output  wire     [2:0]              o_cpld_attr             ,
    output  wire                        o_cpld_ep               ,
    output  wire                        o_cpld_td               ,
    output  wire                        o_cpld_th               ,
    output  wire     [2:0]              o_cpld_tc               ,
    output  wire     [4:0]              o_cpld_tlp_type         ,
    output  wire     [2:0]              o_cpld_fmt              ,
	        
    output  wire     [11:0]             o_cpld_byte_cnt         ,	
    output  wire                        o_cpld_bcm              ,
    output  wire     [2:0]              o_cpld_compl_status     ,
    output  wire     [15:0]             o_cpld_completer_id     ,
    output  wire     [6:0]              o_cpld_low_addr         ,
    output  wire     [7:0]              o_cpld_tag              ,
    output  wire     [15:0]             o_cpld_requester_id     ,	
   //**********************************************************************		
   
    output  wire                        o_dma_wr_en             ,
    output  wire    [15:0]              o_dma_wr_byte_en        , 
    output  wire    [ADDR_WIDTH-1:0]    o_dma_wr_addr           ,
    output  wire    [127:0]             o_dma_wr_data           ,
    output  wire                        o_dma_rd_en             ,
    output  wire    [ADDR_WIDTH-1:0]    o_dma_rd_addr           ,
    input           [127:0]             i_dma_rd_data           ,  
	
    output  wire                        o_pio_wr_en             ,
    output  wire    [15:0]              o_pio_wr_byte_en        ,
    output  wire    [63:0]              o_pio_wr_addr           ,
    output  wire    [127:0]             o_pio_wr_data           ,
    output  wire                        o_pio_rd_en             ,
    output  wire    [63:0]              o_pio_rd_addr           ,
    input           [127:0]             i_pio_rd_data           
);
 
//mwr req & ack
wire                            mwr32_req;
wire                            mwr32_req_ack;
wire                            mwr64_req;
wire                            mwr64_req_ack;
//mrd req & ack
wire                            mrd32_req;
wire                            mrd32_req_ack;
wire                            mrd64_req;
wire                            mrd64_req_ack;
wire  [63:0]                    o_mem_addr ;
wire                            o_mrd_req_rdy;

	
rt01_dma_rx u_rt01_dma_rx
(
    .clk                     (clk                    ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                   (rst_n                  ),
    .i_cfg_max_rd_req_size   (i_cfg_max_rd_req_size  ),  
    .i_cfg_max_payload_size  (i_cfg_max_payload_size ),
    //**********************************************************************
	.i_tlp_common_length     (i_tlp_common_length    ),
    .i_tlp_req_dwbe          (i_tlp_req_dwbe         ),
    .i_tlp_req_addr          (i_tlp_req_addr         ),
    .i_tlp_cpld_byte_cnt     (i_tlp_cpld_byte_cnt    ),
    .i_tlp_cpld_low_addr     (i_tlp_cpld_low_addr    ),	
	
    .i_bar_sel               (i_bar_sel              ),
    .i_mwr_req_en            (i_mwr_req_en           ),
    .i_mwr_req_data          (i_mwr_req_data         ),
    .i_mwr_req_dw_vld        (i_mwr_req_dw_vld       ),
    .i_mwr_req_en_flag       (i_mwr_req_en_flag      ),

    .i_cpld_tag              (i_tlp_cpld_tag         ),
    .i_cpld_en               (i_cpld_en              ),
    .i_cpld_data             (i_cpld_data            ),
	.i_cpld_dw_vld           (i_cpld_dw_vld          ),
    .i_cpld_en_flag          (i_cpld_en_flag         ),
    .i_cpld_multi_flag       (i_cpld_multi_flag      ),	
	
	//**********************************************************************
    .i_mwr32_req_ack         (mwr32_req_ack          ),
	.i_mwr64_req_ack         (mwr64_req_ack          ),
    .i_mrd32_req_ack         (mrd32_req_ack          ),
    .i_mrd64_req_ack         (mrd64_req_ack          ),
    .o_mwr32_req             (mwr32_req              ),
    .o_mwr64_req             (mwr64_req              ),
    .o_mrd32_req             (mrd32_req              ),
    .o_mrd64_req             (mrd64_req              ),

    .o_mem_length            (o_mem_length           ),		
    .o_mem_addr              (o_mem_addr             ),	
	
	.o_mwr_ram_addr          (o_mwr_ram_addr         ),
	.o_mrd_ram_addr          (o_mrd_ram_addr         ),	
	
    //**********************************************************************	
	
	.o_dma_wr_en             (o_dma_wr_en            ),
    .o_dma_wr_byte_en        (o_dma_wr_byte_en       ), 
    .o_dma_wr_addr           (o_dma_wr_addr          ),
    .o_dma_wr_data           (o_dma_wr_data          ), 
    .o_pio_wr_en             (o_pio_wr_en            ),
    .o_pio_wr_byte_en        (o_pio_wr_byte_en       ),
    .o_pio_wr_addr           (o_pio_wr_addr          ),
    .o_pio_wr_data           (o_pio_wr_data          )
);


rt02_dma_tx #(
    .ADDR_WIDTH             (ADDR_WIDTH             ),
	.RD_DATA_DELAY          (RD_DATA_DELAY          )   //receive rd_data delay clk cyle, support 1、2、3 clk cyle.
)                                                          
u_rt02_dma_tx                                            
( 
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_cfg_pbus_num             (i_cfg_pbus_num             ),  
    .i_cfg_pbus_dev_num         (i_cfg_pbus_dev_num         ),
    .i_cfg_max_rd_req_size      (i_cfg_max_rd_req_size      ),  
    .i_cfg_max_payload_size     (i_cfg_max_payload_size     ),
    //**********************************************************************master
    .i_axis_slave0_trdy         (i_axis_slave0_trdy         ),
    .i_axis_slave0_tvld         (i_axis_slave0_tvld         ),
    .i_axis_slave0_tlast        (i_axis_slave0_tlast        ),
    .i_axis_slave1_trdy         (i_axis_slave1_trdy         ),
    .i_axis_slave1_tvld         (i_axis_slave1_tvld         ),
    .i_axis_slave1_tlast        (i_axis_slave1_tlast        ),
    .i_axis_slave2_trdy         (i_axis_slave2_trdy         ),
    .i_axis_slave2_tvld         (i_axis_slave2_tvld         ),
    .i_axis_slave2_tlast        (i_axis_slave2_tlast        ),	
	//**********************************************************************
		//other ctrl
    .i_cpld_finish              (i_cpld_finish              ),	
    .o_mwr_req_cmd              (o_mwr_req_cmd              ),	//mwr写指令来后，需要去开始读16in_16out_ram 1clk
    .i_mwr_tx_start             (i_mwr_tx_start             ),	//这个信号表示数据已经搬到16in_128out ram.可以开始mwr组包	 1clk
	
    //PIO to tx
    .i_bar_sel                  (i_bar_sel                  ),
	.i_mrd_tc                   (i_tlp_common_tc            ),
    .i_mrd_attr                 (i_tlp_common_attr          ),
    .i_mrd_length               (i_tlp_common_length        ),
    .i_mrd_id                   (i_tlp_req_requester_id     ),
    .i_mrd_tag                  (i_tlp_req_tag              ),
    .i_mrd_addr                 (i_tlp_req_addr             ),
	
	.i_mrd_req_vld              (i_mrd_req_en               ),	  //mrd_en
    .o_mrd_req_rdy              (o_mrd_req_rdy              ),
	
	//**********************************************************************
    .i_mwr32_req                (mwr32_req                  ),
    .i_mwr64_req                (mwr64_req                  ),
    .i_mrd32_req                (mrd32_req                  ),
    .i_mrd64_req                (mrd64_req                  ),
    .o_mwr32_req_ack            (mwr32_req_ack              ),
    .o_mwr64_req_ack            (mwr64_req_ack              ),
    .o_mrd32_req_ack            (mrd32_req_ack              ),
    .o_mrd64_req_ack            (mrd64_req_ack              ),

    .i_mem_length               (o_mem_length               ),
    .i_mem_addr                 (o_mem_addr                 ),
	
    //to mwr
	.i_mwr_data_ready           (i_mwr_data_ready           ),
	.o_mwr_data_valid           (o_mwr_data_valid           ),
    .o_mwr_data                 (o_mwr_data                 ),
    .o_mwr_last_data_en         (o_mwr_last_data_en         ),
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
    .o_mwr_addr           	    (o_mwr_addr                 ),
    //to mrd
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
    .o_mrd_addr           	    (o_mrd_addr                 ),	
	//cpld
    .i_cpld_tx_data_ready       (i_cpld_tx_data_ready       ),	
    .o_cpld_tx_data_valid       (o_cpld_tx_data_valid       ),
    .o_cpld_tx_data             (o_cpld_tx_data             ),
    .o_cpld_tx_last_data_en     (o_cpld_tx_last_data_en     ),
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
    .o_cpld_requester_id        (o_cpld_requester_id        ),		
	
    //**********************************************************************	
    .o_pio_rd_en                (o_pio_rd_en                ),
    .o_pio_rd_addr              (o_pio_rd_addr              ),
    .i_pio_rd_data              (i_pio_rd_data              ),
	.o_dma_rd_en                (o_dma_rd_en                ),
    .o_dma_rd_addr              (o_dma_rd_addr              ),
    .i_dma_rd_data              (i_dma_rd_data              )
);

assign o_axis_master_trdy = o_mrd_req_rdy;

endmodule
