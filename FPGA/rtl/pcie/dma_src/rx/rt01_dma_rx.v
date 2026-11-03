

//////////////////////////////////////////////////////////////////////////////
module rt01_dma_rx (
    input                       clk                     ,   //gen1:62.5MHz,gen2:125MHz
    input                       rst_n                   ,
    input         [2:0]         i_cfg_max_rd_req_size   ,
    input         [2:0]         i_cfg_max_payload_size  ,
	
    input         [9:0]         i_tlp_common_length     ,
	input         [7:0]         i_tlp_req_dwbe          ,  
    input         [63:0]        i_tlp_req_addr          ,
    input         [11:0]        i_tlp_cpld_byte_cnt     ,	
    input         [6:0]         i_tlp_cpld_low_addr     ,
	
    input         [2:0]         i_bar_sel               ,
	input                       i_mwr_req_en            ,  	//mwr_en 数据使能
    input         [127:0]       i_mwr_req_data          ,
    input         [3:0]         i_mwr_req_dw_vld        ,
    input                       i_mwr_req_en_flag       ,  

	input         [7:0]         i_cpld_tag              ,
    input                       i_cpld_en               ,
    input         [127:0]       i_cpld_data             ,	
    input         [3:0]         i_cpld_dw_vld           ,
    input                       i_cpld_en_flag          ,
    input                       i_cpld_multi_flag       , 	

    //**********************************************************************
	////////to tx engine mwr and mrd
	input                       i_mwr32_req_ack         , 
	input                       i_mwr64_req_ack         , 
	input                       i_mrd32_req_ack         , 
	input                       i_mrd64_req_ack         , 
	output   wire               o_mwr32_req             ,   
	output   wire               o_mwr64_req             , 
	output   wire               o_mrd32_req             , 
	output   wire               o_mrd64_req             , 
	//mem addr	
	output   wire  [10:0]       o_mem_length            , 
	output   wire  [63:0]       o_mem_addr              ,
	//ram addr
	output   wire  [31:0]       o_mwr_ram_addr          ,  //存储器写操作。从这个地址读取ram_16in_16out
	output   wire  [31:0]       o_mrd_ram_addr          ,  //存储器读操作。接收CplD数据，写入ram_16in_16out   

    output   wire               o_dma_wr_en             ,
    output   wire  [15:0]       o_dma_wr_byte_en        , 
    output   wire  [63:0]       o_dma_wr_addr           ,
    output   wire  [127:0]      o_dma_wr_data           ,
                   
    output   reg                o_pio_wr_en             ,
    output   reg   [15:0]       o_pio_wr_byte_en        ,
    output   reg   [63:0]       o_pio_wr_addr           ,
    output   reg   [127:0]      o_pio_wr_data       
);


wire  [63:0]    cpld_start_addr  ;
wire  [2:0]     pio_bar_sel      ;
wire            pio_wr_en        ;
wire  [15:0]    pio_wr_byte_en   ;
wire  [63:0]    pio_wr_addr      ;
wire  [127:0]   pio_wr_data      ;

rt0101_dma_request_ctrl u_rt0101_dma_request_ctrl
(
    .clk               (clk              ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n             (rst_n            ),

    //**********************************************************************master
    //AXIS master interface
    .i_mwr_req_en      (i_mwr_req_en     ),
    .i_tlp_req_addr    (i_tlp_req_addr   ),
    .i_mwr_req_data    (i_mwr_req_data   ),
    .i_tlp_req_dwbe    (i_tlp_req_dwbe   ),
    .i_bar_sel         (i_bar_sel        ),

	//**********************************************************************
    .i_mwr32_req_ack   (i_mwr32_req_ack  ),
	.i_mwr64_req_ack   (i_mwr64_req_ack  ),
    .i_mrd32_req_ack   (i_mrd32_req_ack  ),
    .i_mrd64_req_ack   (i_mrd64_req_ack  ),
    .o_mwr32_req       (o_mwr32_req      ),
    .o_mwr64_req       (o_mwr64_req      ),
    .o_mrd32_req       (o_mrd32_req      ),
    .o_mrd64_req       (o_mrd64_req      ),

	
    .o_mem_length      (o_mem_length     ),		
    .o_mem_addr        (o_mem_addr       ),	
	
	.o_mwr_ram_addr    (o_mwr_ram_addr   ),
	.o_mrd_ram_addr    (o_mrd_ram_addr   )	
);

rt0102_dma_cpld_tag_ctrl u_rt0102_dma_cpld_tag_ctrl
(
    .clk                     (clk                    ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                   (rst_n                  ),
    .i_cfg_max_rd_req_size   (i_cfg_max_rd_req_size  ),  //gen1:62.5MHz,gen2:125MHz
    .i_cfg_max_payload_size  (i_cfg_max_payload_size ),
    //**********************************************************************
    .i_cpld_length           (i_tlp_common_length    ),
    .i_cpld_tag              (i_cpld_tag             ),
    .i_multicpld_flag        (i_cpld_multi_flag      ),
    .i_cpld_data_flag        (i_cpld_en_flag         ),

    //**********************************************************************
    .o_cpld_addr             (cpld_start_addr      )
);


//***************************calculating dwbe from byte count and lower address
wire    [1:0]       last_dw_byte_num;
reg     [3:0]       first_dw_be;
reg     [3:0]       last_dw_be;
wire    [7:0]       cpld_dwbe ;

always@(*)
begin
    case(i_tlp_cpld_low_addr[1:0])
        2'd0: first_dw_be = 4'b1111;
        2'd1: first_dw_be = 4'b1110;
        2'd2: first_dw_be = 4'b1100;
        2'd3: first_dw_be = 4'b1000;
        default: first_dw_be = 4'hf;
    endcase
end

assign last_dw_byte_num = i_tlp_cpld_byte_cnt[1:0] + i_tlp_cpld_low_addr[1:0];

always@(*)
begin
    case(last_dw_byte_num)
        2'd0: last_dw_be = 4'b1111;
        2'd1: last_dw_be = 4'b0001;
        2'd2: last_dw_be = 4'b0011;
        2'd3: last_dw_be = 4'b0111;
        default: last_dw_be = 4'hf;
    endcase
end

assign cpld_dwbe = {last_dw_be,first_dw_be};



rt0103_dma_data_wr_ctrl u_rt0103_dma_data_wr_ctrl
(
    .clk                (clk                ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n              (rst_n              ),

    //**********************************************************************
    .i_bar_sel          (i_bar_sel          ),
    .i_de               (i_cpld_en          ),
    .i_length           (i_tlp_common_length),
    .i_dwbe             (cpld_dwbe          ),
    .i_data             (i_cpld_data        ),
    .i_dw_vld           (i_cpld_dw_vld      ),
    .i_addr             (cpld_start_addr    ),
    .i_data_flag        (i_cpld_en_flag     ),

    //**********************************************************************
    //ram write control
    .o_wr_bar_sel       (       ),
    .o_wr_en            (o_dma_wr_en        ),
    .o_wr_addr          (o_dma_wr_addr      ),
    .o_wr_data          (o_dma_wr_data      ),
    .o_wr_be            (o_dma_wr_byte_en   )
);


rt0103_dma_data_wr_ctrl u_rt0104_pio_data_wr_ctrl
(
    .clk                (clk                ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n              (rst_n              ),

    //**********************************************************************
    .i_bar_sel          (i_bar_sel          ),
    .i_de               (i_mwr_req_en       ),
    .i_length           (i_tlp_common_length),
    .i_dwbe             (i_tlp_req_dwbe     ),
    .i_data             (i_mwr_req_data     ),
    .i_dw_vld           (i_mwr_req_dw_vld   ),
    .i_addr             (i_tlp_req_addr     ),
    .i_data_flag        (i_mwr_req_en_flag  ),

    //**********************************************************************
    //ram write control
    .o_wr_bar_sel       (pio_bar_sel        ),
    .o_wr_en            (pio_wr_en          ),
    .o_wr_addr          (pio_wr_addr        ),
    .o_wr_data          (pio_wr_data        ),
    .o_wr_be            (pio_wr_byte_en     )
);

always@(*)
begin
    if(pio_bar_sel == 2'b0)
    begin
        o_pio_wr_en       = pio_wr_en     ;
        o_pio_wr_addr     = pio_wr_addr   ;
        o_pio_wr_data     = pio_wr_data   ;
        o_pio_wr_byte_en  = pio_wr_byte_en;
    end
    else
    begin
        o_pio_wr_en       = 'd0;  
        o_pio_wr_addr     = 'd0;
        o_pio_wr_data     = 'd0;
        o_pio_wr_byte_en  = 'd0;
    end
end


endmodule
