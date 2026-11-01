

//////////////////////////////////////////////////////////////////////////////
module rt0202_dma_mrd_ctrl (
    input                               clk                     ,   //gen1:62.5MHz,gen2:125MHz
    input                               rst_n                   ,
    input           [7:0]               i_cfg_pbus_num          ,
    input           [4:0]               i_cfg_pbus_dev_num      ,
    input           [2:0]               i_cfg_max_rd_req_size   ,
    input           [2:0]               i_cfg_max_payload_size  ,
    //**********************************************************************
	input                               i_axis_slave1_trdy      ,
    input                               i_axis_slave1_tvld      ,
    input                               i_axis_slave1_tlast     , 
    //from dma controller
    input                               i_mrd32_req             ,
    output  reg                         o_mrd32_req_ack         ,
    input                               i_mrd64_req             ,
    output  reg                         o_mrd64_req_ack         ,
    input           [10:0]              i_mem_length            ,
    input           [63:0]              i_mem_addr              ,
    //**********************************************************************
	output  reg                         o_tx_mrd_en             ,
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
    output  reg     [7:0]               o_mrd_tag               ,
    output  wire    [15:0]              o_mrd_requester_id      ,
    output  reg     [63:0]              o_mrd_addr                    	
);

localparam      IDLE        = 2'd0;
localparam      HEADER_TX   = 2'd1;

reg     [10:0]  mrd_length/* synthesis PAP_MARK_DEBUG=”true”*/;
reg     [10:0]  mrd_length_ff;

wire            mrd_req_de/* synthesis PAP_MARK_DEBUG=”true”*/;

reg             mrd32_req_tx;
reg             mrd64_req_tx;

reg     [1:0]   state/* synthesis PAP_MARK_DEBUG=”true”*/;
reg     [1:0]   next_state;

wire            mrd_req_rcv;


//tlp_tx
wire    [3:0]   first_dwbe;
wire    [3:0]   last_dwbe;

reg             tx_busy/* synthesis PAP_MARK_DEBUG=”true”*/;
reg             tx_busy_d1;
wire            tx_busy_ne;

wire            mrd_req_ack;
wire            mrd_tx_hold;

wire            tx_done;
wire [9:0]      max_rd_req_size;

assign mrd_req_rcv = i_mrd32_req || i_mrd64_req;
assign mrd_req_ack = o_mrd32_req_ack || o_mrd64_req_ack;
assign mrd_req_de = mrd_req_rcv && mrd_req_ack;

assign tx_done  =  i_axis_slave1_trdy && i_axis_slave1_tvld && i_axis_slave1_tlast;

//
reg [2:0] min_size;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        min_size <= 'd0;
    else if(i_cfg_max_payload_size <= i_cfg_max_rd_req_size)
        min_size <= i_cfg_max_payload_size;
    else 
        min_size <= i_cfg_max_rd_req_size;
end
//

assign max_rd_req_size = (min_size == 3'd0) ? 10'h20 :
                         (min_size == 3'd1) ? 10'h40 :
                         (min_size == 3'd2) ? 10'h80 :
                         (min_size == 3'd3) ? 10'h100 : 10'd20;

//get tlp information
//length > max_rd_req_size,
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        mrd_length <= 'd0;
    else if(mrd_req_de && !tx_busy)
        mrd_length <= i_mem_length;
    else if((mrd_length >  max_rd_req_size) && o_tx_mrd_en && i_axis_slave1_trdy)
        mrd_length <= mrd_length - max_rd_req_size;
    else if((mrd_length <= max_rd_req_size) && o_tx_mrd_en && i_axis_slave1_trdy)
        mrd_length <= 'd0;
end

always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        mrd_length_ff <= 10'd0;
    else
        mrd_length_ff <= mrd_length;
end

//the true length to be transmitted
assign o_mrd_length = (mrd_length_ff > max_rd_req_size) ? max_rd_req_size : mrd_length_ff ;

always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        o_mrd_addr <= 64'd0;
    else if(mrd_req_de && !tx_busy)
        o_mrd_addr <= i_mem_addr;
    else if(|mrd_length && tx_done)
        o_mrd_addr <= o_mrd_addr + {52'b0,max_rd_req_size,2'b0};
end

//start tx mrd
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        o_tx_mrd_en <= 1'b0;
    else if(i_axis_slave1_trdy && o_tx_mrd_en)
        o_tx_mrd_en <= 1'b0;
    else if(mrd_req_de && !tx_busy)  //new req rcv
        o_tx_mrd_en <= 1'b1;
    else if(|mrd_length && tx_done)  //for length > max_rd_req_size
        o_tx_mrd_en <= 1'b1;
end


always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        o_mrd_tag <= 8'b0;
    else if(tx_busy_ne)
        o_mrd_tag <= 8'b0;
    else if(tx_done)
        o_mrd_tag <= o_mrd_tag + 1'b1;
    else
        o_mrd_tag <= o_mrd_tag;
end



//**********************************************************************

always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        mrd32_req_tx <= 1'b0;
    else if(tx_done && ~(|mrd_length))
        mrd32_req_tx <= 1'b0;
    else if(mrd_req_de && i_mrd32_req)
        mrd32_req_tx <= 1'b1;
	else
        mrd32_req_tx <= mrd32_req_tx;	    
end

always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        mrd64_req_tx <= 1'b0;
    else if(tx_done && ~(|mrd_length))
        mrd64_req_tx <= 1'b0;
    else if(mrd_req_de && i_mrd64_req)
        mrd64_req_tx <= 1'b1;
	else
        mrd64_req_tx <= mrd64_req_tx;	    
end

//tlp header
assign o_mrd_requester_id = {i_cfg_pbus_num,i_cfg_pbus_dev_num,3'b0};
assign {o_mrd_fmt,o_mrd_tlp_type}  = mrd32_req_tx ? 8'h0
                                    : mrd64_req_tx ? 8'h20 : 8'h00;
assign o_mrd_tc    = 3'b0;
assign o_mrd_attr  = 3'b0;
assign {o_mrd_th,o_mrd_td,o_mrd_ep,o_mrd_at}  = 5'b0;
assign first_dwbe  = 4'hf;
assign last_dwbe   = o_mrd_length == 10'h1 ? 4'h0 : 4'hf;
assign o_mrd_dwbe  = {last_dwbe,first_dwbe};


//mrd_tx
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        state <= IDLE;
    else
        state <= next_state;
end

always @(*)
begin
    case(state)
        IDLE:
        begin
            if(o_tx_mrd_en && i_axis_slave1_trdy) //start transmit
                next_state = HEADER_TX;
            else
                next_state = IDLE;
        end
        HEADER_TX:
        begin
            if(mrd_tx_hold)
                next_state = HEADER_TX;
            else
                next_state = IDLE;
        end
        default:
        begin
            next_state = IDLE;
        end
    endcase
end


always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        tx_busy <= 1'b0;
    else if(~(|mrd_length) && tx_done)
        tx_busy <= 1'b0;
    else if(mrd_req_de)
        tx_busy <= 1'b1;
end



always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        tx_busy_d1 <= 1'b0;
    else
        tx_busy_d1 <= tx_busy;
end

assign tx_busy_ne = ~tx_busy && tx_busy_d1;


//ack
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        o_mrd32_req_ack <= 1'b0;
    else if(!i_mrd32_req)
        o_mrd32_req_ack <= 1'b0;
    else if(i_mrd32_req && !tx_busy)
        o_mrd32_req_ack <= 1'b1;
end

always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        o_mrd64_req_ack <= 1'b0;
    else if(!i_mrd64_req)
        o_mrd64_req_ack <= 1'b0;
    else if(i_mrd64_req && !tx_busy)
        o_mrd64_req_ack <= 1'b1;
end

//when i_axis_slave1_trdy down,hold all tx logic
assign mrd_tx_hold = ~i_axis_slave1_trdy && i_axis_slave1_tvld;


endmodule
