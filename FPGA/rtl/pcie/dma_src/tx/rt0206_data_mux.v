

//////////////////////////////////////////////////////////////////////////////
module rt0206_data_mux#(
    parameter                           ADDR_WIDTH     = 'd32,
    parameter                           RD_DATA_DELAY  = 'd1
)
(
    input                               clk             ,  //gen1:62.5MHz,gen2:125MHz
    input                               rst_n           ,
    //**********************************************************************
  	input                               i_cpld_finish   ,  //cpld写入16bit RAM结束(1clk)
	input                               i_mwr_tx_busy   ,  //mwr发送期间i_mwr_tx_busy信号高。这个信号下沿是mwr tx的结束标志。
	
    input      [2:0]                    i_bar_sel       ,
    input                               i_rd_clk_en     ,
    input      [ADDR_WIDTH-1:0]         i_rd_addr       ,
    input      [127:0]                  i_rd_data       ,
	
    output reg [127:0]                  o_rd_data       
);



reg i_mwr_tx_busy_d1;
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        i_mwr_tx_busy_d1 <= 1'b0;
    else
        i_mwr_tx_busy_d1 <= i_mwr_tx_busy;
end

assign i_mwr_tx_busy_ne = ~i_mwr_tx_busy && i_mwr_tx_busy_d1;

wire cpld_finish   = i_cpld_finish;
wire mwr_tx_finish = i_mwr_tx_busy_ne; 

wire  [ADDR_WIDTH-1:0] rd_addr = {i_rd_addr[ADDR_WIDTH-5:0],4'b0};
//****************************************

reg cpld_finish_flag;
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        cpld_finish_flag <= 1'b0;
    else if(cpld_finish)
        cpld_finish_flag <= 1'b1;
    else if(i_rd_clk_en && (i_bar_sel == 3'b1) && (rd_addr[11:0] == 12'h130))
        cpld_finish_flag <= 1'b0;
    else
        cpld_finish_flag <= cpld_finish_flag;	    
end

reg mwr_tx_finish_flag;
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        mwr_tx_finish_flag <= 1'b0;
    else if(mwr_tx_finish)
        mwr_tx_finish_flag <= 1'b1;
    else if(i_rd_clk_en && (i_bar_sel == 3'b1) && (rd_addr[11:0] == 12'h230))
        mwr_tx_finish_flag <= 1'b0;
    else
        mwr_tx_finish_flag <= mwr_tx_finish_flag;	    
end


reg [127:0] cpld_finish_state;
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        cpld_finish_state <= 'd0;
    else if(i_rd_clk_en && (i_bar_sel == 3'b1) && (rd_addr[11:0] == 12'h130))
        cpld_finish_state <= {127'b0,cpld_finish_flag};
    else
        cpld_finish_state <= cpld_finish_state;	    
end

reg [127:0] mwr_tx_finish_state;
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        mwr_tx_finish_state <= 'd0;
    else if(i_rd_clk_en && (i_bar_sel == 3'b1) && (rd_addr[11:0] == 12'h230))
        mwr_tx_finish_state <= {127'b0,mwr_tx_finish_flag};
    else
        mwr_tx_finish_state <= mwr_tx_finish_state;	    
end

//
reg [127:0] cpld_finish_state_d1;  //delay
reg [127:0] cpld_finish_state_d2;  //delay
always@(posedge clk or negedge rst_n)begin
    if(!rst_n) begin
        cpld_finish_state_d1 <= 'd0;
        cpld_finish_state_d2 <= 'd0;
	end
    else begin
        cpld_finish_state_d1 <= cpld_finish_state;	  
        cpld_finish_state_d2 <= cpld_finish_state_d1;	
    end		
end
reg [127:0] mwr_tx_finish_state_d1;  //delay
reg [127:0] mwr_tx_finish_state_d2;  //delay
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        mwr_tx_finish_state_d1 <= 'd0;
        mwr_tx_finish_state_d2 <= 'd0;
	end
    else begin
        mwr_tx_finish_state_d1 <= mwr_tx_finish_state;
        mwr_tx_finish_state_d2 <= mwr_tx_finish_state_d1;
    end	    
end

////////////////////////////////////////////////////////////////////
reg  [11:0] rd_addr_d1;
reg  [11:0] rd_addr_d2;
reg  [11:0] rd_addr_d3;
always@(posedge clk or negedge rst_n)begin
    if(!rst_n) begin
        rd_addr_d1 <= 'd0;
        rd_addr_d2 <= 'd0;
	    rd_addr_d3 <= 'd0;
	end
    else begin
        rd_addr_d1 <= rd_addr[11:0];
        rd_addr_d2 <= rd_addr_d1;
	    rd_addr_d3 <= rd_addr_d2;
    end		
end


always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        o_rd_data <= 'd0;
    else if(i_bar_sel == 3'b1) begin
	    case(RD_DATA_DELAY)
            2'b01:begin
			    if(rd_addr_d1[11:0] == 12'h130)
                   o_rd_data <= cpld_finish_state;
				else if(rd_addr_d1[11:0] == 12'h230)
                   o_rd_data <= mwr_tx_finish_state;
			end
            2'b10:begin
			    if(rd_addr_d2[11:0] == 12'h130)
                   o_rd_data <= cpld_finish_state_d1;
				else if(rd_addr_d2[11:0] == 12'h230)
                   o_rd_data <= mwr_tx_finish_state_d1;
			end
            2'b11:begin
			    if(rd_addr_d3[11:0] == 12'h130)
                   o_rd_data <= cpld_finish_state_d2;
				else if(rd_addr_d3[11:0] == 12'h230)
                   o_rd_data <= mwr_tx_finish_state_d2;
			end
        default:
                o_rd_data <= o_rd_data;              
        endcase
	end
    else
    begin
        o_rd_data <= i_rd_data;
    end
end


endmodule
