module video_rect_read_data_m0
#(
	parameter DATA_WIDTH = 16                       // 每拍视频数据位宽
)
(
	input                       	video_clk,          // 视频像素时钟
	input                       	rst,
    input                       display_enable,

	input	[11:0]                 	video_left_offset, 
	input	[11:0]                 	video_top_offset,
	input	[11:0]                 	video_width,
	input	[11:0]                 	video_height,
	
	output reg                  	read_req,           // 整帧读请求
	input                       	read_req_ack,       // 读请求应答
	output                      	read_en,            // 读数据使能
	input	[DATA_WIDTH - 1:0]     	read_data,          // 读数据

	input                       	timing_hs,    
	input                       	timing_vs,    
	input                       	timing_de,    
	input	[DATA_WIDTH - 1:0]     	timing_data, 

	output                      	hs,                 // 行同步
	output                      	vs,                 // 场同步
	output                      	de,                 // 视频有效
	output	[DATA_WIDTH - 1:0]    	vout_data           // 视频数据
);

wire[11:0]             pos_x;
wire[11:0]             pos_y;
wire                   pos_hs;
wire                   pos_vs;
wire                   pos_de;
wire[DATA_WIDTH - 1:0] pos_data;
reg[DATA_WIDTH - 1:0]  pos_data_d0;
reg[DATA_WIDTH - 1:0]  pos_data_d1;

// 行场同步及数据有效信号延迟两拍。
reg                    pos_hs_d0;
reg                    pos_vs_d0;
reg                    pos_de_d0;
reg                    pos_hs_d1;
reg                    pos_vs_d1;
reg                    pos_de_d1;
reg                    pos_hs_d2;
reg                    pos_vs_d2;
reg                    pos_de_d2;
reg                    read_en_r;
reg                    read_en_r_d0;
// read_req_ack 来自 core_clk 域，在 pix_clk 域直接采样属跨时钟域。
// 补两级同步器；该信号是电平保持型，多两拍不影响握手语义。
reg                    req_ack_d0;
reg                    req_ack_d1;

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		req_ack_d0 <= 1'b0;
		req_ack_d1 <= 1'b0;
	end
	else
	begin
		req_ack_d0 <= read_req_ack;
		req_ack_d1 <= req_ack_d0;
	end
end

reg[DATA_WIDTH - 1:0]  vout_data_r;
assign hs = pos_hs_d2;
assign vs = pos_vs_d2;
assign de = pos_de_d2;

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		pos_hs_d0 <= 1'b0;
		pos_vs_d0 <= 1'b0;
		pos_de_d0 <= 1'b0;
		pos_hs_d1 <= 1'b0;
		pos_vs_d1 <= 1'b0;
		pos_de_d1 <= 1'b0;
		// 输出同步信号也须复位，避免复位期间保留上一帧状态。
		pos_hs_d2 <= 1'b0;
		pos_vs_d2 <= 1'b0;
		pos_de_d2 <= 1'b0;
		pos_data_d0 <= {DATA_WIDTH{1'b0}};
		pos_data_d1 <= {DATA_WIDTH{1'b0}};
		read_en_r_d0 <= 1'b0;
	end
	else
	begin
		// 坐标对应的行场同步和有效信号延迟三拍。
		pos_hs_d0   <= pos_hs;
		pos_vs_d0   <= pos_vs;
		pos_de_d0   <= pos_de;
		pos_hs_d1   <= pos_hs_d0;
		pos_vs_d1   <= pos_vs_d0;
		pos_de_d1   <= pos_de_d0;	
		pos_hs_d2   <= pos_hs_d1;
		pos_vs_d2   <= pos_vs_d1;
		pos_de_d2   <= pos_de_d1;
		pos_data_d0 <= pos_data;
		pos_data_d1 <= pos_data_d0;
		read_en_r_d0 <= read_en_r;
	end
end

assign read_en = read_en_r; 

//当显示时序数据pos_de有效,且横纵坐标(pos_x,pos_y)在需要显示的区域内,拉高读使能
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_en_r <= 1'b0;
    else if(display_enable && pos_de == 1'b1 && pos_x >= video_left_offset && pos_x < video_left_offset + video_width
	        && pos_y >= video_top_offset && pos_y < video_top_offset + video_height)
		read_en_r <= 1'b1;
	else
		read_en_r <= 1'b0;
end

assign vout_data = vout_data_r;
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(read_en_r_d0 == 1'b1)  //因为读使能read_en_r拉高后,下个时钟才给出读FIFO读端口的数据,这里延迟一拍赋值数据
		vout_data_r <= read_data;
	else
		vout_data_r <= pos_data_d1;
end

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_req <= 1'b0;
	else if (!display_enable)
        read_req <= 1'b0;
	else if(pos_vs_d0 == 1'b1 & pos_vs == 1'b0) // 检测场同步边沿。
		read_req <= 1'b1;
	else if(req_ack_d1 == 1'b1)   // 已同步
		read_req <= 1'b0;
end

//这个模块主要是为了获取显示有效区域的横纵坐标
//输出的显示同步信号比输入的显示同步信号延迟两拍
timing_gen_xy#(.DATA_WIDTH(DATA_WIDTH)) timing_gen_xy_m0(
	.rst_n    (~rst          ), 
	.clk      (video_clk     ), 

	.i_hs     (timing_hs     ), 
	.i_vs     (timing_vs     ), 
	.i_de     (timing_de     ), 
	.i_data   (timing_data   ), 

	.o_hs     (pos_hs        ), 
	.o_vs     (pos_vs        ), 
	.o_de     (pos_de        ), 
	.o_data   (pos_data      ), 

	.x        (pos_x         ), 
	.y        (pos_y         )  
);
endmodule
