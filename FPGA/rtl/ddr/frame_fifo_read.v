`timescale 1ns/1ps
module frame_fifo_read
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 23,
	parameter BUSRT_BITS             = 10,
	parameter FIFO_DEPTH             = 256,
	parameter BURST_SIZE             = 128
)               
(
	input                            rst,                  
	input                            mem_clk,                    // 存储控制器用户时钟
	output reg                       rd_burst_req,               // 发起突发读请求
	output reg[BUSRT_BITS - 1:0]     rd_burst_len,               // 突发读长度，单位为数据拍
	output reg[ADDR_BITS - 1:0]      rd_burst_addr,              // 突发读起始地址
	input                            rd_burst_data_valid,        // 返回读数据有效
	input                            rd_burst_finish,            // 本次读突发完成
	input                            read_req,                   // 帧读取请求，保持到 read_req_ack 有效
	output reg                       read_req_ack,               // 帧读取请求应答
	output                           read_finish,                // 整帧读取完成
	input[ADDR_BITS - 1:0]           read_addr_0,                // 读取帧槽 0 的基地址
	input[ADDR_BITS - 1:0]           read_addr_1,                // 读取帧槽 1 的基地址
	input[ADDR_BITS - 1:0]           read_addr_2,                // 读取帧槽 2 的基地址
	input[ADDR_BITS - 1:0]           read_addr_3,                // 读取帧槽 3 的基地址
	input[1:0]                       read_addr_index,            // 按索引选择读帧槽基地址
	input[ADDR_BITS - 1:0]           read_len,                   // 整帧读取长度
	output reg                       fifo_aclr,                  // FIFO 异步清零
	input[15:0]                      wrusedw                     // 读缓存写侧已用字数
);
localparam ONE                       = 256'd1;                   // 全一常量，可按所需位宽取切片
localparam ZERO                      = 256'd0;                   // 全零常量
// 读状态编码
localparam S_IDLE                    = 0;                        // 空闲，等待帧读取请求
localparam S_ACK                     = 1;                        // 读取请求应答
localparam S_WAIT                    = 2;
localparam S_CHECK_FIFO              = 3;                        // 确认 FIFO 剩余空间可容纳一个读突发
localparam S_READ_BURST              = 4;                        // 发起读突发
localparam S_READ_BURST_END          = 5;                        // 一个读突发完成
localparam S_END                     = 6;                        // 整帧读取完成

reg                                  read_req_d0;                // 读请求同步到 mem_clk 域的第一级
reg                                  read_req_d1;                // 第二级
reg                                  read_req_d2;                // 第三级；后续状态机使用此级请求
reg[ADDR_BITS - 1:0]                 read_len_d0;                // 读取长度跨域采样第一级；请求握手期间须保持稳定
reg[ADDR_BITS - 1:0]                 read_len_d1;                // 第二级
reg[ADDR_BITS - 1:0]                 read_len_latch;             // 锁存本次帧读取长度
reg[ADDR_BITS - 1:0]                 read_cnt;                   // 累计已读取的数据拍数
reg[3:0]                             state;                      // 状态机
reg[1:0]                             read_addr_index_d0;         // 同步到 mem_clk 域的第一级
reg[1:0]                             read_addr_index_d1;         // 同步到 mem_clk 域的第二级
reg[15:0]                            wait_cnt;
assign read_finish = (state == S_END) ? 1'b1 : 1'b0;             // 在 S_END 状态报告整帧读取完成
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		read_req_d0    <=  1'b0;
		read_req_d1    <=  1'b0;
		read_req_d2    <=  1'b0;
		read_len_d0    <=  ZERO[ADDR_BITS - 1:0];               
		read_len_d1    <=  ZERO[ADDR_BITS - 1:0];               
		read_addr_index_d0 <= 2'b00;
		read_addr_index_d1 <= 2'b00;
	end
	else
	begin
		read_req_d0    <=  read_req;
		read_req_d1    <=  read_req_d0;
		read_req_d2    <=  read_req_d1;     
		read_len_d0    <=  read_len;
		read_len_d1    <=  read_len_d0; 
		read_addr_index_d0 <= read_addr_index;
		read_addr_index_d1 <= read_addr_index_d0;
	end 
end


always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		read_len_latch <= ZERO[ADDR_BITS - 1:0];
		rd_burst_addr <= ZERO[ADDR_BITS - 1:0];
		rd_burst_req <= 1'b0;
		read_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		rd_burst_len <= ZERO[BUSRT_BITS - 1:0];
		read_req_ack <= 1'b0;
		wait_cnt <= 16'd0;
	end
	else
		case(state)
			// 空闲时等待同步后的读请求，随后进入 S_ACK。
			S_IDLE:
			begin
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				read_req_ack <= 1'b0;
			end
			// 在 S_ACK 应答请求、清空 FIFO，并锁存帧槽地址和长度。
			S_ACK:
			begin
				if(read_req_d2 == 1'b0)
				begin
					state <= S_WAIT;
					wait_cnt <= 16'd0;
					fifo_aclr <= 1'b0;
					read_req_ack <= 1'b0;
				end
				else
				begin
					// 读取请求应答
					read_req_ack <= 1'b1;
					// 清空 FIFO
					fifo_aclr <= 1'b1;
					// 按索引选择读帧槽基地址
					if(read_addr_index_d1 == 2'd0)
						rd_burst_addr <= read_addr_0;
					else if(read_addr_index_d1 == 2'd1)
						rd_burst_addr <= read_addr_1;
					else if(read_addr_index_d1 == 2'd2)
						rd_burst_addr <= read_addr_2;
					else if(read_addr_index_d1 == 2'd3)
						rd_burst_addr <= read_addr_3;
					// 锁存长度
					read_len_latch <= read_len_d1;
				end
				// 清零已读拍数
				read_cnt <= ZERO[ADDR_BITS - 1:0];
			end
			S_WAIT:
			begin
				if(wait_cnt >= 16'd200)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					wait_cnt <= wait_cnt + 16'd1;
				end
			end
			S_CHECK_FIFO:
			begin
				// 新帧读请求优先进入 S_ACK。
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				// FIFO 空间足够时发起一个读突发。
				else if(wrusedw < (FIFO_DEPTH - BURST_SIZE - 2))
				begin
					state <= S_READ_BURST;
					rd_burst_len <= BURST_SIZE[BUSRT_BITS - 1:0];
					rd_burst_req <= 1'b1;
				end
			end
			
			S_READ_BURST:
			begin
				if(rd_burst_data_valid)
					rd_burst_req <= 1'b0;
				// 本次突发完成
				if(rd_burst_finish == 1'b1)
				begin
					state <= S_READ_BURST_END;
					// 累计本次突发拍数
					read_cnt <= read_cnt + BURST_SIZE[ADDR_BITS - 1:0];
					// 地址前进到下一突发
					rd_burst_addr <= rd_burst_addr + BURST_SIZE[ADDR_BITS - 1:0];
				end     
			end
			S_READ_BURST_END:
			begin
				// 新帧读请求优先进入 S_ACK。
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				// 未达到整帧长度则继续读突发，
				// 否则结束本帧读取。
				else if(read_cnt < read_len_latch)
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				state <= S_IDLE;
			end
			default:
				state <= S_IDLE;
		endcase
end
endmodule
