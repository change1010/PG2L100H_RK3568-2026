`ifndef LANE_DELAY_TAPS_V
`define LANE_DELAY_TAPS_V
// 两个固定抽头的像素有效拍延时。无效内容由上层有效位屏蔽，数据RAM无需复位。
// DEPTH为8/16/32；TAP>1。地址指向下一次写入位置，预读下一拍需要的抽头。
module lane_delay_taps #(parameter WIDTH=44,DEPTH=32,TAP=4)(
    input clk,rst_n,advance,
    input [WIDTH-1:0] value,
    output reg [WIDTH-1:0] near_value,tail_value
);
    localparam AW=DEPTH==32?5:(DEPTH==16?4:3);
    localparam [AW-1:0] NEAR_OFFSET=DEPTH-TAP+1;
    reg [AW-1:0] write_ptr;
    wire [AW-1:0] near_addr=write_ptr+NEAR_OFFSET;
    wire [AW-1:0] tail_addr=write_ptr+1'b1;
    (* syn_ramstyle = "block_ram" *) reg [WIDTH-1:0] near_mem[0:DEPTH-1];
    (* syn_ramstyle = "block_ram" *) reg [WIDTH-1:0] tail_mem[0:DEPTH-1];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) write_ptr<=0;
        else if(advance) write_ptr<=write_ptr+1'b1;
    end
    always @(posedge clk) begin
        if(advance) begin
            near_mem[write_ptr]<=value;
            tail_mem[write_ptr]<=value;
            near_value<=near_mem[near_addr];
            tail_value<=tail_mem[tail_addr];
        end
    end
endmodule
`endif
