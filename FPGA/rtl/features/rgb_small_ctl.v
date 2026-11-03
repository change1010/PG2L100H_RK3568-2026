`timescale 1ns/1ps
// BAR0 字偏移为 0xc0..0xdf，每字 16 字节。
// ARM 后主机缓冲区配置保持到端点复位；STOP 停止新采集并等待已占用缓冲区释放。
// STOP 不保证 PCIe 写事务已全部完成，不能据此释放主机内存。
module rgb_small_ctl (
    input clk, rst_n,
    input reg_wr, input [31:0] reg_addr, reg_data,
    input read_en, input [31:0] read_addr, output reg [127:0] read_data,
    output reg enable, armed, output reg [3:0] period_div,
    output reg [63:0] host_addr, completion,
    output reg [31:0] consumer,
    input [31:0] producer, tx_dropped,
    input [127:0] mem_status, pix_status,
    input pending, tx_busy
);
    reg [31:0] format;
    reg config_error;
    wire addresses_ok=host_addr!=0 && host_addr[11:0]==0 &&
        completion!=0 && completion[11:0]==0 &&
        host_addr+64'd2076672>host_addr &&
        completion+64'd256>completion &&
        (host_addr+64'd2076672<=completion || completion+64'd256<=host_addr);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            enable<=0; armed<=0; period_div<=2; host_addr<=0; completion<=0;
            consumer<=0; format<=0; config_error<=0; read_data<=0;
        end else begin
            if(reg_wr) case(reg_addr)
                'hc1: begin
                    if(!reg_data[0]) enable<=0;
                    else if(format==2 && addresses_ok) begin enable<=1; armed<=1; end
                    else config_error<=1;
                end
                'hc2: if(!armed && reg_data>=1 && reg_data<=15) period_div<=reg_data[3:0];
                'hc3: if(!armed) format<=reg_data;
                'hc4: if(!armed) host_addr[31:0]<=reg_data;
                'hc5: if(!armed) host_addr[63:32]<=reg_data;
                'hc6: if(!armed) completion[31:0]<=reg_data;
                'hc7: if(!armed) completion[63:32]<=reg_data;
                // 消费者只能释放已发布且尚未归还的序号。
                'hc8: if((reg_data-consumer)<=(producer-consumer)) consumer<=reg_data;
                default: begin end
            endcase
            if(read_en) begin
                read_data<=0;
                case(read_addr)
                    'hc0: read_data<={32'd1,32'd691200,32'd4,32'h52474231};
                    'hc1: read_data<={96'd0,27'd0,config_error,tx_busy,pending,armed,enable};
                    'hc2: read_data<={124'd0,period_div};
                    'hc3: read_data<={32'd1,32'd1920,32'h01680280,format};
                    'hc4: read_data<={96'd0,host_addr[31:0]};
                    'hc5: read_data<={96'd0,host_addr[63:32]};
                    'hc6: read_data<={96'd0,completion[31:0]};
                    'hc7: read_data<={96'd0,completion[63:32]};
                    'hc8: read_data<={96'd0,consumer};
                    'hc9: read_data<={64'd0,tx_dropped,producer};
                    'hca: read_data<=mem_status;
                    'hcb: read_data<=pix_status;
                    'hcc: read_data<={32'd3,32'd692224,32'd21600,32'h05000000};
                    default: read_data<=0;
                endcase
            end
        end
    end
endmodule
