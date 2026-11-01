`timescale 1ns/1ps
// 在 TLP 边界公平仲裁；包头一旦提出，即使停顿也保持发送权。
// 本地超时不能释放尚未发完的包。
module tx_packet_mux4 (
    input clk, rst_n,
    input s_valid, input [127:0] s_data, input s_last, s_user, output s_ready,
    input l_valid, input [127:0] l_data, input l_last, l_user, output l_ready,
    input p_valid, input [127:0] p_data, input p_last, p_user, output p_ready,
    input r_valid, input [127:0] r_data, input r_last, r_user, output r_ready,
    output m_valid, output [127:0] m_data, output m_last, m_user, input m_ready
);
    reg owned;
    reg [1:0] owner, next_owner, selected;
    reg found;
    integer i, candidate;
    wire [3:0] valid={r_valid,p_valid,l_valid,s_valid};
    always @* begin
        selected=owner; found=owned; candidate=0;
        if(!owned) begin
            for(i=3;i>=0;i=i-1) begin
                candidate=(next_owner+i)&3;
                if(valid[candidate]) begin selected=candidate; found=1; end
            end
        end
    end
    assign m_valid=found && valid[selected];
    assign m_data=selected==0 ? s_data : selected==1 ? l_data : selected==2 ? p_data : r_data;
    assign m_last=selected==0 ? s_last : selected==1 ? l_last : selected==2 ? p_last : r_last;
    assign m_user=selected==0 ? s_user : selected==1 ? l_user : selected==2 ? p_user : r_user;
    assign s_ready=found && selected==0 && m_ready;
    assign l_ready=found && selected==1 && m_ready;
    assign p_ready=found && selected==2 && m_ready;
    assign r_ready=found && selected==3 && m_ready;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin owned<=0; owner<=0; next_owner<=0; end
        else if(m_valid) begin
            owner<=selected;
            if(m_ready && m_last) begin owned<=0; next_owner<=selected+1'b1; end
            else owned<=1;
        end
    end
endmodule
