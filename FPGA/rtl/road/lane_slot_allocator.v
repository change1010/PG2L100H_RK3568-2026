`ifndef LANE_SLOT_ALLOCATOR_V
`define LANE_SLOT_ALLOCATOR_V
// 同拍空槽分配：并行屏蔽流水中预订的槽，四组平衡选择最低编号。
// 保持原有0..15优先级与31无槽哨兵，不改变候选进入/写回拍数。
module lane_slot_allocator #(
    parameter MASK_INPUTS=0
)(
    input [15:0] alive,
    input [3:0] pending,
    input [19:0] reserved,
    input [63:0] reserved_masks,
    output [4:0] slot,
    output [15:0] slot_mask
);
    wire [15:0] available;
    genvar k;
    generate for(k=0;k<16;k=k+1) begin: masks
        if(MASK_INPUTS) begin: from_masks
            assign available[k]=!alive[k] &&
                !(pending[0]&&reserved_masks[k])&&!(pending[1]&&reserved_masks[16+k])&&
                !(pending[2]&&reserved_masks[32+k])&&!(pending[3]&&reserved_masks[48+k]);
        end else begin: from_indices
            assign available[k]=!alive[k] &&
                !(pending[0] && reserved[0+:5]==k) &&
                !(pending[1] && reserved[5+:5]==k) &&
                !(pending[2] && reserved[10+:5]==k) &&
                !(pending[3] && reserved[15+:5]==k);
        end
        if(k==0) assign slot_mask[k]=available[k];
        else assign slot_mask[k]=available[k]&&!(|available[k-1:0]);
    end endgenerate
    wire [3:0] group_valid;
    wire [1:0] group_index[0:3];
    generate for(k=0;k<4;k=k+1) begin: groups
        wire [3:0] v=available[k*4+:4];
        assign group_valid[k]=|v;
        assign group_index[k]=v[0]?2'd0:v[1]?2'd1:v[2]?2'd2:2'd3;
    end endgenerate
    wire low_valid=group_valid[0]||group_valid[1];
    wire high_valid=group_valid[2]||group_valid[3];
    wire [2:0] low_slot=group_valid[0]?{1'b0,group_index[0]}:{1'b1,group_index[1]};
    wire [2:0] high_slot=group_valid[2]?{1'b0,group_index[2]}:{1'b1,group_index[3]};
    generate if(MASK_INPUTS) begin: mask_encode
        // 已选独热位并行编码，避免组内索引再经过三级优先MUX。
        wire none_available=!(|available);
        assign slot={5{none_available}}|{1'b0,|(slot_mask&16'hff00),
            |(slot_mask&16'hf0f0),|(slot_mask&16'hcccc),|(slot_mask&16'haaaa)};
    end else begin: index_encode
        assign slot=low_valid?{2'b00,low_slot}:high_valid?{2'b01,high_slot}:5'd31;
    end endgenerate
endmodule
`endif
