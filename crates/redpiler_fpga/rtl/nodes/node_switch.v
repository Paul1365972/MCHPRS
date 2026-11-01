module node_switch #(
    parameter POWERED = 0,
    parameter INDEX = 0,
    parameter INDEX_WIDTH = 1
) (
    input wire clock,
    input wire input_nodes,
    input wire [31:0] input_index,
    input wire input_value,
    output reg powered = POWERED,
    output wire [3:0] next_signal
);
    wire selected = input_nodes && input_index[INDEX_WIDTH-1:0] == INDEX;
    wire next_powered = selected ? input_value : powered;
    assign next_signal = {4{next_powered}};

    always @(posedge clock)
        powered <= next_powered;
endmodule
