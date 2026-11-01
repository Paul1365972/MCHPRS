module node_trapdoor #(
    parameter POWERED = 0
) (
    input wire clock,
    input wire input_power,
    output reg powered = POWERED
);
    always @(posedge clock)
        powered <= input_power;
endmodule
