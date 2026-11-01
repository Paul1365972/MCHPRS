module node_wire #(
    parameter [3:0] STRENGTH = 0
) (
    input wire clock,
    input wire [3:0] input_strength,
    output reg [3:0] strength = STRENGTH
);
    always @(posedge clock)
        strength <= input_strength;
endmodule
