module redstone_wire #(
    parameter logic [3:0] STRENGTH
) (
    input logic clock,
    input logic [3:0] default_strength,
    output logic [3:0] strength = STRENGTH
);
    always_ff @(posedge clock) strength <= default_strength;
endmodule
