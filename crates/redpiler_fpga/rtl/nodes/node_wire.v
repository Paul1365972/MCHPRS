module node_wire #(
    parameter [3:0] STRENGTH = 0,
    parameter STALE = 0
) (
    input wire clock,
    input wire [3:0] input_strength,
    input wire changed,
    output reg [3:0] strength = STRENGTH
);
    wire update;

    always @(posedge clock)
        if (update) strength <= input_strength;

    node_settling #(.STALE(STALE)) settling (.clock(clock), .changed(changed), .update(update));
endmodule
