module node_trapdoor #(
    parameter POWERED = 0,
    parameter STALE = 0
) (
    input wire clock,
    input wire input_power,
    input wire changed,
    output reg powered = POWERED
);
    wire update;

    always @(posedge clock)
        if (update) powered <= input_power;

    node_settling #(.STALE(STALE)) settling (.clock(clock), .changed(changed), .update(update));
endmodule
