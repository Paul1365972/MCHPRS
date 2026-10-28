// An iron trapdoor, an output that follows its input.
module trapdoor #(
    parameter logic POWERED
) (
    input logic clock,
    input logic default_on,
    output logic powered = POWERED
);
    always_ff @(posedge clock) powered <= default_on;
endmodule
