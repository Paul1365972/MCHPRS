// One register driving thousands of nodes would slow the clock,
// so the sequencer keeps a copy per group of nodes.
// Synthesis merges equal registers even when they are marked to keep;
// `keep_hierarchy` makes it keep this module, and with it every copy, as it is.
(* keep_hierarchy *)
module firing_copy (
    input logic clock,
    input logic beginning,
    output logic [pkg::PRIORITIES-1:0] firing = 0
);
    always_ff @(posedge clock) firing <= {firing[pkg::PRIORITIES-2:0], beginning};
endmodule
