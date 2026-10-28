// One copy of the firing bits, which tell the nodes which priority fires in this clock.
//
// The firing bits reach every node, and one register driving thousands of nodes would slow the
// clock. So the sequencer keeps one copy per 64 nodes. The copies hold equal values, and synthesis
// merges equal registers into one even when they are marked to keep. `keep_hierarchy` stops that:
// synthesis must keep this module, and with it every copy, as it is.
(* keep_hierarchy *)
module firing_copy (
    input logic clock,
    input logic beginning,
    output logic [pkg::PRIORITIES-1:0] firing = 0
);
    always_ff @(posedge clock) firing <= {firing[pkg::PRIORITIES-2:0], beginning};
endmodule
