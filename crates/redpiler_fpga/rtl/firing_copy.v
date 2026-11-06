// One group's copy of the sequencer's firing bits. Synthesis merges identical registers even when
// they are marked to keep, so each copy keeps its own hierarchy instead.
(* keep_hierarchy *)
module fpga_firing_copy (
    input wire clock,
    input wire beginning,
    output reg [3:0] firing = 0
);
    always @(posedge clock)
        firing <= {firing[2:0], beginning};
endmodule
