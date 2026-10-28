// A note block: it plays when its input turns on. `played` holds that until the host reads it.
module note_block #(
    parameter logic POWERED
) (
    input logic clock,
    input logic shown_read,
    input logic default_on,
    output logic powered = POWERED,
    output logic played = 0
);
    logic turned_on;

    assign turned_on = default_on && !powered;

    always_ff @(posedge clock) begin
        powered <= default_on;
        played <= (played && !shown_read) || turned_on;
    end
endmodule
