module note_block #(
    parameter logic POWERED
) (
    input logic clock,
    input logic shown_read,
    input logic default_on,
    output logic powered = POWERED,
    output logic played_since_read = 0
);
    logic turned_on;

    assign turned_on = default_on && !powered;

    always_ff @(posedge clock) begin
        powered <= default_on;
        played_since_read <= (played_since_read && !shown_read) || turned_on;
    end
endmodule
