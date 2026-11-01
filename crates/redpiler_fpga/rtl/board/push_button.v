module push_button #(
    parameter SETTLE_CLOCKS = 1_000_000
) (
    input wire clock,
    input wire button,
    output reg pressed = 0
);
    reg [1:0] synchronized = 0;
    reg stable = 0;
    reg [$clog2(SETTLE_CLOCKS)-1:0] settling = 0;

    always @(posedge clock) begin
        synchronized <= {synchronized[0], button};
        pressed <= 0;
        if (synchronized[1] == stable) begin
            settling <= 0;
        end else if (settling == SETTLE_CLOCKS - 1) begin
            settling <= 0;
            stable <= synchronized[1];
            pressed <= synchronized[1];
        end else begin
            settling <= settling + 1;
        end
    end
endmodule
