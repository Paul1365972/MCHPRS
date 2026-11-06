// Carries each pulse from the source clock's domain into the destination clock's as a one-clock
// pulse. Pulses must be at least three destination clocks apart.
module pulse_crossing (
    input wire source_clock,
    input wire pulse,
    input wire destination_clock,
    output reg crossed = 0
);
    reg toggle = 0;
    reg previous_toggle = 0;
    wire toggle_seen;

    synchronizer synchronizer (
        .clock(destination_clock), .signal(toggle), .synchronized(toggle_seen)
    );

    always @(posedge source_clock)
        if (pulse)
            toggle <= !toggle;

    always @(posedge destination_clock) begin
        previous_toggle <= toggle_seen;
        crossed <= toggle_seen != previous_toggle;
    end
endmodule
