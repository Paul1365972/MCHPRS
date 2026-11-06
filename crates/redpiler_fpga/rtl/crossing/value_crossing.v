// Carries value into the destination clock's domain whenever update pulses: crossed holds it from the
// clock in which updated pulses. Updates must be at least three destination clocks apart.
module value_crossing #(
    parameter WIDTH = 1
) (
    input wire source_clock,
    input wire update,
    input wire [WIDTH - 1:0] value,
    input wire destination_clock,
    output reg updated = 0,
    output reg [WIDTH - 1:0] crossed = 0
);
    reg toggle = 0;
    reg [WIDTH - 1:0] held = 0;
    reg previous_toggle = 0;
    wire toggle_seen;

    synchronizer synchronizer (
        .clock(destination_clock), .signal(toggle), .synchronized(toggle_seen)
    );

    always @(posedge source_clock)
        if (update) begin
            held <= value;
            toggle <= !toggle;
        end

    always @(posedge destination_clock) begin
        previous_toggle <= toggle_seen;
        updated <= toggle_seen != previous_toggle;
        if (toggle_seen != previous_toggle)
            crossed <= held;
    end
endmodule
