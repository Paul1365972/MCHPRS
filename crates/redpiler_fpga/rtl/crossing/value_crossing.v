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
    reg [WIDTH - 1:0] held = 0;
    wire arrived;

    pulse_crossing arrival (
        .source_clock(source_clock), .pulse(update), .destination_clock(destination_clock),
        .crossed(arrived)
    );

    always @(posedge source_clock)
        if (update)
            held <= value;

    always @(posedge destination_clock) begin
        updated <= arrived;
        if (arrived)
            crossed <= held;
    end
endmodule
