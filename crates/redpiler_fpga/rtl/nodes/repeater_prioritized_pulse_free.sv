// CircuitAnalysis::is_pulse_free selects this module.
// A tick sets an unlocked pulse-free repeater to its rear input, and without a pending tick the
// two are equal already.
module repeater_prioritized_pulse_free #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter logic LOCKED = 0,
    parameter int PENDING_DELAY,
    localparam logic [DELAY:0] PENDING_TICK = (DELAY + 1)'(1) << PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic default_on,
    input logic side_on,
    output logic powered = POWERED,
    output logic locked = LOCKED,
    output logic [DELAY-1:0] countdown = PENDING_TICK[DELAY:1]
);
    always_ff @(posedge clock) begin
        if (tick_start && !side_on) powered <= default_on;
        locked <= side_on;
        if (tick_start) countdown <= '0;
        else if (!side_on && powered != default_on) countdown[DELAY-1] <= 1'b1;
    end
endmodule
