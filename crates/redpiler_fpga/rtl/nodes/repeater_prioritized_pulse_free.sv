// CircuitAnalysis::is_pulse_free selects this module.
// A pending tick implies that the input still differs from the output.
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
    logic schedule;
    logic [DELAY-1:0] next_countdown;
    logic fire;

    assign schedule = !side_on && countdown == 0 && powered != default_on;

    always_comb begin
        next_countdown = countdown;
        if (schedule) next_countdown[DELAY-1] = 1'b1;
        fire = tick_start && next_countdown[0];
    end

    always_ff @(posedge clock) begin
        if (fire && !side_on) powered <= !powered;
        locked <= side_on;
        countdown <= tick_start ? (next_countdown >> 1) : next_countdown;
    end
endmodule
