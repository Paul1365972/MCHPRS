// A repeater facing a diode ticks at priority 0, in the clock that starts the tick, unless its tick
// ends a pulse, which fires at priority 1 once its countdown has run out. Any other repeater's tick
// priority follows its output. PULSE_FREE requires that the repeater never fires unpowered with its
// input off, so it never starts a pulse.
module node_repeater #(
    parameter DELAY = 1,
    parameter FACING_DIODE = 0,
    parameter POWERED = 0,
    parameter PREVIOUS_INPUT = 0,
    parameter LOCKABLE = 0,
    parameter LOCKED = 0,
    parameter PENDING = 0,
    parameter PULSING = 0,
    parameter PULSE_FREE = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire [3:0] firing,
    input wire input_power,
    input wire side_power,
    output reg powered = POWERED,
    output wire locked,
    output wire [DELAY-1:0] countdown,
    output wire pulsing,
    output wire [3:0] next_signal
);
    reg previous_input = PREVIOUS_INPUT;
    wire fire, tick_pending;
    wire unlocked_fire = fire && !locked;
    wire starts_pulse = !PULSE_FREE && unlocked_fire && !powered && !previous_input;
    wire next_powered = unlocked_fire ? !powered || previous_input : powered;
    wire schedule = starts_pulse || !side_power && !tick_pending && next_powered != input_power;
    assign next_signal = {4{next_powered}};

    always @(posedge clock) begin
        previous_input <= input_power;
        powered <= next_powered;
    end

    generate
        if (FACING_DIODE) begin : facing_diode
            reg pulse_register = PULSING;
            assign pulsing = pulse_register;
            assign fire = pulsing ? countdown == 0 && firing[1] : countdown[0] && firing[0];
            assign tick_pending = (|countdown || pulsing) && !fire;
            always @(posedge clock)
                if (fire) pulse_register <= starts_pulse;
            node_countdown #(.DELAY(DELAY), .PENDING(PENDING)) tick (
                .clock(clock), .begin_tick(begin_tick), .schedule(schedule), .countdown(countdown)
            );
        end else begin : follows_output
            wire due;
            assign pulsing = 1'b0;
            assign fire = due && (powered ? firing[1] : firing[2]);
            node_timer #(.DELAY(DELAY), .PENDING(PENDING)) tick (
                .clock(clock), .begin_tick(begin_tick), .fire(fire), .schedule(schedule),
                .countdown(countdown), .due(due), .pending(tick_pending)
            );
        end

        if (LOCKABLE) begin : lock
            reg lock_register = LOCKED;
            assign locked = lock_register;
            always @(posedge clock)
                lock_register <= side_power;
        end else begin : no_lock
            assign locked = 1'b0;
        end
    endgenerate
endmodule
