// A repeater facing a diode schedules its tick at priority 0 and its pulse at priority 1, so it
// counts them down separately; a priority 0 tick fires in the clock that starts the tick and needs
// no due register. Any other repeater's tick priority follows its output.
module node_repeater #(
    parameter DELAY = 1,
    parameter FACING_DIODE = 0,
    parameter POWERED = 0,
    parameter PREVIOUS_INPUT = 0,
    parameter LOCKABLE = 0,
    parameter LOCKED = 0,
    parameter PENDING = 0,
    parameter PENDING_PULSE = 0,
    parameter STALE = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire [3:0] firing,
    input wire input_power,
    input wire side_power,
    input wire changed,
    output reg powered = POWERED,
    output wire locked,
    output wire [DELAY-1:0] countdown,
    output wire [DELAY-1:0] pulse_countdown,
    output wire [3:0] signal,
    output wire [3:0] next_signal
);
    reg previous_input = PREVIOUS_INPUT;
    reg due = 1'b0;
    wire update;
    wire fire = FACING_DIODE
        ? countdown[0] && firing[0] || due && firing[1]
        : due && (powered ? firing[1] : firing[2]);
    wire unlocked_fire = fire && !locked;
    wire pulse = unlocked_fire && !powered && !previous_input;
    wire next_powered = unlocked_fire ? !powered || previous_input : powered;
    wire pending = FACING_DIODE
        ? |countdown && !fire || |pulse_countdown || due && !fire || pulse
        : |countdown || due && !fire || pulse;
    wire schedule = update && !side_power && !pending && next_powered != input_power;
    assign signal = {4{powered}};
    assign next_signal = {4{next_powered}};

    always @(posedge clock) begin
        previous_input <= input_power;
        powered <= next_powered;
        if (fire) due <= 1'b0;
        else if (begin_tick) due <= FACING_DIODE ? pulse_countdown[0] : countdown[0];
    end

    node_settling #(.STALE(STALE)) settling (.clock(clock), .changed(changed), .update(update));
    node_countdown #(.DELAY(DELAY), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick),
        .schedule(FACING_DIODE ? schedule : schedule || pulse), .countdown(countdown)
    );

    generate
        if (LOCKABLE) begin : lock
            reg lock_register = LOCKED;
            assign locked = lock_register;
            always @(posedge clock)
                if (update) lock_register <= side_power;
        end else begin : no_lock
            assign locked = 1'b0;
        end

        if (FACING_DIODE) begin : facing_diode
            node_countdown #(.DELAY(DELAY), .PENDING(PENDING_PULSE)) pulse_tick (
                .clock(clock), .begin_tick(begin_tick), .schedule(pulse),
                .countdown(pulse_countdown)
            );
        end else begin : follows_output
            assign pulse_countdown = 0;
        end
    endgenerate
endmodule
