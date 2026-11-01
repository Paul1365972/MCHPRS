// TWO_LEVEL requires that the desired output is only ever LOW or HIGH; the comparator then stores
// one bit instead of the strength.
module node_comparator #(
    parameter SUBTRACT = 0,
    parameter HAS_FAR_INPUT = 0,
    parameter [3:0] FAR_INPUT = 0,
    parameter FACING_DIODE = 0,
    parameter TWO_LEVEL = 0,
    parameter [3:0] LOW = 0,
    parameter [3:0] HIGH = 0,
    parameter [3:0] STRENGTH = 0,
    parameter [3:0] PREVIOUS_DESIRED = 0,
    parameter PENDING = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire [3:0] firing,
    input wire [3:0] input_strength,
    input wire [3:0] side_strength,
    output wire high,
    output wire [3:0] strength,
    output wire countdown,
    output wire [3:0] next_signal
);
    reg due = 1'b0;
    wire fire = due && (FACING_DIODE ? firing[2] : firing[3]);
    wire pending = countdown || due && !fire;
    wire [3:0] rear = HAS_FAR_INPUT && input_strength < 4'd15 ? FAR_INPUT : input_strength;
    wire [3:0] desired = rear < side_strength ? 4'd0 : SUBTRACT ? rear - side_strength : rear;

    generate
        if (TWO_LEVEL) begin : two_level
            reg previous_high = PREVIOUS_DESIRED == HIGH;
            reg high_register = STRENGTH == HIGH;
            wire next_high = fire ? previous_high : high_register;
            assign high = high_register;
            assign strength = high_register ? HIGH : LOW;
            assign next_signal = next_high ? HIGH : LOW;
            always @(posedge clock) begin
                previous_high <= desired == HIGH;
                high_register <= next_high;
            end
        end else begin : any_level
            reg [3:0] previous_desired = PREVIOUS_DESIRED;
            reg [3:0] strength_register = STRENGTH;
            assign high = 1'b0;
            assign strength = strength_register;
            assign next_signal = fire ? previous_desired : strength_register;
            always @(posedge clock) begin
                previous_desired <= desired;
                strength_register <= next_signal;
            end
        end
    endgenerate

    always @(posedge clock)
        if (fire) due <= 1'b0;
        else if (begin_tick) due <= countdown;

    node_countdown #(.DELAY(1), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick),
        .schedule(!pending && desired != next_signal), .countdown(countdown)
    );
endmodule
