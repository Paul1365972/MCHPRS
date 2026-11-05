module node_comparator #(
    parameter SUBTRACT = 0,
    parameter HAS_FAR_INPUT = 0,
    parameter [3:0] FAR_INPUT = 0,
    parameter FACING_DIODE = 0,
    parameter [3:0] STRENGTH = 0,
    parameter [3:0] PREVIOUS_DESIRED = 0,
    parameter PENDING = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire [3:0] firing,
    input wire [3:0] input_strength,
    input wire [3:0] side_strength,
    output reg [3:0] strength = STRENGTH,
    output wire countdown,
    output wire [3:0] next_signal
);
    reg [3:0] previous_desired = PREVIOUS_DESIRED;
    wire due, pending;
    wire fire = due && (FACING_DIODE ? firing[2] : firing[3]);
    wire [3:0] rear = HAS_FAR_INPUT && input_strength < 4'd15 ? FAR_INPUT : input_strength;
    wire [3:0] desired = rear < side_strength ? 4'd0 : SUBTRACT ? rear - side_strength : rear;
    assign next_signal = fire ? previous_desired : strength;

    always @(posedge clock) begin
        previous_desired <= desired;
        strength <= next_signal;
    end

    node_timer #(.DELAY(1), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick), .fire(fire),
        .schedule(!pending && desired != next_signal),
        .countdown(countdown), .due(due), .pending(pending)
    );
endmodule
