// A command is [code, argument u64]. Once no tick is running the board carries it out and answers
// [status, tick count, payload]; between commands it paces its own ticks by the last run command.
// Whether a tick may start is registered to keep the pacing off the tick counter's path, so the
// circuit only rests once that register has dropped as well.
module protocol_controller #(
    parameter NODE_COUNT = 1,
    parameter STATE_BYTES = 1,
    parameter APPEARANCE_BYTES = 1,
    parameter [63:0] CIRCUIT_ID = 0,
    parameter CLOCKS_PER_BIT = 25
) (
    input wire clock,
    input wire serial_receive,
    input wire clear_fault,
    output wire serial_transmit,
    output wire ticking,
    output wire halted,
    output wire [63:0] tick_count,
    output reg [3:0] fault_code = 0
);
    localparam [7:0] STATUS = 0, INPUT = 1, PAUSE = 2, RUN_UNLIMITED = 3, RUN_TICKS = 4,
        RUN_PERIODIC = 5, READ_APPEARANCE = 6, READ_STATES = 7;
    localparam [3:0] COMMAND_FAULT = 4, INPUT_FAULT = 5, OVERRUN_FAULT = 6;
    localparam COMMAND_BYTES = 9, HEADER_BYTES = 9, IDENTITY_BYTES = 8;
    localparam POSITION_WIDTH =
        $clog2(HEADER_BYTES + IDENTITY_BYTES + STATE_BYTES + APPEARANCE_BYTES + 3);

    wire received, engine_busy, tick_started, sending;
    wire [COMMAND_BYTES * 8 - 1:0] command;
    wire [3:0] receive_fault;
    wire [POSITION_WIDTH - 1:0] position;
    wire [(STATE_BYTES + APPEARANCE_BYTES) * 8 - 1:0] readback;

    reg pending = 0;
    reg [7:0] code = STATUS;
    reg [63:0] argument = 0;
    reg [7:0] mode = PAUSE;
    reg [63:0] target = 0;
    reg [63:0] period = 1;
    reg [63:0] clocks_left = 0;
    reg [31:0] ticks_due = 0;
    reg allowed = 0;
    reg reading = 0;
    reg [7:0] answered = STATUS;
    reg [HEADER_BYTES * 8 - 1:0] header = 0;
    reg [POSITION_WIDTH - 1:0] length = 0;

    wire [3:0] status = code > READ_STATES ? COMMAND_FAULT
        : code == INPUT && argument[30:0] >= NODE_COUNT ? INPUT_FAULT
        : 4'd0;
    wire executing = pending && !engine_busy && !allowed && !sending;
    wire period_elapsed = mode == RUN_PERIODIC && clocks_left == 0;
    wire tick_paid = mode == RUN_PERIODIC && tick_started;
    wire [POSITION_WIDTH - 1:0] payload_position = position - HEADER_BYTES;
    wire [7:0] content = position < HEADER_BYTES ? header[position * 8 +: 8]
        : answered == STATUS ? CIRCUIT_ID[payload_position * 8 +: 8]
        : readback[(payload_position + (answered == READ_APPEARANCE ? STATE_BYTES : 0)) * 8 +: 8];

    assign ticking = engine_busy;
    assign halted = (mode == PAUSE || mode == RUN_TICKS && tick_count == target) && !engine_busy;

    frame_receiver #(.CONTENT_BYTES(COMMAND_BYTES), .CLOCKS_PER_BIT(CLOCKS_PER_BIT)) receiver (
        .clock(clock), .serial(serial_receive), .received(received), .content(command),
        .fault(receive_fault)
    );
    frame_transmitter #(.POSITION_WIDTH(POSITION_WIDTH), .CLOCKS_PER_BIT(CLOCKS_PER_BIT)) transmitter (
        .clock(clock), .start(executing), .length(length), .content(content),
        .position(position), .busy(sending), .serial(serial_transmit)
    );
    compiled_circuit circuit (
        .clock(clock), .set_input(executing && code == INPUT && status == 0),
        .input_index({1'b0, argument[30:0]}), .input_value(argument[31]),
        .run(allowed), .tick_limit(mode == RUN_TICKS ? target : 64'hffff_ffff_ffff_ffff),
        .busy(engine_busy), .tick_started(tick_started), .tick_count(tick_count),
        .readback(readback)
    );

    function [POSITION_WIDTH - 1:0] payload_bytes(input [7:0] answered_code);
        payload_bytes = answered_code == STATUS ? IDENTITY_BYTES
            : answered_code == READ_APPEARANCE ? APPEARANCE_BYTES
            : answered_code == READ_STATES ? STATE_BYTES
            : 0;
    endfunction

    always @(posedge clock) begin
        allowed <= (mode == RUN_UNLIMITED || mode == RUN_TICKS || mode == RUN_PERIODIC && ticks_due != 0)
            && !pending && !(reading && sending);
        if (clear_fault)
            fault_code <= 0;
        else if (fault_code == 0 && receive_fault != 0)
            fault_code <= receive_fault;
        else if (fault_code == 0 && received && pending)
            fault_code <= OVERRUN_FAULT;
        else if (fault_code == 0 && executing && status != 0)
            fault_code <= status;

        if (received && !pending) begin
            pending <= 1;
            code <= command[7:0];
            argument <= command[71:8];
        end
        if (mode == RUN_PERIODIC)
            clocks_left <= period_elapsed ? period - 1 : clocks_left - 1;
        if (period_elapsed && !tick_paid && ticks_due != 32'hffff_ffff)
            ticks_due <= ticks_due + 1;
        else if (tick_paid && !period_elapsed)
            ticks_due <= ticks_due - 1;

        if (executing) begin
            pending <= 0;
            answered <= code;
            reading <= code == READ_APPEARANCE || code == READ_STATES;
            header <= {tick_count, 4'b0, status};
            length <= HEADER_BYTES + (status == 0 ? payload_bytes(code) : 0);
            if (status == 0 && code >= PAUSE && code <= RUN_PERIODIC) begin
                mode <= code;
                target <= tick_count + argument;
                period <= argument;
                clocks_left <= argument - 1;
                ticks_due <= 0;
            end
        end
    end
endmodule
