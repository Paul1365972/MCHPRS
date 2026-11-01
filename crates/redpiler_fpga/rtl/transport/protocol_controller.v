module protocol_controller #(
    parameter NODE_COUNT = 1,
    parameter OUTPUT_COUNT = 0,
    parameter STATE_BYTES = 1,
    parameter OUTPUT_BYTES = 1,
    parameter [63:0] CIRCUIT_ID = 0,
    parameter MAX_INPUTS = 256,
    parameter CLOCKS_PER_BIT = 25
) (
    input wire clock,
    input wire serial_receive,
    input wire clear_fault,
    output wire serial_transmit,
    output wire ticking,
    output wire waiting,
    output wire [63:0] tick_count,
    output wire [3:0] fault_code
);
    wire [7:0] receive_data, transmit_data;
    wire receive_ready, receive_error, transmit_start, transmit_busy;

    uart_receiver #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT)) receiver (
        .clock(clock), .serial(serial_receive), .data(receive_data),
        .ready(receive_ready), .framing_error(receive_error)
    );
    uart_transmitter #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT)) transmitter (
        .clock(clock), .data(transmit_data), .start(transmit_start),
        .serial(serial_transmit), .busy(transmit_busy)
    );
    packet_controller #(
        .NODE_COUNT(NODE_COUNT), .OUTPUT_COUNT(OUTPUT_COUNT),
        .STATE_BYTES(STATE_BYTES), .OUTPUT_BYTES(OUTPUT_BYTES), .CIRCUIT_ID(CIRCUIT_ID),
        .MAX_INPUTS(MAX_INPUTS)
    ) controller (
        .clock(clock),
        .receive_data(receive_data), .receive_ready(receive_ready),
        .receive_error(receive_error),
        .transmit_data(transmit_data), .transmit_start(transmit_start),
        .transmit_busy(transmit_busy), .clear_fault(clear_fault),
        .ticking(ticking), .waiting(waiting),
        .tick_count(tick_count), .fault_code(fault_code)
    );
endmodule
