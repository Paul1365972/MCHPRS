// Runs the serial line on serial_clock, which fixes its baud rate whatever clock runs, and exchanges
// its bytes with clock's domain. A byte to transmit is taken while transmitting is low and held until
// the line's transmitter has loaded it.
module serial_link #(
    parameter CLOCKS_PER_BIT = 25
) (
    input wire clock,
    input wire serial_clock,
    input wire serial_receive,
    output wire serial_transmit,
    output wire [7:0] received_data,
    output wire received,
    output wire framing_error,
    input wire [7:0] transmit_data,
    input wire transmit,
    output wire transmitting
);
    wire [7:0] arrived_data;
    wire arrived, arrived_broken, updated;
    wire [8:0] crossed;

    uart_receiver #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT)) receiver (
        .clock(serial_clock), .serial(serial_receive), .data(arrived_data), .ready(arrived),
        .framing_error(arrived_broken)
    );
    value_crossing #(.WIDTH(9)) reception (
        .source_clock(serial_clock), .update(arrived || arrived_broken),
        .value({arrived_broken, arrived_data}), .destination_clock(clock), .updated(updated),
        .crossed(crossed)
    );
    assign received_data = crossed[7:0];
    assign received = updated && !crossed[8];
    assign framing_error = updated && crossed[8];

    reg [7:0] outgoing = 0;
    reg requested = 0;
    reg loaded = 0;
    wire requested_seen, loaded_seen, line_busy;
    wire load = requested_seen != loaded && !line_busy;

    synchronizer request (.clock(serial_clock), .signal(requested), .synchronized(requested_seen));
    synchronizer acknowledgement (.clock(clock), .signal(loaded), .synchronized(loaded_seen));
    uart_transmitter #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT)) transmitter (
        .clock(serial_clock), .data(outgoing), .start(load), .serial(serial_transmit),
        .busy(line_busy)
    );
    assign transmitting = requested != loaded_seen;

    always @(posedge clock)
        if (transmit) begin
            outgoing <= transmit_data;
            requested <= !requested;
        end

    always @(posedge serial_clock)
        if (load)
            loaded <= requested_seen;
endmodule
