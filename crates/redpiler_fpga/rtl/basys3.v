module basys3 (
    input wire clock,
    input wire serial_receive,
    output wire serial_transmit,
    output wire [2:0] led,
    output wire [6:0] segment,
    output wire [3:0] anode,
    output wire decimal_point
);
    reg [26:0] heartbeat = 0;
    wire busy;
    wire fault;
    wire [63:0] tick_count;

    always @(posedge clock)
        heartbeat <= heartbeat + 1;

    assign led = {fault, busy, heartbeat[26]};

    circuit_system system (
        .clock(clock), .serial_receive(serial_receive),
        .serial_transmit(serial_transmit), .busy(busy),
        .tick_count(tick_count), .fault(fault)
    );
    seven_segment_display display (
        .clock(clock), .value(tick_count[15:0]), .segment(segment),
        .anode(anode), .decimal_point(decimal_point)
    );
endmodule
