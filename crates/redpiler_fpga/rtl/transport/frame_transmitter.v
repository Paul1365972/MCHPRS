// Sends length content bytes and their CRC-16 as one byte-stuffed frame; content shows the byte at
// position CONTENT_LATENCY clocks after position changes.
module frame_transmitter #(
    parameter POSITION_WIDTH = 8,
    parameter CONTENT_LATENCY = 0,
    parameter CLOCKS_PER_BIT = 25
) (
    input wire clock,
    input wire start,
    input wire [POSITION_WIDTH - 1:0] length,
    input wire [7:0] content,
    output reg [POSITION_WIDTH - 1:0] position = 0,
    output reg busy = 0,
    output wire serial
);
    localparam [7:0] FRAME_END = 8'hc0, ESCAPE = 8'hdb, ESCAPED_END = 8'hdc, ESCAPED_ESCAPE = 8'hdd;
    localparam LATENCY_WIDTH = $clog2(CONTENT_LATENCY + 2);
    localparam [LATENCY_WIDTH - 1:0] SETTLING_CLOCKS = CONTENT_LATENCY;

    wire sending;
    wire [15:0] next_crc;
    reg [7:0] data = 0;
    reg send = 0;
    reg [LATENCY_WIDTH - 1:0] settling = SETTLING_CLOCKS;
    reg loaded = 0;
    reg escaped = 0;
    reg [7:0] current = 0;
    reg [15:0] crc = 16'hffff;
    wire special = current == FRAME_END || current == ESCAPE;

    uart_transmitter #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT)) uart (
        .clock(clock), .data(data), .start(send), .serial(serial), .busy(sending)
    );
    crc16 checksum (.previous(crc), .data(current), .next(next_crc));

    always @(posedge clock) begin
        send <= 0;
        if (!busy) begin
            busy <= start;
            position <= 0;
            settling <= SETTLING_CLOCKS;
            loaded <= 0;
            escaped <= 0;
            crc <= 16'hffff;
        end else if (settling != 0) begin
            settling <= settling - 1;
        end else if (!loaded) begin
            current <= position < length ? content : position == length ? crc[15:8] : crc[7:0];
            loaded <= 1;
        end else if (!sending && !send) begin
            send <= 1;
            if (position == length + 2) begin
                data <= FRAME_END;
                busy <= 0;
            end else if (special && !escaped) begin
                data <= ESCAPE;
                escaped <= 1;
            end else begin
                data <= !special ? current : current == FRAME_END ? ESCAPED_END : ESCAPED_ESCAPE;
                escaped <= 0;
                if (position < length)
                    crc <= next_crc;
                position <= position + 1;
                settling <= SETTLING_CLOCKS;
                loaded <= 0;
            end
        end
    end
endmodule
