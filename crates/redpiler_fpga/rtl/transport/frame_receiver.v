// Frames are byte-stuffed (0xc0 ends a frame, 0xdb escapes) and end with a CRC-16 of their content.
// Only frames of exactly CONTENT_BYTES are received; content holds the latest bytes, first byte lowest.
module frame_receiver #(
    parameter CONTENT_BYTES = 1
) (
    input wire clock,
    input wire [7:0] data,
    input wire ready,
    input wire framing_error,
    output reg received = 0,
    output wire [CONTENT_BYTES * 8 - 1:0] content,
    output reg [3:0] fault = 0
);
    localparam [7:0] FRAME_END = 8'hc0, ESCAPE = 8'hdb, ESCAPED_END = 8'hdc, ESCAPED_ESCAPE = 8'hdd;
    localparam [3:0] FRAMING_FAULT = 1, CHECKSUM_FAULT = 2, LENGTH_FAULT = 3;
    localparam FRAME_BYTES = CONTENT_BYTES + 2;

    wire [15:0] next_crc;
    reg escaping = 0;
    reg broken = 0;
    reg [$clog2(FRAME_BYTES + 2) - 1:0] length = 0;
    reg [15:0] crc = 16'hffff;
    reg [FRAME_BYTES * 8 - 1:0] bytes = 0;
    wire [7:0] unescaped = !escaping ? data : data == ESCAPED_END ? FRAME_END : ESCAPE;

    assign content = bytes[CONTENT_BYTES * 8 - 1:0];

    crc16 checksum (.previous(crc), .data(unescaped), .next(next_crc));

    always @(posedge clock) begin
        received <= 0;
        fault <= 0;
        if (framing_error) begin
            fault <= FRAMING_FAULT;
            broken <= 1;
        end else if (ready && data == FRAME_END) begin
            escaping <= 0;
            broken <= 0;
            length <= 0;
            crc <= 16'hffff;
            if (length == FRAME_BYTES && !broken && crc == 0)
                received <= 1;
            else if (length != 0)
                fault <= length == FRAME_BYTES && !broken ? CHECKSUM_FAULT : LENGTH_FAULT;
        end else if (ready && data == ESCAPE && !escaping) begin
            escaping <= 1;
        end else if (ready) begin
            escaping <= 0;
            if (escaping && data != ESCAPED_END && data != ESCAPED_ESCAPE)
                broken <= 1;
            if (length <= FRAME_BYTES)
                length <= length + 1;
            crc <= next_crc;
            bytes <= {unescaped, bytes[FRAME_BYTES * 8 - 1:8]};
        end
    end
endmodule
