// Receives request frames: removes the byte stuffing, collects the content and checks its length
// and CRC. At each closing FRAME_END it reports the frame with its status.
module frame_rx #(
    parameter int CONTENT_BYTES = 1 + pkg::ARGUMENT_BYTES
) (
    input logic clock,
    input logic valid,
    input logic [7:0] data,
    output logic completed = 0,
    output pkg::status_t status = pkg::OK,
    output logic [CONTENT_BYTES * 8 - 1:0] content
);
    localparam int FRAME_BYTES = CONTENT_BYTES + 2;
    localparam int LENGTH_BITS = $clog2(FRAME_BYTES + 2);

    logic [FRAME_BYTES * 8 - 1:0] bytes = 0;
    logic [LENGTH_BITS - 1:0] length = 0;
    logic [15:0] checksum = 16'hffff;
    logic [15:0] next_checksum;
    logic escaping = 0;
    logic misescaped = 0;
    logic [7:0] unstuffed;
    pkg::status_t frame_status;

    assign content = bytes[CONTENT_BYTES * 8 - 1:0];

    always_comb begin
        if (!escaping) unstuffed = data;
        else if (data == pkg::ESCAPED_END) unstuffed = pkg::FRAME_END;
        else unstuffed = pkg::ESCAPE;
    end

    // The CRC over the content followed by its own checksum is 0.
    always_comb begin
        if (misescaped || escaping) frame_status = pkg::BAD_ESCAPE;
        else if (length != LENGTH_BITS'(FRAME_BYTES)) frame_status = pkg::BAD_LENGTH;
        else if (checksum != 0) frame_status = pkg::BAD_CHECKSUM;
        else frame_status = pkg::OK;
    end

    crc16 crc (.checksum(checksum), .data(unstuffed), .next(next_checksum));

    always_ff @(posedge clock) begin
        completed <= 0;
        if (valid && data == pkg::FRAME_END) begin
            completed <= length != 0 || escaping;
            status <= frame_status;
            length <= 0;
            checksum <= 16'hffff;
            escaping <= 0;
            misescaped <= 0;
        end else if (valid && data == pkg::ESCAPE && !escaping) begin
            escaping <= 1;
        end else if (valid) begin
            escaping <= 0;
            if (escaping && data != pkg::ESCAPED_END && data != pkg::ESCAPED_ESCAPE) misescaped <= 1;
            if (length <= LENGTH_BITS'(FRAME_BYTES)) length <= length + 1'b1;
            checksum <= next_checksum;
            bytes <= {unstuffed, bytes[FRAME_BYTES * 8 - 1:8]};
        end
    end
endmodule
