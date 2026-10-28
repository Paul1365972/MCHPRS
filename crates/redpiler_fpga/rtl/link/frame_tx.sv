// Sends an answer frame: the content bytes as they are offered, then the CRC and a FRAME_END,
// with every FRAME_END or ESCAPE inside the frame sent as an escape sequence.
module frame_tx (
    input logic clock,
    input logic content_valid,
    input logic [7:0] content,
    output logic content_ready,
    input logic close,
    output logic closed = 0,
    output logic transmit_valid,
    output logic [7:0] transmit,
    input logic transmit_ready
);
    typedef enum logic [1:0] {CONTENT, CHECKSUM_HIGH, CHECKSUM_LOW, DELIMITER} part_t;

    part_t part = CONTENT;
    // A content byte waits here before anything is computed from it, so no combinational path
    // leads from the content through the byte stuffing back to `content_ready`.
    logic [7:0] held = 0;
    logic held_valid = 0;
    logic [15:0] checksum = 16'hffff;
    logic [15:0] next_checksum;
    logic [7:0] unstuffed;
    logic needs_escape;
    logic escape_sent = 0;
    logic sent;
    logic byte_done;

    always_comb begin
        unique case (part)
            CONTENT: unstuffed = held;
            CHECKSUM_HIGH: unstuffed = checksum[15:8];
            CHECKSUM_LOW: unstuffed = checksum[7:0];
            DELIMITER: unstuffed = pkg::FRAME_END;
        endcase
    end

    assign needs_escape =
        part != DELIMITER && (unstuffed == pkg::FRAME_END || unstuffed == pkg::ESCAPE);

    always_comb begin
        if (!needs_escape) transmit = unstuffed;
        else if (!escape_sent) transmit = pkg::ESCAPE;
        else if (unstuffed == pkg::FRAME_END) transmit = pkg::ESCAPED_END;
        else transmit = pkg::ESCAPED_ESCAPE;
    end

    assign content_ready = !held_valid;
    assign transmit_valid = part != CONTENT || held_valid;
    assign sent = transmit_valid && transmit_ready;
    assign byte_done = sent && (!needs_escape || escape_sent);

    crc16 crc (.checksum(checksum), .data(held), .next(next_checksum));

    always_ff @(posedge clock) begin
        closed <= byte_done && part == DELIMITER;
        if (sent) escape_sent <= needs_escape && !escape_sent;
        if (content_valid && content_ready) begin
            held <= content;
            held_valid <= 1;
        end
        if (byte_done && part == CONTENT) begin
            held_valid <= 0;
            checksum <= next_checksum;
        end
        unique case (part)
            CONTENT: if (close && !closed && !content_valid && !held_valid) part <= CHECKSUM_HIGH;
            CHECKSUM_HIGH: if (byte_done) part <= CHECKSUM_LOW;
            CHECKSUM_LOW: if (byte_done) part <= DELIMITER;
            DELIMITER: if (byte_done) begin
                part <= CONTENT;
                checksum <= 16'hffff;
            end
        endcase
    end
endmodule
