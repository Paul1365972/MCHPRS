module top;
    // The descriptors that IEEE 1800 opens before the simulation starts.
    localparam int STDIN = 32'h8000_0000;
    localparam int STDOUT = 32'h8000_0001;
    // About 1,500 redstone ticks, so a short counted run ends before the next request is read.
    localparam int RUN_ON_CLOCKS = 4096;

    logic board_clock = 0;
    logic circuit_clock = 0;

    logic received_valid = 0;
    logic [7:0] received = 0;
    logic transmit_valid;
    logic [7:0] transmit;
    int next_byte;
    int answers = 0;

    logic paused;

    // Three circuit clocks for every two board clocks, as `verilator::CIRCUIT_CLOCK_HERTZ` says.
    always #3 board_clock = ~board_clock;
    always #2 circuit_clock = ~circuit_clock;

    controlled_circuit controlled_circuit (
        .board_clock(board_clock), .received_valid(received_valid), .received(received),
        .transmit_valid(transmit_valid), .transmit(transmit), .transmit_ready(1'b1),
        .circuit_clock(circuit_clock), .paused(paused)
    );

    always @(posedge board_clock) begin
        if (transmit_valid) begin
            $fwrite(STDOUT, "%c", transmit);
            if (transmit == pkg::FRAME_END) begin
                $fflush(STDOUT);
                answers++;
            end
        end
    end

    initial begin
        for (int requests = 1; ; requests++) begin
            // A request frame starts and ends with a FRAME_END.
            for (int frame_ends = 0; frame_ends < 2;) begin
                next_byte = $fgetc(STDIN);
                if (next_byte < 0) $finish;
                @(negedge board_clock);
                received = 8'(next_byte);
                received_valid = 1;
                @(negedge board_clock);
                received_valid = 0;
                if (received == pkg::FRAME_END) frame_ends++;
            end
            wait (answers == requests);
            for (int clocks = 0; clocks < RUN_ON_CLOCKS && !paused; clocks++) @(posedge board_clock);
        end
    end
endmodule
