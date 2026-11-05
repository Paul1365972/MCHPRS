// Draws a grid of cells centred on a 640 by 480 picture at 60 Hz with a 25 MHz pixel rate. `lit`
// and OCCUPIED hold one bit per cell, row by row from the top left. `lit` is copied into flip-flops
// before any logic reads it, so placement need not pull the circuit towards the screen. Colour and
// syncs are registered together, one pixel behind the beam counters.
module vga_screen #(
    parameter COLUMNS = 1,
    parameter ROWS = 1,
    parameter CELL_PIXELS = 1,
    parameter [COLUMNS * ROWS - 1:0] OCCUPIED = 0
) (
    input wire clock,
    input wire [COLUMNS * ROWS - 1:0] lit,
    output reg [11:0] color = 0,
    output reg horizontal_sync = 1,
    output reg vertical_sync = 1
);
    localparam CLOCKS_PER_PIXEL = 4;
    localparam WIDTH = 640, LINE_PIXELS = 800, HORIZONTAL_SYNC_START = 656, HORIZONTAL_SYNC_END = 752;
    localparam HEIGHT = 480, FRAME_LINES = 525, VERTICAL_SYNC_START = 490, VERTICAL_SYNC_END = 492;
    localparam GRID_WIDTH = COLUMNS * CELL_PIXELS, LEFT = (WIDTH - GRID_WIDTH) / 2;
    localparam GRID_HEIGHT = ROWS * CELL_PIXELS, TOP = (HEIGHT - GRID_HEIGHT) / 2;
    localparam CELL_WIDTH = COLUMNS * ROWS > 1 ? $clog2(COLUMNS * ROWS) : 1;
    localparam OFFSET_WIDTH = $clog2(CELL_PIXELS + 1);
    localparam [11:0] LIT_COLOR = 12'hfd5, UNLIT_COLOR = 12'h431, BACKGROUND_COLOR = 12'h000;

    reg [COLUMNS * ROWS - 1:0] shown = 0;
    reg [CLOCKS_PER_PIXEL - 1:0] pixel_phase = 1;
    reg [9:0] x = 0, y = 0;
    reg [OFFSET_WIDTH - 1:0] column_offset = 0, row_offset = 0;
    reg [CELL_WIDTH - 1:0] beam_cell = 0, row_first_cell = 0;

    wire pixel = pixel_phase[CLOCKS_PER_PIXEL - 1];
    wire line_end = x == LINE_PIXELS - 1;
    wire frame_end = y == FRAME_LINES - 1;
    wire in_columns = x - LEFT < GRID_WIDTH;
    wire in_rows = y - TOP < GRID_HEIGHT;
    wire cell_column_end = column_offset == CELL_PIXELS - 1;
    wire cell_row_end = row_offset == CELL_PIXELS - 1;
    wire [CELL_WIDTH - 1:0] next_row_first_cell = frame_end ? 0
        : in_rows && cell_row_end ? row_first_cell + COLUMNS
        : row_first_cell;

    always @(posedge clock) begin
        shown <= lit;
        pixel_phase <= {pixel_phase[CLOCKS_PER_PIXEL - 2:0], pixel};
        if (pixel) begin
            color <= !(in_columns && in_rows && OCCUPIED[beam_cell]) ? BACKGROUND_COLOR
                : shown[beam_cell] ? LIT_COLOR
                : UNLIT_COLOR;
            horizontal_sync <= x < HORIZONTAL_SYNC_START || x >= HORIZONTAL_SYNC_END;
            vertical_sync <= y < VERTICAL_SYNC_START || y >= VERTICAL_SYNC_END;
            x <= line_end ? 0 : x + 1;
            if (line_end) begin
                y <= frame_end ? 0 : y + 1;
                row_offset <= frame_end || !in_rows || cell_row_end ? 0 : row_offset + 1;
                row_first_cell <= next_row_first_cell;
                beam_cell <= next_row_first_cell;
                column_offset <= 0;
            end else if (in_columns) begin
                column_offset <= cell_column_end ? 0 : column_offset + 1;
                if (cell_column_end)
                    beam_cell <= beam_cell + 1;
            end
        end
    end
endmodule
