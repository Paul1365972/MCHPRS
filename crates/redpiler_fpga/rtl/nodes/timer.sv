// Countdown bit k falls due on the (k+1)th tick boundary.
// Scheduling on `tick_start` comes from the previous tick's last phase,
// so it enters the top bit before the shift.
// Snapshots read the countdown between ticks.
module timer #(
    parameter int DELAY,
    parameter int PENDING_DELAY,
    parameter int FIRES = pkg::FIRES_WHEN_DUE,
    localparam logic [DELAY:0] PENDING_TICK = (DELAY + 1)'(1) << PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic schedule,
    // High in the clock of the node's priority; `fire` is high when the node itself fires.
    input logic firing,
    output logic [DELAY-1:0] countdown = PENDING_TICK[DELAY:1],
    output logic fire,
    output logic pending
);
    logic due = 0;
    logic [DELAY-1:0] next_countdown;

    always_comb begin
        case (FIRES)
            pkg::FIRES_IN_PRIORITY: fire = firing;
            default: fire = firing && due;
        endcase
        next_countdown = countdown;
        if (schedule) next_countdown[DELAY-1] = 1'b1;
    end
    assign pending = |countdown || due;

    always_ff @(posedge clock) begin
        if (tick_start) begin
            countdown <= next_countdown >> 1;
            due <= next_countdown[0];
        end else begin
            countdown <= next_countdown;
            // Keeping `due` in the condition avoids a shared control set across distant nodes.
            if (firing && due) due <= 1'b0;
        end
    end
endmodule
