input logic clock,
// No tick begins while `hold` is high.
input logic hold,
// `run_start` and `drive_valid` are high for one clock,
// one at a time and only while `hold` and `idle` are high.
input logic run_start,
// `run_mode`, `run_count`, `drive_node` and `drive_input` come from the board clock:
// the circuit reads each in the clock of its strobe alone.
input pkg::mode_t run_mode,
input logic [pkg::COUNT_BITS-1:0] run_count,
// Low from the clock after `drive_valid` until the input has changed its node,
// and while a tick runs.
output logic idle,
output logic [63:0] tick_count,
output logic paused,
input logic drive_valid,
input logic [pkg::NODE_BITS-1:0] drive_node,
input pkg::drive_input_t drive_input,
// The shown bits have been read: what marks events since the last read starts over in this clock.
input logic shown_read,
// Read while `hold` and `idle` are high, when it and `tick_count` hold still.
// Its lowest `circuit_pkg::SHOWN_BITS` bits are the shown bits.
output logic [circuit_pkg::STATE_BITS-1:0] state
