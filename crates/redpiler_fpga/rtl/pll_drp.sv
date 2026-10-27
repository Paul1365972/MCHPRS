// The PLL that makes the circuit clock, with its dynamic reconfiguration port (DRP) open to the
// host. The host sets the clock after routing by writing the PLL's registers (Xilinx XAPP888).
module pll_drp #(
    parameter int OSCILLATOR_HERTZ
) (
    // From the pin itself: nextpnr refuses a buffer before the PLL.
    input logic oscillator,
    input logic board_clock,
    input logic access,
    input logic write,
    input logic [6:0] address,
    input logic [15:0] write_data,
    output logic ready,
    output logic [15:0] read_data,
    // High while the registers of another setting are written (XAPP888).
    input logic reset,
    output logic locked,
    output logic unbuffered
);
    localparam int PLL_OSCILLATOR_HERTZ = 1_000_000_000;
    localparam int MULTIPLIER = PLL_OSCILLATOR_HERTZ / OSCILLATOR_HERTZ;

    logic feedback;

    PLLE2_ADV #(
        .CLKIN1_PERIOD(1.0e9 / OSCILLATOR_HERTZ),
        .CLKFBOUT_MULT(MULTIPLIER),
        .DIVCLK_DIVIDE(1),
        .CLKOUT0_DIVIDE(MULTIPLIER)
    ) pll (
        .CLKIN1(oscillator), .CLKIN2(1'b0), .CLKINSEL(1'b1),
        .CLKFBIN(feedback), .CLKFBOUT(feedback), .CLKOUT0(unbuffered),
        .RST(reset), .PWRDWN(1'b0), .LOCKED(locked),
        .DCLK(board_clock), .DEN(access), .DWE(write), .DADDR(address),
        .DI(write_data), .DO(read_data), .DRDY(ready)
    );
endmodule
