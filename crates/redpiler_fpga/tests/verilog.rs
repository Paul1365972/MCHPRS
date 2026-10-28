use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler_fpga::verilog::Design;
use mchprs_ril::Module;

const REPEATERS: usize = 7000;
const VERILATOR_LINE_TOKEN_LIMIT: usize = 40_000;

#[test]
fn a_large_circuit_has_no_line_too_long_for_verilator() {
    let repeaters = (0..REPEATERS).map(|index| {
        let next = index + 1;
        format!("%repeater_{index} = repeater(delay=1) -> [%repeater_{next}:0]")
    });
    let repeaters: Vec<_> = repeaters.collect();
    let module = Module::parse(&format!(
        "circuit @long {{ %lever = lever -> [%repeater_0:0] {} %repeater_{REPEATERS} = lamp }}",
        repeaters.join(" ")
    ));
    let graph = module.expect("the circuit parses").circuits()[0].graph();
    let circuit = Circuit::new(graph).expect("the circuit is valid");
    let design = Design::new(&circuit, 0);
    let design = design.expect("the circuit is supported");
    let tokens = |line: &str| line.split([' ', ',']).count();
    let most_tokens = design.source.module.lines().map(tokens).max();
    assert!(most_tokens < Some(VERILATOR_LINE_TOKEN_LIMIT));
}
