use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler_fpga::layout::ReadbackScope;
use mchprs_redpiler_fpga::verilog::Design;
use mchprs_ril::Module;

const REPEATERS: usize = 7000;
// Verilator's default `--preproc-token-limit`.
const VERILATOR_LINE_TOKEN_LIMIT: usize = 40_000;

#[test]
fn a_large_circuit_has_no_line_too_long_for_full_readback() {
    let circuit = large_circuit(ExtractOptions::default());
    assert_within_verilator_line_limit(&circuit, ReadbackScope::All);
}

#[test]
fn a_large_circuit_has_no_line_too_long_for_observed_readback() {
    let options = ExtractOptions {
        ports_only: true,
        ..ExtractOptions::default()
    };
    let circuit = large_circuit(options);
    assert_within_verilator_line_limit(&circuit, ReadbackScope::Observed);
}

fn large_circuit(options: ExtractOptions) -> Circuit {
    let repeaters = (0..REPEATERS).map(|index| {
        let next = index + 1;
        format!("%repeater_{index} = repeater(delay=1) -> [%repeater_{next}:0]")
    });
    let repeaters: Vec<_> = repeaters.collect();
    let module = Module::parse(&format!(
        "circuit @long {{ %lever = lever -> [%repeater_0:0] {} %repeater_{REPEATERS} = lamp }}",
        repeaters.join(" ")
    ));
    let mut graph = module.expect("the circuit parses").circuits()[0].graph();
    options.observe(&mut graph);
    Circuit::new(graph).expect("the circuit is valid")
}

fn assert_within_verilator_line_limit(circuit: &Circuit, scope: ReadbackScope) {
    let design = Design::new(circuit, scope, 0);
    let design = design.expect("the circuit is supported");
    let tokens = |line: &str| line.split([' ', ',']).count();
    let most_tokens = design.source.module.lines().map(tokens).max();
    assert!(most_tokens < Some(VERILATOR_LINE_TOKEN_LIMIT));
}
