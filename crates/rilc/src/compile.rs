use crate::run_passes;
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::passes::Pass;
use mchprs_ril::Printed;
use std::path::{Path, PathBuf};
use std::{fs, process};

pub struct Compilation<'a> {
    pub extract: &'a ExtractOptions,
    pub passes: &'a [&'a dyn Pass],
    pub print_after_all: bool,
}

pub fn compile(input_path: &Path, output_path: &Option<PathBuf>, compilation: &Compilation) {
    let ril_src = match fs::read_to_string(input_path) {
        Ok(src) => src,
        Err(err) => {
            eprintln!("error: couldn't read `{}`: {}", input_path.display(), err);
            process::exit(1);
        }
    };
    let Some(module) = crate::load_ril(input_path, &ril_src) else {
        process::exit(1);
    };

    if !module.tests().is_empty() {
        eprintln!("error: tests are not supported in this context");
        process::exit(1);
    }
    let mut result = String::new();
    for circuit in module.circuits() {
        let graph = circuit.graph(compilation.extract);
        let result_graph = run_passes(compilation.passes, graph, compilation.print_after_all);
        result.push('\n');
        result.push_str(&Printed::new(&result_graph, &circuit.name).to_string());
        result.push('\n');
    }

    let output_path = if let Some(path) = output_path {
        path.clone()
    } else {
        input_path.with_extension("out.ril")
    };

    if let Err(err) = fs::write(output_path, result) {
        println!("error: failed to write output file: {}", err);
        process::exit(1);
    }
}
