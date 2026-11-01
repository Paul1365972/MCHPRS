use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::passes::PassRegistry;
use mchprs_ril::ast::Global;
use mchprs_ril::dump_graph;
use mchprs_ril::RilOptions;
use std::ffi::OsStr;
use std::path::{Path, PathBuf};
use std::{fs, process};

pub fn compile(input_path: &Path, output_path: &Option<PathBuf>, options: &RilOptions) {
    let (output, extension) = if input_path.extension() == Some(OsStr::new("ril")) {
        (compile_ril(input_path, options).into_bytes(), "out.ril")
    } else {
        (compile_schematic(input_path, options), "circuit")
    };
    let output_path = match output_path {
        Some(path) => path.clone(),
        None => input_path.with_extension(extension),
    };
    if let Err(err) = fs::write(output_path, output) {
        eprintln!("error: failed to write output file: {}", err);
        process::exit(1);
    }
}

fn compile_ril(input_path: &Path, options: &RilOptions) -> String {
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

    let registry = PassRegistry::default();
    let Some(pass_pipeline) = crate::pass_pipeline(&registry, options) else {
        process::exit(1);
    };

    let mut result = String::new();
    for (name, global) in &module.globals {
        let graph = match global {
            Global::Circuit(circuit) => {
                module.get_graph(circuit, options.compile_options().observed)
            }
            Global::Schematic(schematic) => {
                let schematic_path = input_path.parent().unwrap().join(&schematic.path);
                match crate::extract_schematic(&schematic_path, &options.compile_options()) {
                    Some(graph) => graph,
                    None => process::exit(1),
                }
            }
            Global::Test(_) => {
                eprintln!("error: tests are not supported in this context");
                process::exit(1);
            }
            Global::BackendCircuit(_) => {
                eprintln!("error: backend circuits are not supported in this context");
                process::exit(1);
            }
        };

        let result_graph = crate::run_pipeline(&pass_pipeline, graph, options.print_after_all);
        result.push('\n');
        dump_graph(&mut result, &result_graph, name).unwrap();
        result.push('\n');
    }
    result
}

fn compile_schematic(input_path: &Path, options: &RilOptions) -> Vec<u8> {
    let Some(graph) = crate::extract_schematic(input_path, &options.compile_options()) else {
        process::exit(1);
    };
    let registry = PassRegistry::default();
    let Some(pass_pipeline) = crate::pass_pipeline(&registry, options) else {
        process::exit(1);
    };
    let graph = crate::run_pipeline(&pass_pipeline, graph, options.print_after_all);
    Circuit::new(graph)
        .expect("extraction and the passes keep the circuit invariants")
        .to_bytes()
}
