use crate::{chosen_passes, extract_schematic, run_passes};
use mchprs_ril::ast::Global;
use mchprs_ril::dump_graph;
use mchprs_ril::RILOptions;
use std::path::{Path, PathBuf};
use std::{fs, process};

pub fn compile(input_path: &Path, output_path: &Option<PathBuf>, options: &RILOptions) {
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

    let Some(passes) = chosen_passes(options) else {
        process::exit(1);
    };

    let mut result = String::new();
    for (name, global) in &module.globals {
        let graph = match global {
            Global::Circuit(circuit) => {
                module.get_graph(circuit, options.compile_options().observed)
            }
            Global::Schematic(schematic) => {
                match extract_schematic(input_path, &schematic.path, &options.compile_options()) {
                    Some(graph) => graph,
                    None => process::exit(1),
                }
            }
            Global::Test(_) => {
                eprintln!("error: tests are not supported in this context");
                process::exit(1);
            }
            Global::BackendCircuit(_) => todo!("backend circuit compilation"),
        };

        let result_graph = run_passes(&passes, graph, options.print_after_all);
        result.push('\n');
        dump_graph(&mut result, &result_graph, name).unwrap();
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
