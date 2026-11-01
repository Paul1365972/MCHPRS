use crate::compile::{compile_input, stage_lines, StageNames};
use crate::input::{read_world, Input};
use crate::numbers::{counted, grouped};
use crate::{Error, Outcome};
use anstream::println;
use mchprs_redpiler::compile::StageReport;
use mchprs_redpiler::graph::Graph;
use mchprs_ril::{Input as TestInput, Module, Printed, Test};
use owo_colors::OwoColorize as _;
use std::ffi::OsStr;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

/// Recursively search for ril files starting from `path` and collect into `paths`.
fn search_path(path: PathBuf, paths: &mut Vec<PathBuf>) -> io::Result<()> {
    if path.metadata()?.is_dir() {
        for dir_entry in path.read_dir()? {
            search_path(dir_entry?.path(), paths)?;
        }
    } else if path.extension() == Some(OsStr::new("ril")) {
        paths.push(path);
    }
    Ok(())
}

#[derive(Default)]
struct Tally {
    passed: usize,
    updated: usize,
    failed: usize,
    broken_modules: usize,
}

pub fn run_tests(path: &Path, update: bool, print_after_all: bool) -> Outcome {
    let test_root = if path.is_dir() {
        path
    } else {
        path.parent().unwrap_or(path)
    };
    let mut ril_paths = Vec::new();
    search_path(path.to_owned(), &mut ril_paths)
        .map_err(|error| format!("couldn't search {} for tests: {error}", path.display()))?;
    ril_paths.sort();
    println!(
        "Found {}.",
        counted(ril_paths.len() as u64, "RIL test module")
    );

    let mut tally = Tally::default();
    for ril_path in &ril_paths {
        let relative = ril_path.strip_prefix(test_root).unwrap_or(ril_path);
        let module_name = slash_separated(&relative.with_extension(""));
        let loaded = fs::read_to_string(ril_path)
            .map_err(|error| error.to_string())
            .and_then(|source| Module::parse(&source).map_err(|error| error.to_string()));
        let module = match loaded {
            Ok(module) => module,
            Err(error) => {
                eprintln!("error: {}:{error}", slash_separated(relative));
                tally.broken_modules += 1;
                continue;
            }
        };
        let mut results = Vec::new();
        for test in module.tests() {
            let full_name = format!("{module_name}:{}", test.name);
            let graph = match run_test(ril_path, test, print_after_all) {
                Ok(graph) => graph,
                Err(error) => {
                    println!("{} {full_name}", "[FAIL]".red());
                    eprintln!("error: {error}");
                    tally.failed += 1;
                    continue;
                }
            };
            let mismatches = test.compare(&graph);
            if mismatches.is_empty() {
                println!("{} {full_name}", "[PASS]".green());
                tally.passed += 1;
            } else if update {
                println!("{} {full_name}", "[UPDATED]".blue());
                results.push((test, graph));
            } else {
                println!("{} {full_name}", "[FAIL]".red());
                for mismatch in mismatches {
                    if let Some(expected) = mismatch.expected {
                        println!("{}", format!("- {expected}").red());
                    }
                    if let Some(actual) = mismatch.actual {
                        println!("{}", format!("+ {actual}").green());
                    }
                }
                tally.failed += 1;
            }
        }
        if !results.is_empty() {
            let results: Vec<_> = results.iter().map(|(test, graph)| (*test, graph)).collect();
            fs::write(ril_path, module.with_results(&results))
                .map_err(|error| format!("couldn't update {}: {error}", ril_path.display()))?;
            tally.updated += results.len();
        }
    }

    let total = tally.passed + tally.updated + tally.failed;
    if total == 0 && tally.broken_modules == 0 {
        return Err(format!("no tests found at {}", path.display()).into());
    }
    let mut summary = if update {
        format!(
            "{} passed, {} updated, {} failed",
            counted(tally.passed as u64, "test"),
            grouped(tally.updated as u64),
            grouped(tally.failed as u64)
        )
    } else {
        format!(
            "{}/{} passed",
            grouped(tally.passed as u64),
            counted(total as u64, "test")
        )
    };
    match tally.broken_modules {
        0 => {}
        1 => summary.push_str(", 1 module failed to load"),
        broken => summary.push_str(&format!(
            ", {} failed to load",
            counted(broken as u64, "module")
        )),
    }
    println!("{summary}.");
    Ok(tally.failed == 0 && tally.broken_modules == 0)
}

fn run_test(ril_path: &Path, test: &Test, print_after_all: bool) -> Result<Graph, Error> {
    let mut printed = String::new();
    let mut names = StageNames::default();
    let mut inspect = |stage: &StageReport, graph: &Graph| {
        if print_after_all {
            let name = names.next(stage.name);
            printed.push_str(&format!("\n{}\n", Printed::new(graph, &name)));
        }
    };
    let input = match &test.input {
        TestInput::Schematic(schematic) => {
            let path = ril_path
                .parent()
                .unwrap_or(Path::new(""))
                .join(&schematic.path);
            Input::World(read_world(&path)?)
        }
        TestInput::Circuit(circuit) => Input::Circuit(circuit.clone()),
    };
    let options = &test.options;
    let (graph, stages) = compile_input(&input, &options.extract, &options.passes, &mut inspect)?;
    if print_after_all {
        println!("{}{printed}", stage_lines(&stages));
    }
    Ok(graph)
}

fn slash_separated(path: &Path) -> String {
    let components: Vec<_> = path
        .components()
        .map(|component| component.as_os_str().to_string_lossy())
        .collect();
    components.join("/")
}
