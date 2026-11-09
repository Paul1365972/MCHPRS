use crate::compile::{compile_test, stage_lines, StageNames};
use crate::input::{read_world, Loaded};
use crate::numbers::{counted, grouped};
use crate::{Error, Outcome};
use anstream::println;
use anyhow::{anyhow, Context};
use mchprs_redpiler::graph::Graph;
use mchprs_ril::{Input, Module, Printed, Test};
use owo_colors::OwoColorize as _;
use std::ffi::OsStr;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

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
    let ril_paths = files_under(path, "ril")
        .with_context(|| format!("couldn't search {} for tests", path.display()))?;
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
                    eprintln!("error: {error:#}");
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
            fs::write(ril_path, module.with_results(&results))
                .with_context(|| format!("couldn't update {}", ril_path.display()))?;
            tally.updated += results.len();
        }
    }

    let total = tally.passed + tally.updated + tally.failed;
    if total == 0 && tally.broken_modules == 0 {
        return Err(anyhow!("no tests found at {}", path.display()));
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
    let mut inspect = |stage, graph: &Graph| {
        if print_after_all {
            let name = names.next(stage);
            printed.push_str(&format!("\n{}\n", Printed::new(graph, &name)));
        }
    };
    let input = match &test.input {
        Input::Schematic(schematic) => {
            let path = ril_path
                .parent()
                .unwrap_or(Path::new(""))
                .join(&schematic.path);
            Loaded::World(read_world(&path)?)
        }
        Input::Circuit(circuit) => Loaded::Circuit(circuit.clone()),
    };
    let (graph, stages) = compile_test(&input, &test.options, &mut inspect)?;
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

fn files_under(path: &Path, extension: &str) -> io::Result<Vec<PathBuf>> {
    fn search(path: PathBuf, extension: &OsStr, files: &mut Vec<PathBuf>) -> io::Result<()> {
        if path.metadata()?.is_dir() {
            for entry in path.read_dir()? {
                search(entry?.path(), extension, files)?;
            }
        } else if path.extension() == Some(extension) {
            files.push(path);
        }
        Ok(())
    }
    let mut files = Vec::new();
    search(path.to_owned(), OsStr::new(extension), &mut files)?;
    files.sort();
    Ok(files)
}
