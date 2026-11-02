use crate::{extract_schematic, run_passes};
use anstream::println;
use mchprs_redpiler::graph::Graph;
use mchprs_ril::{Input, Test};
use owo_colors::OwoColorize as _;
use std::{
    ffi::OsStr,
    fs, io,
    path::{Path, PathBuf},
};

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

enum TestOutcome {
    Passed,
    Updated(Graph),
    Failed,
}

fn run_test(test_root: &Path, test_path: &Path, test: &Test, update: bool) -> TestOutcome {
    let graph = match &test.input {
        Input::Schematic(schematic) => {
            match extract_schematic(test_path, &schematic.path, &test.options.extract) {
                Some(graph) => graph,
                None => return TestOutcome::Failed,
            }
        }
        Input::Circuit(circuit) => circuit.graph(&test.options.extract),
    };

    let result_graph = run_passes(&test.options.passes, graph, false);
    let test_path = test_path.strip_prefix(test_root).unwrap_or(test_path);
    let full_name = format!("{}:{}", test_path.with_extension("").display(), test.name);
    let mismatches = test.compare(&result_graph);
    if mismatches.is_empty() {
        println!("{} {}", "[PASS]".green(), full_name);
        TestOutcome::Passed
    } else if update {
        println!("{} {}", "[UPDATED]".blue(), full_name);
        TestOutcome::Updated(result_graph)
    } else {
        println!("{} {}", "[FAIL]".red(), full_name);
        for mismatch in mismatches {
            if let Some(expected) = mismatch.expected {
                println!("{}", format!("- {expected}").red());
            }
            if let Some(actual) = mismatch.actual {
                println!("{}", format!("+ {actual}").green());
            }
        }
        TestOutcome::Failed
    }
}

pub fn run_tests(path: PathBuf, update: bool) -> bool {
    let mut ril_paths = Vec::new();
    let test_root = if path.is_dir() {
        path.as_path()
    } else {
        path.parent().unwrap_or(&path)
    };
    if let Err(error) = search_path(path.clone(), &mut ril_paths) {
        eprintln!(
            "error: failed to discover tests at {}: {error}",
            path.display()
        );
        return false;
    }
    ril_paths.sort();
    println!("Found {} RIL test modules.", ril_paths.len());

    let mut num_passed = 0;
    let mut num_failed = 0;
    let mut num_updated = 0;
    for path in ril_paths {
        let src = match fs::read_to_string(&path) {
            Ok(src) => src,
            Err(error) => {
                eprintln!("error: failed to read {}: {error}", path.display());
                num_failed += 1;
                continue;
            }
        };
        let Some(module) = crate::load_ril(&path, &src) else {
            num_failed += 1;
            continue;
        };
        let mut results = Vec::new();
        for test in module.tests() {
            match run_test(test_root, &path, test, update) {
                TestOutcome::Passed => num_passed += 1,
                TestOutcome::Updated(graph) => results.push((test, graph)),
                TestOutcome::Failed => num_failed += 1,
            }
        }
        if !results.is_empty() {
            let results: Vec<_> = results.iter().map(|(test, graph)| (*test, graph)).collect();
            match fs::write(&path, module.with_results(&results)) {
                Ok(()) => num_updated += results.len(),
                Err(error) => {
                    eprintln!("error: failed to update {}: {error}", path.display());
                    num_failed += results.len();
                }
            }
        }
    }

    if update {
        println!("{num_passed} tests passed, {num_updated} updated, {num_failed} failed.");
    } else {
        println!("{}/{} tests passed.", num_passed, num_passed + num_failed)
    }
    if num_passed + num_updated + num_failed == 0 {
        eprintln!("error: no tests found at {}", path.display());
        return false;
    }
    num_failed == 0
}
