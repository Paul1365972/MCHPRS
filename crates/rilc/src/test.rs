use anstream::println;
use mchprs_ril::string_replacer::StringReplacer;
use mchprs_ril::{self as ril, RILModule, RILTest};
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
    Updated,
    Failed,
}

fn run_test(
    test_root: &Path,
    test_path: &Path,
    module: &RILModule,
    test: RILTest,
    update: bool,
    test_src: &mut StringReplacer,
) -> TestOutcome {
    let graph = match &test.schematic_path {
        Some(schematic_path) => {
            match crate::extract_schematic(
                &test_path.parent().unwrap().join(schematic_path),
                &test.options.extract,
            ) {
                Some(graph) => graph,
                None => return TestOutcome::Failed,
            }
        }
        None => test.graph,
    };

    let passes = match crate::named_passes(&test.options.passes) {
        Ok(passes) => passes,
        Err(error) => {
            eprintln!("error: {error}");
            return TestOutcome::Failed;
        }
    };
    let result_graph = crate::run_passes(&passes, graph, false);
    let test_path = test_path.strip_prefix(test_root).unwrap_or(test_path);
    let full_name = format!("{}:{}", test_path.with_extension("").display(), test.name);
    if !module.compare_test_result(&test.name, &result_graph) {
        let mut result_ril = String::new();
        ril::dump_graph(&mut result_ril, &result_graph, &test.name).unwrap();
        if update {
            println!("{} {}", "[UPDATED]".blue(), full_name);
            module.update_test(test_src, &test.name, &result_ril);
            TestOutcome::Updated
        } else {
            println!("{} {}", "[FAIL]".red(), full_name);
            println!("Actual RIL:");
            println!("{}", result_ril);
            TestOutcome::Failed
        }
    } else {
        println!("{} {}", "[PASS]".green(), full_name);
        TestOutcome::Passed
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
        let tests = module.get_tests();
        let mut updated = 0;

        let mut src = StringReplacer::new(&src);

        for test in tests {
            match run_test(test_root, &path, &module, test, update, &mut src) {
                TestOutcome::Passed => num_passed += 1,
                TestOutcome::Updated => updated += 1,
                TestOutcome::Failed => num_failed += 1,
            }
        }
        if updated > 0 {
            match fs::write(&path, src.finish().as_ref()) {
                Ok(()) => num_updated += updated,
                Err(error) => {
                    eprintln!("error: failed to update {}: {error}", path.display());
                    num_failed += updated;
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
