use std::path::Path;
use std::process::Command;

fn git(args: &[&str]) -> Option<String> {
    let output = Command::new("git").args(args).output().ok()?;
    if !output.status.success() {
        return None;
    }
    Some(String::from_utf8(output.stdout).ok()?.trim().to_owned())
}

fn rerun_if_git_file_changed(name: &str) {
    if let Some(path) = git(&["rev-parse", "--git-path", name])
        && Path::new(&path).exists()
    {
        println!("cargo:rerun-if-changed={path}");
    }
}

fn main() {
    let hash = git(&["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown commit".to_owned());
    println!("cargo:rustc-env=GIT_HASH={hash}");
    rerun_if_git_file_changed("HEAD");
    rerun_if_git_file_changed("packed-refs");
    if let Some(branch) = git(&["symbolic-ref", "-q", "HEAD"]) {
        rerun_if_git_file_changed(&branch);
    }
}
