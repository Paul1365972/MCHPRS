use crate::process::ToolError;
use anyhow::Context;
use mchprs_simulation::backend::CompileError;
use tempfile::TempDir;

pub fn create() -> Result<TempDir, CompileError> {
    let directory = tempfile::Builder::new().prefix("mchprs-fpga-").tempdir();
    let directory = directory.context("could not create the build directory");
    directory.map_err(CompileError::Failed)
}

pub fn keep_if_a_tool_failed(directory: &mut TempDir, error: CompileError) -> CompileError {
    let CompileError::Failed(error) = error else {
        return error;
    };
    let a_tool_failed = error.chain().any(|cause| {
        matches!(
            cause.downcast_ref::<ToolError>(),
            Some(ToolError::Failed { .. } | ToolError::TimedOut { .. } | ToolError::Io { .. })
        )
    });
    if !a_tool_failed {
        return CompileError::Failed(error);
    }
    directory.disable_cleanup(true);
    let kept = format!("build files kept in {}", directory.path().display());
    CompileError::Failed(error.context(kept))
}
