use crate::process::ToolError;
use mchprs_simulation::backend::CompileError;
use std::path::PathBuf;
use std::{env, fs, io};

#[derive(Debug)]
pub struct BuildDirectory {
    pub path: PathBuf,
    is_kept: bool,
}

impl BuildDirectory {
    pub fn new() -> io::Result<BuildDirectory> {
        let path = loop {
            let directory_nonce: u32 = rand::random();
            let path = env::temp_dir().join(format!("mchprs-fpga-{directory_nonce:08x}"));
            match fs::create_dir(&path) {
                Ok(()) => break path,
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
                Err(error) => return Err(error),
            }
        };
        Ok(BuildDirectory {
            path,
            is_kept: false,
        })
    }

    pub fn keep_if_a_tool_failed(&mut self, error: CompileError) -> CompileError {
        let CompileError::Failed(error) = error else {
            return error;
        };
        self.is_kept = error.chain().any(|cause| {
            matches!(
                cause.downcast_ref::<ToolError>(),
                Some(ToolError::Failed { .. } | ToolError::TimedOut { .. } | ToolError::Io { .. })
            )
        });
        if !self.is_kept {
            return CompileError::Failed(error);
        }
        CompileError::Failed(error.context(format!("build files kept in {}", self.path.display())))
    }
}

impl Drop for BuildDirectory {
    fn drop(&mut self) {
        if !self.is_kept {
            let _ = fs::remove_dir_all(&self.path);
        }
    }
}
