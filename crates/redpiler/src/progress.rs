use std::sync::Mutex;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PassStatus {
    pub index: usize,
    pub count: usize,
    pub name: &'static str,
}

#[derive(Default)]
pub struct CompileProgress {
    status: Mutex<Option<PassStatus>>,
}

impl CompileProgress {
    pub fn status(&self) -> Option<PassStatus> {
        *self.status.lock().unwrap()
    }

    pub(crate) fn set(&self, status: Option<PassStatus>) {
        *self.status.lock().unwrap() = status;
    }
}
