#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Readback<T> {
    Complete(T),
    Incomplete,
}

impl<T> Readback<T> {
    pub fn map<U>(self, f: impl FnOnce(T) -> U) -> Readback<U> {
        match self {
            Readback::Complete(value) => Readback::Complete(f(value)),
            Readback::Incomplete => Readback::Incomplete,
        }
    }
}
