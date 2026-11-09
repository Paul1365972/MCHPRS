use mchprs_redpiler::passes::{all, Pass};
use mchprs_syntax::flag::Flag;

pub const PASSES: Flag = Flag::valued(
    "passes",
    "PASSES",
    "Comma-separated passes to run instead of the backend's own",
);

pub fn named(names: &str) -> Result<Vec<Box<dyn Pass>>, UnknownPass> {
    let named = |name| all().into_iter().find(|pass| pass.name() == name);
    names
        .split(',')
        .map(|name| named(name).ok_or_else(|| UnknownPass(name.to_owned())))
        .collect()
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("unknown pass `{0}`")]
pub struct UnknownPass(String);
