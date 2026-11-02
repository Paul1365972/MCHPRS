use mchprs_redpiler::backend::Backend;
use mchprs_redpiler_direct::DirectBackend;

pub static BACKENDS: &[&dyn Backend] = &[&DirectBackend];

pub fn named(name: &str) -> Result<&'static dyn Backend, String> {
    BACKENDS
        .iter()
        .copied()
        .find(|backend| backend.name() == name)
        .ok_or_else(|| {
            let names: Vec<_> = BACKENDS.iter().map(|backend| backend.name()).collect();
            format!(
                "unknown engine `{name}` (expected one of: {})",
                names.join(", ")
            )
        })
}
