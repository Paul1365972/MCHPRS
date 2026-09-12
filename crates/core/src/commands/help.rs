use super::graph::NodeId;
use super::registry::CommandRegistry;
use itertools::Itertools;
use mchprs_syntax::flag::Flag;

pub(super) struct CommandHelp {
    pub(super) usage: String,
    pub(super) flags: Vec<Flag>,
}

#[derive(Default)]
pub(super) struct Help<'a> {
    pub(super) usages: Vec<String>,
    pub(super) flags: Vec<&'a Flag>,
}

pub(super) fn generate_help<'a>(registry: &'a CommandRegistry, path: &[NodeId]) -> Help<'a> {
    let mut help = Help::default();
    if let Some(&last) = path.last() {
        let prefix = path.iter().map(|&id| registry.graph.node(id).name());
        for (below, command) in registry.commands_below(last) {
            let usage = prefix
                .clone()
                .chain(below)
                .chain([command.usage.as_str()])
                .join(" ");
            help.usages.push(format!("/{}", usage.trim_end()));
            help.flags.extend(&command.flags);
        }
    }
    help.flags = help.flags.into_iter().unique().collect();
    help
}
