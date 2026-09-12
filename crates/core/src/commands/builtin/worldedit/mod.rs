//! [Worldedit](https://github.com/EngineHub/WorldEdit) and [RedstoneTools](https://github.com/paulikauro/RedstoneTools) implementation

mod clipboard;
mod history;
mod navigation;
mod region;
mod schematic;
mod selection;
mod stack;

use crate::commands::registry::CommandRegistry;

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    navigation::register(registry);
    selection::register(registry);
    region::register(registry);
    stack::register(registry);
    clipboard::register(registry);
    history::register(registry);
    schematic::register(registry);
}
