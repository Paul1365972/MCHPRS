use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::graph::{NodeKind, NodeState};
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_simulation::ports::Role;
use std::fmt::Display;
use std::str::FromStr;

pub trait Value: Copy + PartialEq + Display + FromStr {}

impl<T: Copy + PartialEq + Display + FromStr> Value for T {}

pub trait Fields {
    fn value<T: Value>(&mut self, name: &str, value: &mut T, default: Option<T>);
    fn optional<T: Value>(&mut self, name: &str, value: &mut Option<T>);
    fn flag(&mut self, name: &str, value: &mut bool);
    fn powered(&mut self, power: &mut SignalStrength);
}

pub const PORTS: [(&str, Option<Role>); 3] = [
    ("input", Some(Role::Input)),
    ("output", Some(Role::Output)),
    ("internal", None),
];

macro_rules! node_kinds {
    ($($variant:ident $({ $($field:ident: $value:expr),* $(,)? })? => $name:literal),* $(,)?) => {
        pub const KINDS: &[(&str, NodeKind)] = &[
            $(($name, NodeKind::$variant $({ $($field: $value),* })?)),*
        ];

        pub fn kind_name(kind: NodeKind) -> &'static str {
            match kind {
                $(NodeKind::$variant $({ $($field: _),* })? => $name),*
            }
        }
    };
}

node_kinds! {
    Repeater { delay: 1, prioritized: false } => "repeater",
    Torch => "torch",
    Comparator {
        mode: ComparatorMode::Compare,
        far_input: None,
        prioritized: false,
    } => "comparator",
    Lamp => "lamp",
    Button { press_ticks: 10 } => "button",
    Lever => "lever",
    PressurePlate => "pressure_plate",
    Trapdoor => "trapdoor",
    Wire => "wire",
    Constant => "constant",
    NoteBlock { instrument: Instrument::Harp, note: 0 } => "note_block",
}

pub fn visit(kind: &mut NodeKind, state: &mut NodeState, fields: &mut impl Fields) {
    match kind {
        NodeKind::Repeater { delay, prioritized } => {
            fields.value("delay", delay, None);
            fields.flag("prioritized", prioritized);
            fields.flag("locked", &mut state.repeater_locked);
            fields.powered(&mut state.power);
        }
        NodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        } => {
            fields.value("mode", mode, Some(ComparatorMode::Compare));
            fields.value("power", &mut state.power, Some(SignalStrength::Zero));
            fields.optional("far", far_input);
            fields.flag("prioritized", prioritized);
        }
        NodeKind::Button { press_ticks } => {
            fields.value("press_ticks", press_ticks, Some(10));
            fields.powered(&mut state.power);
        }
        NodeKind::Wire => fields.value("power", &mut state.power, Some(SignalStrength::Zero)),
        NodeKind::Constant => {
            fields.value("power", &mut state.power, Some(SignalStrength::Zero));
            fields.flag("locked", &mut state.repeater_locked);
        }
        NodeKind::NoteBlock { instrument, note } => {
            fields.value("instrument", instrument, Some(Instrument::Harp));
            fields.value("note", note, Some(0));
            fields.powered(&mut state.power);
        }
        NodeKind::PressurePlate => {
            fields.flag("touched", &mut state.plate_touched);
            fields.powered(&mut state.power);
        }
        NodeKind::Torch | NodeKind::Lamp | NodeKind::Lever | NodeKind::Trapdoor => {
            fields.powered(&mut state.power)
        }
    }
}

pub fn visit_tick(tick: &mut PendingTick, order: &mut Option<usize>, fields: &mut impl Fields) {
    fields.value("delay", &mut tick.delay, None);
    fields.value("priority", &mut tick.priority, Some(TickPriority::Normal));
    fields.optional("order", order);
}
