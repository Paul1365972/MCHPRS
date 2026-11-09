use crate::graph::NodeKind;
use mchprs_blocks::blocks::Block;
use mchprs_redstone::wire;
use mchprs_simulation::ports::Role;
use mchprs_syntax::flag::{Flag, FlagValues};

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct WireOutputs {
    pub wire_dots: bool,
    pub wire_crosses: bool,
    pub illegal_wires: bool,
}

impl WireOutputs {
    pub const DOTS: Flag =
        Flag::switch("wire-dot-out", Some('d'), "Treat redstone dots as outputs");
    pub const ILLEGAL_STATES: Flag = Flag::switch(
        "illegal-states-out",
        Some('l'),
        "Treat wires in illegal states as outputs",
    );
    pub const CROSSES: Flag = Flag::switch(
        "wire-cross-out",
        Some('c'),
        "Treat redstone crosses as outputs",
    );
    pub const FLAGS: [Flag; 3] = [
        WireOutputs::DOTS,
        WireOutputs::ILLEGAL_STATES,
        WireOutputs::CROSSES,
    ];

    pub fn from_flags(flags: &FlagValues) -> WireOutputs {
        WireOutputs {
            wire_dots: flags.is_on(&WireOutputs::DOTS),
            wire_crosses: flags.is_on(&WireOutputs::CROSSES),
            illegal_wires: flags.is_on(&WireOutputs::ILLEGAL_STATES),
        }
    }

    pub fn role(&self, block: Block, kind: NodeKind) -> Option<Role> {
        if let Block::RedstoneWire(wire) = block {
            let output = self.wire_dots && wire::is_dot(wire)
                || self.wire_crosses && wire::is_cross(wire)
                || self.illegal_wires && wire::is_illegal(wire);
            return output.then_some(Role::Output);
        }
        kind.traits().role
    }
}
