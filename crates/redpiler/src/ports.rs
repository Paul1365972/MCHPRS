use crate::graph::NodeKind;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_redstone::wire;
use rustc_hash::FxHashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Role {
    Input,
    Output,
}

/// Which parts of the circuit the caller sees change. The optimizer may remove anything else.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Observed {
    #[default]
    Everything,
    /// Every component, and dust only where it is a port
    Components,
    Ports,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ports {
    conventional: bool,
    wire_dots: bool,
    wire_crosses: bool,
    illegal_wires: bool,
    declared: FxHashMap<BlockPos, Role>,
}

impl Ports {
    pub fn conventional() -> Ports {
        Ports {
            conventional: true,
            ..Ports::none()
        }
    }

    pub fn none() -> Ports {
        Ports {
            conventional: false,
            wire_dots: false,
            wire_crosses: false,
            illegal_wires: false,
            declared: FxHashMap::default(),
        }
    }

    pub fn with_wire_dots(mut self) -> Ports {
        self.wire_dots = true;
        self
    }

    pub fn with_wire_crosses(mut self) -> Ports {
        self.wire_crosses = true;
        self
    }

    pub fn with_illegal_wires(mut self) -> Ports {
        self.illegal_wires = true;
        self
    }

    pub fn declare(mut self, pos: BlockPos, role: Role) -> Ports {
        self.declared.insert(pos, role);
        self
    }

    pub fn declared(&self) -> impl Iterator<Item = (BlockPos, Role)> + '_ {
        self.declared.iter().map(|(&pos, &role)| (pos, role))
    }

    pub fn role(&self, pos: BlockPos, block: Block, kind: &NodeKind) -> Option<Role> {
        if let Some(&role) = self.declared.get(&pos) {
            return Some(role);
        }
        if let Block::RedstoneWire(wire) = block {
            let output = self.wire_dots && wire::is_dot(wire)
                || self.wire_crosses && wire::is_cross(wire)
                || self.illegal_wires && wire::is_illegal(wire);
            return output.then_some(Role::Output);
        }
        if self.conventional {
            return kind.conventional_role();
        }
        None
    }
}
