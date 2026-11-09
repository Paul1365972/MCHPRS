use crate::ports::Role;
use mchprs_blocks::BlockPos;
use rustc_hash::FxHashMap;

/// What a simulation does with the component at a position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Component {
    pub port: Option<Role>,
    pub observed: bool,
}

#[derive(Debug, Clone, Default)]
pub struct Layout(FxHashMap<BlockPos, Component>);

impl Layout {
    pub fn at(&self, pos: BlockPos) -> Option<Component> {
        self.0.get(&pos).copied()
    }

    pub fn positions(&self) -> impl Iterator<Item = BlockPos> + '_ {
        self.0.keys().copied()
    }
}

impl FromIterator<(BlockPos, Component)> for Layout {
    fn from_iter<I: IntoIterator<Item = (BlockPos, Component)>>(components: I) -> Layout {
        Layout(components.into_iter().collect())
    }
}
