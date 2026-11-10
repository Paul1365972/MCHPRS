use super::{is_dust, Builder};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockDirection, BlockPos};
use mchprs_redstone::{state_for_placement, HORIZONTAL};

impl Builder<'_> {
    /// Only vanilla gives a lit lamp a second off tick when a block other than its input updates
    /// it (docs/Semantics.md, Lamps)
    pub(super) fn updates(
        &self,
        lamp: BlockPos,
        input: BlockPos,
        pos: BlockPos,
        block: Block,
    ) -> bool {
        let distance =
            |at: BlockPos| (at - lamp).x.abs() + (at - lamp).y.abs() + (at - lamp).z.abs();
        let updates = match diode_facing(block) {
            Some(facing) => {
                let front = pos.offset(facing.opposite().block_face());
                let ahead = self.blocks.block(front);
                let powers_lamp = ahead.is_conductor() || front == input && is_dust(ahead);
                // A repeater updates what is in front of it when it locks or unlocks
                let is_locked_from_side =
                    matches!(block, Block::Repeater(_)) && self.has_diode_into_side(pos, facing);
                let locks_repeater_before_lamp = match ahead {
                    Block::Repeater(locked) if is_across(locked.facing, facing) => {
                        distance(front.offset(locked.facing.opposite().block_face())) <= 1
                    }
                    _ => false,
                };
                distance(front) == 1 && !powers_lamp
                    || distance(front) <= 1 && is_locked_from_side
                    || locks_repeater_before_lamp
            }
            None => {
                let turns_off_or_changes = is_dust(block)
                    || block.button().is_some()
                    || block.pressure_plate().is_some()
                    || matches!(
                        block,
                        Block::NoteBlock { .. }
                            | Block::RedstoneTorch { .. }
                            | Block::RedstoneWallTorch { .. }
                    );
                distance(pos) <= 2 && turns_off_or_changes
            }
        };
        pos != input && updates
    }

    fn has_diode_into_side(&self, pos: BlockPos, facing: BlockDirection) -> bool {
        let faces_side = |side: BlockDirection| {
            let beside = self.blocks.block(pos.offset(side.block_face()));
            diode_facing(beside) == Some(side)
        };
        [facing.rotate(), facing.rotate_ccw()]
            .into_iter()
            .any(faces_side)
    }

    pub(super) fn powers(&self, input: BlockPos, lamp: BlockPos) -> bool {
        let placed = state_for_placement(&self.blocks, input, self.blocks.block(input));
        let Block::RedstoneWire(dust) = placed else {
            return true;
        };
        let points_at_lamp = |direction: BlockDirection| {
            input.offset(direction.block_face()) == lamp && !dust.side(direction).is_none()
        };
        HORIZONTAL.into_iter().any(points_at_lamp)
    }
}

fn diode_facing(block: Block) -> Option<BlockDirection> {
    match block {
        Block::Repeater(repeater) => Some(repeater.facing),
        Block::Comparator(comparator) => Some(comparator.facing),
        _ => None,
    }
}

fn is_across(facing: BlockDirection, other: BlockDirection) -> bool {
    facing != other && facing != other.opposite()
}
