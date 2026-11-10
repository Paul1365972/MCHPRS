use super::{Builder, Head, Piece, Start, GROUNDS_OF_CLIMBING_DUST};
use mchprs_blocks::blocks::{Block, ComparatorMode};
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use mchprs_redstone::HORIZONTAL;

const MOST_LEVELS: u64 = 3;
const LEVEL_HEIGHT: i32 = 2;

const WEIGHTED_MODES: [ComparatorMode; 4] = [
    ComparatorMode::Subtract,
    ComparatorMode::Subtract,
    ComparatorMode::Subtract,
    ComparatorMode::Compare,
];

const LATE_INPUT_PERCENT: u64 = 50;
const CHANGING_STARTS: [Start; 2] = [Start::Input, Start::BranchOffDust];
const LATE_INPUT_PIECES: [Piece; 2] = [Piece::Repeater, Piece::Comparator];

#[derive(Clone, Copy, PartialEq)]
enum Entry {
    Corner,
    RearDust,
    SideDust,
}

const ENTRIES: [Entry; 3] = [Entry::Corner, Entry::RearDust, Entry::SideDust];

#[derive(Clone, Copy)]
struct Level {
    comparator: BlockPos,
    heading: BlockDirection,
    side: BlockDirection,
}

impl Level {
    fn rear_dust(self) -> BlockPos {
        self.comparator.offset(self.heading.opposite().block_face())
    }

    fn side_dust(self) -> BlockPos {
        self.comparator.offset(self.side.block_face())
    }

    fn corner(self) -> BlockPos {
        self.rear_dust().offset(self.side.block_face())
    }

    fn entered_at(self, entry: Entry) -> BlockPos {
        match entry {
            Entry::Corner => self.corner(),
            Entry::RearDust => self.rear_dust(),
            Entry::SideDust => self.side_dust(),
        }
    }

    fn above(self) -> Level {
        Level {
            comparator: self.comparator + BlockPos::new(0, LEVEL_HEIGHT, 0),
            ..self
        }
    }

    fn head(self) -> Head {
        Head {
            pos: self.comparator,
            heading: self.heading,
            piece: Piece::Comparator,
        }
    }
}

impl Builder<'_> {
    pub(super) fn add_comparator_column(&mut self, head: Head) -> Option<()> {
        let head = match head.piece {
            Piece::Block => self.add_piece(head, Piece::Torch)?,
            _ => head,
        };
        let entry = self.rng.pick(&ENTRIES);
        let emits_to_all_sides = matches!(head.piece, Piece::Source | Piece::Torch);
        let is_entry =
            matches!(head.piece, Piece::Dust) || emits_to_all_sides && entry == Entry::Corner;
        let entered_at = if is_entry { head.pos } else { head.ahead() };
        let lowest = self.level_entered_at(entered_at, entry, is_entry)?;
        if !is_entry {
            let is_diode = matches!(head.piece, Piece::Repeater | Piece::Comparator);
            if entry == Entry::Corner && is_diode && self.rng.chance(50) {
                self.place(entered_at, Block::Stone)?;
            } else {
                self.place_dust(entered_at)?;
            }
        }
        for dust in [lowest.rear_dust(), lowest.corner(), lowest.side_dust()] {
            if self.is_free(dust) {
                self.place_dust(dust)?;
            }
        }
        let mut levels = vec![self.place_level(lowest)?];
        for _ in 1..self.rng.within(1..=MOST_LEVELS) {
            let highest = levels[levels.len() - 1];
            match self.add_level_above(highest) {
                Some(level) => levels.push(level),
                None => break,
            }
        }
        for &level in &levels {
            if self.rng.chance(LATE_INPUT_PERCENT) {
                self.add_late_input(level);
            }
        }
        for level in levels {
            self.continue_path(level.head());
        }
        Some(())
    }

    fn level_entered_at(
        &mut self,
        entered_at: BlockPos,
        entry: Entry,
        is_placed: bool,
    ) -> Option<Level> {
        let mut levels = Vec::new();
        for heading in HORIZONTAL {
            for side in [heading.rotate(), heading.rotate_ccw()] {
                let at_origin = Level {
                    comparator: BlockPos::new(0, 0, 0),
                    heading,
                    side,
                };
                levels.push(Level {
                    comparator: entered_at - at_origin.entered_at(entry),
                    ..at_origin
                });
            }
        }
        self.rng.shuffle(&mut levels);
        let fits = |level: &Level| {
            let blocks = [
                level.comparator,
                level.rear_dust(),
                level.corner(),
                level.side_dust(),
            ];
            let mut placed = blocks.iter().filter(|&&pos| !self.is_free(pos));
            match is_placed {
                true => placed.eq([&entered_at]),
                false => placed.next().is_none(),
            }
        };
        levels.into_iter().find(fits)
    }

    fn place_level(&mut self, level: Level) -> Option<Level> {
        let mode = self.rng.pick(&WEIGHTED_MODES);
        self.place_comparator(level.comparator, level.heading.opposite(), mode)?;
        Some(level)
    }

    fn add_level_above(&mut self, level: Level) -> Option<Level> {
        self.climb(level.rear_dust(), level.side.opposite())?;
        self.climb(level.side_dust(), level.side)?;
        self.place_level(level.above())
    }

    fn climb(&mut self, dust: BlockPos, toward_step: BlockDirection) -> Option<()> {
        let above = dust.offset(BlockFace::Top);
        let step = above.offset(toward_step.block_face());
        let above_step = step.offset(BlockFace::Top);
        if !self.is_free(above_step) {
            return None;
        }
        self.place_dust(step)?;
        let ground = self.rng.pick(&GROUNDS_OF_CLIMBING_DUST);
        self.place(above, ground)?;
        self.place_dust(above.offset(BlockFace::Top))?;
        self.kept_free.push(above_step);
        Some(())
    }

    fn add_late_input(&mut self, level: Level) -> Option<()> {
        let start = self.rng.pick(&CHANGING_STARTS);
        let toward_path = level.heading.opposite();
        self.lead_path_to(start, level.rear_dust(), toward_path, &LATE_INPUT_PIECES)
    }
}
