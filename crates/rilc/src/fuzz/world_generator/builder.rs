mod comparator_column;
mod lamp_update;

use crate::fuzz::rng::Rng;
use crate::fuzz::scenario_generator::random_strength;
use crate::input::World;
use mchprs_blocks::block_entities::{BlockEntity, ContainerType};
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::{
    Block, Button, Comparator, ComparatorMode, Instrument, LeverFace, PressurePlate, RedstoneWire,
    Repeater, SlabType, TrapdoorHalf,
};
use mchprs_blocks::region::Region;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use mchprs_redpiler::circuit::{NOTES, REPEATER_DELAYS};
use mchprs_redstone::diode::is_diode;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_redstone::{can_survive, state_for_placement, update_region, HORIZONTAL};

const ARENA_WIDTH: u64 = 12;
const ARENA_HEIGHT: u64 = 8;
const SOURCE_MARGIN: u64 = 3;
const SOURCE_HEIGHT: i32 = 2;
/// Vanilla updates the neighbours of dust in an order that depends on where the dust is
const FARTHEST_ARENA_BLOCKS: u64 = 100;
const HIGHEST_ARENA_FLOOR: u64 = 200;

const MOST_PATHS: u64 = 4;
const MOST_SIDE_INPUTS: u64 = 4;
const MOST_PIECES_OF_PATH: u64 = 6;
const LONGEST_DUST_RUN: u64 = 3;
const MOST_DUST_LAID_TO_GOAL: u64 = 10;

const TOP_SLAB: Block = Block::SmoothStoneSlab {
    ty: SlabType::Top,
    waterlogged: false,
};
const GROUNDS: [Block; 3] = [Block::Stone, TOP_SLAB, Block::Glass];
const GROUNDS_OF_CLIMBING_DUST: [Block; 2] = [TOP_SLAB, Block::Glass];

#[derive(Clone, Copy)]
enum Start {
    Input,
    Torch,
    RedstoneBlock,
    ContainerAndComparator,
    BranchOffDust,
}

const STARTS: [Start; 5] = [
    Start::Input,
    Start::Torch,
    Start::RedstoneBlock,
    Start::ContainerAndComparator,
    Start::BranchOffDust,
];

#[derive(Clone, Copy)]
enum Piece {
    Source,
    Dust,
    Repeater,
    Comparator,
    Block,
    Torch,
}

fn pieces_after(piece: Piece) -> &'static [Piece] {
    match piece {
        Piece::Source | Piece::Torch => &[Piece::Dust, Piece::Repeater, Piece::Comparator],
        Piece::Dust | Piece::Repeater | Piece::Comparator => &[
            Piece::Dust,
            Piece::Repeater,
            Piece::Comparator,
            Piece::Block,
        ],
        Piece::Block => &[Piece::Torch, Piece::Repeater, Piece::Comparator],
    }
}

#[derive(Clone, Copy)]
enum End {
    Lamp,
    IronTrapdoor,
    NoteBlock,
    Open,
    LoopToOwnDust,
    ComparatorColumn,
}

const ENDS: [End; 6] = [
    End::Lamp,
    End::IronTrapdoor,
    End::NoteBlock,
    End::Open,
    End::LoopToOwnDust,
    End::ComparatorColumn,
];

#[derive(Clone, Copy)]
struct Head {
    pos: BlockPos,
    heading: BlockDirection,
    piece: Piece,
}

impl Head {
    fn ahead(self) -> BlockPos {
        self.pos.offset(self.heading.block_face())
    }
}

struct Lamp {
    pos: BlockPos,
    input: BlockPos,
}

struct Builder<'a> {
    rng: &'a mut Rng,
    blocks: BlockVolume,
    placed: Vec<BlockPos>,
    kept_free: Vec<BlockPos>,
    lamps: Vec<Lamp>,
}

impl Builder<'_> {
    fn new(rng: &mut Rng) -> Builder<'_> {
        let mut horizontal =
            || rng.below(2 * FARTHEST_ARENA_BLOCKS) as i32 - FARTHEST_ARENA_BLOCKS as i32;
        let (x, z) = (horizontal(), horizontal());
        let corner = BlockPos::new(x, rng.below(HIGHEST_ARENA_FLOOR) as i32, z);
        let (width, height) = (ARENA_WIDTH as i32, ARENA_HEIGHT as i32);
        let opposite_corner = corner + BlockPos::new(width - 1, height - 1, width - 1);
        Builder {
            rng,
            blocks: BlockVolume::empty(Region::new(corner, opposite_corner)),
            placed: Vec::new(),
            kept_free: Vec::new(),
            lamps: Vec::new(),
        }
    }

    fn is_free(&self, pos: BlockPos) -> bool {
        self.blocks.region().contains(pos)
            && self.blocks.block(pos) == Block::Air
            && !self.kept_free.contains(&pos)
    }

    fn place(&mut self, pos: BlockPos, block: Block) -> Option<()> {
        let mut lamps = self.lamps.iter();
        if !self.is_free(pos) || lamps.any(|lamp| self.updates(lamp.pos, lamp.input, pos, block)) {
            return None;
        }
        self.blocks.set_block(pos, block);
        let turns_input_away = |lamp: &Lamp| !self.powers(lamp.input, lamp.pos);
        if self.lamps.iter().any(turns_input_away) {
            self.blocks.set_block(pos, Block::Air);
            return None;
        }
        self.placed.push(pos);
        Some(())
    }

    fn place_on_ground(&mut self, pos: BlockPos, block: Block) -> Option<()> {
        if !self.is_free(pos) {
            return None;
        }
        let below = pos.offset(BlockFace::Bottom);
        if self.is_free(below) {
            let ground = self.rng.pick(&GROUNDS);
            self.place(below, ground)?;
        }
        if !can_survive(&self.blocks, pos, block) {
            return None;
        }
        self.place(pos, block)
    }

    fn random_placed(&mut self, is_wanted: fn(Block) -> bool) -> Option<BlockPos> {
        let mut wanted = self.placed.clone();
        wanted.retain(|&pos| is_wanted(self.blocks.block(pos)));
        (!wanted.is_empty()).then(|| self.rng.pick(&wanted))
    }

    fn add_path(&mut self, start: Start) -> Option<()> {
        let head = self.start_path(start)?;
        self.continue_path(head)
    }

    fn continue_path(&mut self, mut head: Head) -> Option<()> {
        for _ in 0..self.rng.within(1..=MOST_PIECES_OF_PATH) {
            let piece = self.rng.pick(pieces_after(head.piece));
            head = self.add_piece(head, piece)?;
        }
        let end = self.rng.pick(&ENDS);
        self.end_path(head, end)
    }

    fn start_path(&mut self, start: Start) -> Option<Head> {
        let heading = self.rng.pick(&HORIZONTAL);
        let (pos, piece) = match start {
            Start::BranchOffDust => (self.random_placed(is_dust)?, Piece::Dust),
            Start::ContainerAndComparator => return self.add_container(heading),
            Start::Input => {
                let pos = self.source_pos();
                let source = self.rng.pick(&[
                    Block::Lever {
                        face: LeverFace::Floor,
                        facing: heading,
                        powered: false,
                    },
                    Block::StoneButton(Button::new(LeverFace::Floor, heading, false)),
                    Block::OakButton(Button::new(LeverFace::Floor, heading, false)),
                    Block::StonePressurePlate(PressurePlate { powered: false }),
                ]);
                self.place_on_ground(pos, source)?;
                (pos, Piece::Source)
            }
            Start::RedstoneBlock => {
                let pos = self.source_pos();
                self.place(pos, Block::RedstoneBlock)?;
                (pos, Piece::Source)
            }
            Start::Torch => {
                let pos = self.source_pos();
                self.place_on_ground(pos, Block::RedstoneTorch { lit: true })?;
                (pos, Piece::Torch)
            }
        };
        Some(Head {
            pos,
            heading,
            piece,
        })
    }

    fn source_pos(&mut self) -> BlockPos {
        let mut from_wall = || {
            let from_wall = self
                .rng
                .within(SOURCE_MARGIN..=ARENA_WIDTH - 1 - SOURCE_MARGIN);
            from_wall as i32
        };
        let from_corner = BlockPos::new(from_wall(), SOURCE_HEIGHT, from_wall());
        self.blocks.region().min() + from_corner
    }

    fn add_container(&mut self, heading: BlockDirection) -> Option<Head> {
        let pos = self.source_pos();
        let ty = self.rng.pick(&ContainerType::ALL);
        self.place(pos, ty.block(heading, true))?;
        let inventory = ty.redstone_for(random_strength(self.rng));
        let contents = BlockEntity::Container { inventory, ty };
        self.blocks.set_block_entity(pos, Some(contents));
        let mut head = Head {
            pos,
            heading,
            piece: Piece::Source,
        };
        if self.rng.chance(50) {
            head = self.add_piece(head, Piece::Block)?;
        }
        self.add_piece(head, Piece::Comparator)
    }

    fn add_piece(&mut self, head: Head, piece: Piece) -> Option<Head> {
        let toward_input = head.heading.opposite();
        let ahead = head.ahead();
        match piece {
            Piece::Source => return None,
            Piece::Dust => return self.add_dust_run(head),
            Piece::Torch => {
                let pos = head.pos.offset(BlockFace::Top);
                self.place_on_ground(pos, Block::RedstoneTorch { lit: true })?;
                return Some(Head { pos, piece, ..head });
            }
            Piece::Block => self.place(ahead, Block::Stone)?,
            Piece::Repeater => {
                let delay = self.rng.within(REPEATER_DELAYS);
                let repeater = Repeater::new(delay, toward_input, false, false);
                self.place_on_ground(ahead, Block::Repeater(repeater))?
            }
            Piece::Comparator => {
                let mode = self
                    .rng
                    .pick(&[ComparatorMode::Compare, ComparatorMode::Subtract]);
                self.place_comparator(ahead, toward_input, mode)?
            }
        }
        Some(Head {
            pos: ahead,
            piece,
            ..head
        })
    }

    fn place_comparator(
        &mut self,
        pos: BlockPos,
        toward_input: BlockDirection,
        mode: ComparatorMode,
    ) -> Option<()> {
        let comparator = Comparator::new(toward_input, mode, false);
        self.place_on_ground(pos, Block::Comparator(comparator))
    }

    fn place_dust(&mut self, pos: BlockPos) -> Option<()> {
        self.place_on_ground(pos, Block::RedstoneWire(RedstoneWire::default()))
    }

    fn add_dust_run(&mut self, head: Head) -> Option<Head> {
        let mut head = self.add_dust(head, head.heading, 0)?;
        for _ in 1..self.rng.within(1..=LONGEST_DUST_RUN) {
            let heading = head.heading;
            let heading = self
                .rng
                .pick(&[heading, heading.rotate(), heading.rotate_ccw()]);
            let rise = self.rng.pick(&[0, 1, -1]);
            head = self.add_dust(head, heading, rise)?;
        }
        Some(head)
    }

    fn add_dust(&mut self, head: Head, heading: BlockDirection, rise: i32) -> Option<Head> {
        let pos = head.pos.offset(heading.block_face()) + BlockPos::new(0, rise, 0);
        self.place_dust(pos)?;
        Some(Head {
            pos,
            heading,
            piece: Piece::Dust,
        })
    }

    fn end_path(&mut self, head: Head, end: End) -> Option<()> {
        let ahead = head.ahead();
        match end {
            End::Open => Some(()),
            End::Lamp => {
                let updates =
                    |&at: &BlockPos| self.updates(ahead, head.pos, at, self.blocks.block(at));
                if self.placed.iter().any(updates) || !self.powers(head.pos, ahead) {
                    return None;
                }
                self.place(ahead, Block::RedstoneLamp { lit: false })?;
                self.lamps.push(Lamp {
                    pos: ahead,
                    input: head.pos,
                });
                Some(())
            }
            End::IronTrapdoor => {
                let trapdoor = Block::IronTrapdoor {
                    facing: head.heading,
                    half: TrapdoorHalf::Bottom,
                    open: false,
                    powered: false,
                    waterlogged: false,
                };
                self.place(ahead, trapdoor)
            }
            End::NoteBlock => {
                let above = ahead.offset(BlockFace::Top);
                if self.blocks.block(above) != Block::Air {
                    return None;
                }
                let note_block = Block::NoteBlock {
                    instrument: Instrument::Harp,
                    note: self.rng.within(NOTES),
                    powered: false,
                };
                self.place(ahead, note_block)?;
                self.kept_free.push(above);
                Some(())
            }
            End::LoopToOwnDust => {
                let dust = self.random_placed(is_dust)?;
                let beside_dust = dust.offset(self.rng.pick(&HORIZONTAL).block_face());
                self.lay_dust_to(head, beside_dust)?;
                Some(())
            }
            End::ComparatorColumn => self.add_comparator_column(head),
        }
    }

    fn lay_dust_to(&mut self, head: Head, goal: BlockPos) -> Option<Head> {
        if !self.is_free(goal) {
            return None;
        }
        let mut head = self.add_dust(head, head.heading, 0)?;
        for _ in 1..MOST_DUST_LAID_TO_GOAL {
            if head.pos == goal {
                break;
            }
            head = self.add_dust_toward(head, goal)?;
        }
        (head.pos == goal).then_some(head)
    }

    fn add_dust_toward(&mut self, head: Head, goal: BlockPos) -> Option<Head> {
        let rise = (goal.y - head.pos.y).signum();
        let blocks_to_goal = |heading: &BlockDirection| {
            let beside = head.pos.offset(heading.block_face());
            (goal.x - beside.x).abs() + (goal.z - beside.z).abs()
        };
        let mut headings = HORIZONTAL;
        headings.sort_by_key(blocks_to_goal);
        headings
            .into_iter()
            .find_map(|heading| self.add_dust(head, heading, rise))
    }

    fn add_side_input(&mut self) -> Option<()> {
        let diode_pos = self.random_placed(is_diode)?;
        let (facing, pieces_beside) = match self.blocks.block(diode_pos) {
            Block::Repeater(repeater) => {
                (repeater.facing, &[Piece::Repeater, Piece::Comparator][..])
            }
            Block::Comparator(comparator) => (
                comparator.facing,
                &[Piece::Repeater, Piece::Comparator, Piece::Dust][..],
            ),
            _ => return None,
        };
        let side = self.rng.pick(&[facing.rotate(), facing.rotate_ccw()]);
        let start = self.rng.pick(&STARTS);
        self.lead_path_to(start, diode_pos, side, pieces_beside)
    }

    fn lead_path_to(
        &mut self,
        start: Start,
        target: BlockPos,
        toward_path: BlockDirection,
        last_pieces: &[Piece],
    ) -> Option<()> {
        let beside = target.offset(toward_path.block_face());
        let head = self.start_path(start)?;
        match self.rng.pick(last_pieces) {
            Piece::Dust => self.lay_dust_to(head, beside).map(drop),
            piece => {
                let head = self.lay_dust_to(head, beside.offset(toward_path.block_face()))?;
                let toward_target = Head {
                    heading: toward_path.opposite(),
                    ..head
                };
                self.add_piece(toward_target, piece).map(drop)
            }
        }
    }

    fn placed_blocks(mut self) -> BlockVolume {
        for &pos in &self.placed {
            let placed = state_for_placement(&self.blocks, pos, self.blocks.block(pos));
            self.blocks.set_block(pos, placed);
        }
        self.blocks
    }
}

pub fn random_world(rng: &mut Rng) -> World {
    let mut builder = Builder::new(rng);
    builder.add_path(Start::Input);
    for _ in 1..builder.rng.within(1..=MOST_PATHS) {
        let start = builder.rng.pick(&STARTS);
        builder.add_path(start);
    }
    for _ in 0..builder.rng.within(0..=MOST_SIDE_INPUTS) {
        builder.add_side_input();
    }
    let blocks = builder.placed_blocks();
    let region = blocks.region();
    let mut world = Interpreter::new(blocks, Vec::new());
    update_region(&mut world, region);
    world
}

fn is_dust(block: Block) -> bool {
    matches!(block, Block::RedstoneWire(_))
}
