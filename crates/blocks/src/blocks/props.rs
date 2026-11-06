use crate::registries::sound_event;
use serde::{Deserialize, Serialize};

use super::{Block, BlockDirection, BlockTransform, FlipDirection};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Repeater {
    pub delay: u8,
    pub facing: BlockDirection,
    pub locked: bool,
    pub powered: bool,
}

impl Default for Repeater {
    fn default() -> Self {
        Repeater {
            delay: 1,
            facing: Default::default(),
            locked: false,
            powered: false,
        }
    }
}

impl Repeater {
    pub fn new(delay: u8, facing: BlockDirection, locked: bool, powered: bool) -> Repeater {
        Repeater {
            delay,
            facing,
            locked,
            powered,
        }
    }
}

#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ComparatorMode {
    #[default]
    Compare,
    Subtract,
}

impl ComparatorMode {
    pub fn toggle(self) -> ComparatorMode {
        match self {
            ComparatorMode::Subtract => ComparatorMode::Compare,
            ComparatorMode::Compare => ComparatorMode::Subtract,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct Comparator {
    pub facing: BlockDirection,
    pub mode: ComparatorMode,
    pub powered: bool,
}

impl Comparator {
    pub fn new(facing: BlockDirection, mode: ComparatorMode, powered: bool) -> Comparator {
        Comparator {
            facing,
            mode,
            powered,
        }
    }
}

#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub enum LeverFace {
    Floor,
    #[default]
    Wall,
    Ceiling,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct StandingSign {
    pub rotation: u8,
    pub waterlogged: bool,
}

impl BlockTransform for StandingSign {
    fn rotate90(&mut self) {
        self.rotation = (self.rotation + 4) % 16;
    }

    fn flip(&mut self, dir: FlipDirection) {
        let mirror = match dir {
            FlipDirection::FlipX => 16,
            FlipDirection::FlipZ => 24,
        };
        self.rotation = (mirror - self.rotation) % 16;
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct Button {
    pub face: LeverFace,
    pub facing: BlockDirection,
    pub powered: bool,
}

impl Button {
    pub fn new(face: LeverFace, facing: BlockDirection, powered: bool) -> Button {
        Button {
            face,
            facing,
            powered,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum RedstoneWireSide {
    Up,
    Side,
    #[default]
    None,
}

impl RedstoneWireSide {
    pub fn is_none(self) -> bool {
        matches!(self, RedstoneWireSide::None)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct RedstoneWire {
    pub north: RedstoneWireSide,
    pub south: RedstoneWireSide,
    pub east: RedstoneWireSide,
    pub west: RedstoneWireSide,
    pub power: u8,
}

impl RedstoneWire {
    pub fn new(
        north: RedstoneWireSide,
        south: RedstoneWireSide,
        east: RedstoneWireSide,
        west: RedstoneWireSide,
        power: u8,
    ) -> RedstoneWire {
        RedstoneWire {
            north,
            south,
            east,
            west,
            power,
        }
    }

    pub fn side(self, direction: BlockDirection) -> RedstoneWireSide {
        match direction {
            BlockDirection::North => self.north,
            BlockDirection::South => self.south,
            BlockDirection::East => self.east,
            BlockDirection::West => self.west,
        }
    }

    pub fn with_side(mut self, direction: BlockDirection, side: RedstoneWireSide) -> RedstoneWire {
        match direction {
            BlockDirection::North => self.north = side,
            BlockDirection::South => self.south = side,
            BlockDirection::East => self.east = side,
            BlockDirection::West => self.west = side,
        }
        self
    }
}

impl BlockTransform for RedstoneWire {
    fn rotate90(&mut self) {
        *self = RedstoneWire {
            north: self.west,
            east: self.north,
            south: self.east,
            west: self.south,
            ..*self
        }
    }

    fn flip(&mut self, dir: FlipDirection) {
        *self = match dir {
            FlipDirection::FlipX => RedstoneWire {
                east: self.west,
                west: self.east,
                ..*self
            },
            FlipDirection::FlipZ => RedstoneWire {
                north: self.south,
                south: self.north,
                ..*self
            },
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum TrapdoorHalf {
    #[default]
    Top,
    Bottom,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Instrument {
    Harp,
    Basedrum,
    Snare,
    Hat,
    Bass,
    Flute,
    Bell,
    Guitar,
    Chime,
    Xylophone,
    IronXylophone,
    CowBell,
    Didgeridoo,
    Bit,
    Banjo,
    Pling,
    Zombie,
    Skeleton,
    Creeper,
    Dragon,
    WitherSkeleton,
    Piglin,
    CustomHead,
}

impl Instrument {
    pub fn from_block_below(block: Block) -> Instrument {
        match block {
            // All stone materials
            _ if block.is_stone() => Instrument::Basedrum,
            // All sand/aggregate materials: ConcretePowder
            Block::Sand => Instrument::Snare,
            // All glass materials: GlassPane
            _ if block.is_glass() => Instrument::Hat,
            // All wood materials: Log, Plank
            _ if block.is_wood() => Instrument::Bass,
            Block::Clay => Instrument::Flute,
            Block::GoldBlock => Instrument::Bell,
            _ if block.is_wool() => Instrument::Guitar,
            Block::PackedIce => Instrument::Chime,
            Block::BoneBlock { .. } => Instrument::Xylophone,
            Block::IronBlock => Instrument::IronXylophone,
            Block::SoulSand => Instrument::CowBell,
            Block::Pumpkin => Instrument::Didgeridoo,
            Block::EmeraldBlock => Instrument::Bit,
            Block::HayBlock { .. } => Instrument::Banjo,
            Block::Glowstone => Instrument::Pling,
            _ => Instrument::Harp,
        }
    }

    pub fn to_sound_id(&self) -> i32 {
        match self {
            Instrument::Harp => sound_event::BLOCK_NOTE_BLOCK_HARP,
            Instrument::Basedrum => sound_event::BLOCK_NOTE_BLOCK_BASEDRUM,
            Instrument::Snare => sound_event::BLOCK_NOTE_BLOCK_SNARE,
            Instrument::Hat => sound_event::BLOCK_NOTE_BLOCK_HAT,
            Instrument::Bass => sound_event::BLOCK_NOTE_BLOCK_BASS,
            Instrument::Flute => sound_event::BLOCK_NOTE_BLOCK_FLUTE,
            Instrument::Bell => sound_event::BLOCK_NOTE_BLOCK_BELL,
            Instrument::Guitar => sound_event::BLOCK_NOTE_BLOCK_GUITAR,
            Instrument::Chime => sound_event::BLOCK_NOTE_BLOCK_CHIME,
            Instrument::Xylophone => sound_event::BLOCK_NOTE_BLOCK_XYLOPHONE,
            Instrument::IronXylophone => sound_event::BLOCK_NOTE_BLOCK_IRON_XYLOPHONE,
            Instrument::CowBell => sound_event::BLOCK_NOTE_BLOCK_COW_BELL,
            Instrument::Didgeridoo => sound_event::BLOCK_NOTE_BLOCK_DIDGERIDOO,
            Instrument::Bit => sound_event::BLOCK_NOTE_BLOCK_BIT,
            Instrument::Banjo => sound_event::BLOCK_NOTE_BLOCK_BANJO,
            Instrument::Pling => sound_event::BLOCK_NOTE_BLOCK_PLING,
            Instrument::Zombie => sound_event::BLOCK_NOTE_BLOCK_IMITATE_ZOMBIE,
            Instrument::Skeleton => sound_event::BLOCK_NOTE_BLOCK_IMITATE_SKELETON,
            Instrument::Creeper => sound_event::BLOCK_NOTE_BLOCK_IMITATE_CREEPER,
            Instrument::Dragon => sound_event::BLOCK_NOTE_BLOCK_IMITATE_ENDER_DRAGON,
            Instrument::WitherSkeleton => sound_event::BLOCK_NOTE_BLOCK_IMITATE_WITHER_SKELETON,
            Instrument::Piglin => sound_event::BLOCK_NOTE_BLOCK_IMITATE_PIGLIN,
            _ => 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum HopperFacing {
    #[default]
    Down,
    North,
    South,
    West,
    East,
}

impl BlockTransform for HopperFacing {
    fn flip(&mut self, dir: FlipDirection) {
        match dir {
            FlipDirection::FlipX => match self {
                Self::East => *self = Self::West,
                Self::West => *self = Self::East,
                _ => {}
            },
            FlipDirection::FlipZ => match self {
                Self::North => *self = Self::South,
                Self::South => *self = Self::North,
                _ => {}
            },
        }
    }

    fn rotate90(&mut self) {
        *self = match *self {
            Self::North => Self::East,
            Self::East => Self::South,
            Self::South => Self::West,
            Self::West => Self::North,
            x => x,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SlabType {
    Top,
    #[default]
    Bottom,
    Double,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BlockAxis {
    X,
    #[default]
    Y,
    Z,
}

impl BlockTransform for BlockAxis {
    fn rotate90(&mut self) {
        *self = match *self {
            Self::X => Self::Z,
            Self::Z => Self::X,
            x => x,
        }
    }
    fn flip(&mut self, _dir: FlipDirection) {}
}
