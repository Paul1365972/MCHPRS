#[cfg(feature = "nbt")]
mod nbt_codec;

use crate::blocks::{Block, HopperFacing};
use crate::items::Item;
use crate::registries::{block_entity_type, menu};
use crate::{BlockDirection, BlockFacing};
use serde::{Deserialize, Serialize};
use std::str::FromStr;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InventoryEntry {
    pub id: u32,
    pub slot: i8,
    pub count: i8,
    pub nbt: Option<Vec<u8>>,
}

#[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SignBlockEntity {
    pub front_rows: [String; 4],
    pub back_rows: [String; 4],
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ContainerType {
    Furnace,
    Barrel,
    Hopper,
}

impl FromStr for ContainerType {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let named = |ty: &ContainerType| ty.name() == s;
        ContainerType::ALL.into_iter().find(named).ok_or(())
    }
}

impl std::fmt::Display for ContainerType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

impl ContainerType {
    pub const ALL: [ContainerType; 3] = [
        ContainerType::Barrel,
        ContainerType::Furnace,
        ContainerType::Hopper,
    ];

    pub fn name(self) -> &'static str {
        match self {
            ContainerType::Barrel => "barrel",
            ContainerType::Furnace => "furnace",
            ContainerType::Hopper => "hopper",
        }
    }

    pub fn items_needed_for_signal_strength(self, signal_strength: u8) -> usize {
        let slots = self.num_slots() as usize;
        match signal_strength {
            0 => 0,
            15 => slots * 64,
            _ => ((32 * slots * signal_strength as usize) as f32 / 7.0 - 1.0).ceil() as usize,
        }
    }

    pub fn num_slots(self) -> u8 {
        match self {
            ContainerType::Furnace => 3,
            ContainerType::Barrel => 27,
            ContainerType::Hopper => 5,
        }
    }

    pub fn block_entity_type(self) -> i32 {
        match self {
            ContainerType::Furnace => block_entity_type::FURNACE,
            ContainerType::Barrel => block_entity_type::BARREL,
            ContainerType::Hopper => block_entity_type::HOPPER,
        }
    }

    pub fn menu(self) -> i32 {
        // https://wiki.vg/Inventory
        match self {
            ContainerType::Furnace => menu::FURNACE,
            ContainerType::Barrel => menu::GENERIC_9X3,
            ContainerType::Hopper => menu::HOPPER,
        }
    }

    pub fn item(self) -> Item {
        match self {
            ContainerType::Furnace => Item::Furnace {},
            ContainerType::Barrel => Item::Barrel {},
            ContainerType::Hopper => Item::Hopper {},
        }
    }

    pub fn block(self, furnace_facing: BlockDirection, hopper_enabled: bool) -> Block {
        match self {
            ContainerType::Furnace => Block::Furnace {
                facing: furnace_facing,
                lit: false,
            },
            ContainerType::Barrel => Block::Barrel {
                facing: BlockFacing::Up,
                open: false,
            },
            ContainerType::Hopper => Block::Hopper {
                enabled: hopper_enabled,
                facing: HopperFacing::Down,
            },
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum BlockEntity {
    Comparator {
        output_strength: u8,
    },
    Container {
        comparator_override: u8,
        inventory: Vec<InventoryEntry>,
        ty: ContainerType,
    },
    Sign(Box<SignBlockEntity>),
}

impl BlockEntity {
    pub fn ty(&self) -> i32 {
        match self {
            BlockEntity::Comparator { .. } => block_entity_type::COMPARATOR,
            BlockEntity::Container { ty, .. } => ty.block_entity_type(),
            BlockEntity::Sign(_) => block_entity_type::SIGN,
        }
    }
}

impl Block {
    pub fn container_type(self) -> Option<ContainerType> {
        match self {
            Block::Furnace { .. } => Some(ContainerType::Furnace),
            Block::Barrel { .. } => Some(ContainerType::Barrel),
            Block::Hopper { .. } => Some(ContainerType::Hopper),
            _ => None,
        }
    }
}
