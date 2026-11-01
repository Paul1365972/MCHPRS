#[cfg(feature = "nbt")]
mod nbt_codec;

use crate::signal_strength::SignalStrength;
use mchprs_proc_macros::protocol_id;
use serde::{Deserialize, Serialize};
use std::str::FromStr;

/// A single item in an inventory
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InventoryEntry {
    pub id: u32,
    pub slot: i8,
    pub count: i8,
    pub nbt: Option<Vec<u8>>,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
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
        Ok(match s {
            "barrel" => ContainerType::Barrel,
            "furnace" => ContainerType::Furnace,
            "hopper" => ContainerType::Hopper,
            _ => return Err(()),
        })
    }
}

impl std::fmt::Display for ContainerType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ContainerType::Furnace => "minecraft:furnace",
            ContainerType::Barrel => "minecraft:barrel",
            ContainerType::Hopper => "minecraft:hopper",
        })
    }
}

impl ContainerType {
    pub fn num_slots(self) -> u8 {
        match self {
            ContainerType::Furnace => 3,
            ContainerType::Barrel => 27,
            ContainerType::Hopper => 5,
        }
    }

    pub fn items_needed_for_signal_strength(self, signal_strength: SignalStrength) -> usize {
        let slots = self.num_slots() as usize;
        match signal_strength {
            SignalStrength::Zero => 0,
            SignalStrength::Fifteen => slots * 64,
            _ => ((32 * slots * signal_strength.get() as usize) as f32 / 7.0 - 1.0).ceil() as usize,
        }
    }

    pub fn window_type(self) -> i32 {
        // https://wiki.vg/Inventory
        match self {
            ContainerType::Furnace => protocol_id!("minecraft:menu", "minecraft:furnace"),
            ContainerType::Barrel => protocol_id!("minecraft:menu", "minecraft:generic_9x3"),
            ContainerType::Hopper => protocol_id!("minecraft:menu", "minecraft:hopper"),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BlockEntity {
    Comparator {
        #[serde(deserialize_with = "SignalStrength::deserialize_saturating")]
        output_strength: SignalStrength,
    },
    Container {
        #[serde(deserialize_with = "SignalStrength::deserialize_saturating")]
        comparator_override: SignalStrength,
        inventory: Vec<InventoryEntry>,
        ty: ContainerType,
    },
    Sign(Box<SignBlockEntity>),
}

impl BlockEntity {
    /// The protocol id for the block entity
    pub fn ty(&self) -> i32 {
        macro_rules! block_entity_id {
            ($name:literal) => {
                protocol_id!("minecraft:block_entity_type", $name)
            };
        }
        match self {
            BlockEntity::Comparator { .. } => block_entity_id!("minecraft:comparator"),
            BlockEntity::Container { ty, .. } => match ty {
                ContainerType::Furnace => block_entity_id!("minecraft:furnace"),
                ContainerType::Barrel => block_entity_id!("minecraft:barrel"),
                ContainerType::Hopper => block_entity_id!("minecraft:hopper"),
            },
            BlockEntity::Sign(_) => block_entity_id!("minecraft:sign"),
        }
    }
}
