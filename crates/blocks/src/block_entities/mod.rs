#[cfg(feature = "nbt")]
mod nbt_codec;

use crate::items::Item;
use crate::registries::{block_entity_type, menu};
use crate::signal_strength::{SignalStrength, UncappedSignalStrength};
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::str::FromStr;

const MAX_ITEM_COUNT: u32 = i8::MAX as u32;

/// A single item in an inventory
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
    pub const MAX_SIGNAL_STRENGTH: UncappedSignalStrength =
        UncappedSignalStrength::new(14 * MAX_ITEM_COUNT as u16 + 1);

    pub fn num_slots(self) -> u8 {
        match self {
            ContainerType::Furnace => 3,
            ContainerType::Barrel => 27,
            ContainerType::Hopper => 5,
        }
    }

    pub fn window_type(self) -> i32 {
        // https://wiki.vg/Inventory
        match self {
            ContainerType::Furnace => menu::FURNACE,
            ContainerType::Barrel => menu::GENERIC_9X3,
            ContainerType::Hopper => menu::HOPPER,
        }
    }

    /// Vanilla's `AbstractContainerMenu.getRedstoneSignalFromContainer`, in its float arithmetic
    pub fn signal_strength(self, inventory: &[InventoryEntry]) -> UncappedSignalStrength {
        let fullness = inventory
            .iter()
            .map(|entry| entry.count as f32 / Item::from_id(entry.id).max_stack_size() as f32)
            .sum::<f32>()
            / f32::from(self.num_slots());
        UncappedSignalStrength::new((fullness * 14.0).floor() as u16 + u16::from(fullness > 0.0))
    }

    pub fn inventory_for(self, strength: UncappedSignalStrength) -> Option<Vec<InventoryEntry>> {
        match SignalStrength::try_from(strength) {
            Ok(strength) => Some(self.redstone_for(strength)),
            Err(_) => self.overstacked_for(strength),
        }
    }

    pub fn redstone_for(self, strength: SignalStrength) -> Vec<InventoryEntry> {
        let slots = self.num_slots() as usize;
        let redstone = match strength {
            SignalStrength::Zero => 0,
            SignalStrength::Fifteen => slots * 64,
            _ => ((32 * slots * strength.get() as usize) as f32 / 7.0 - 1.0).ceil() as usize,
        };
        stacks(&[(Item::Redstone, redstone as u32)], 64)
    }

    fn overstacked_for(self, strength: UncappedSignalStrength) -> Option<Vec<InventoryEntry>> {
        let slots = self.num_slots() as usize;
        let mut redstone_worth = (64 * slots as u32 * (u32::from(strength.get()) - 1)).div_ceil(14);
        loop {
            let (totems, redstone) = match redstone_worth % 64 {
                0 => (redstone_worth / 64, 0),
                rest => (redstone_worth / 64 - 1, 64 + rest),
            };
            let inventory = stacks(
                &[(Item::TotemOfUndying, totems), (Item::Redstone, redstone)],
                MAX_ITEM_COUNT,
            );
            match self.signal_strength(&inventory).cmp(&strength) {
                Ordering::Equal if inventory.len() <= slots => return Some(inventory),
                Ordering::Greater => return None,
                _ => redstone_worth += 1,
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum BlockEntity {
    Comparator {
        output_strength: UncappedSignalStrength,
    },
    Container {
        inventory: Vec<InventoryEntry>,
        ty: ContainerType,
    },
    Sign(Box<SignBlockEntity>),
}

impl BlockEntity {
    /// The protocol id for the block entity
    pub fn ty(&self) -> i32 {
        match self {
            BlockEntity::Comparator { .. } => block_entity_type::COMPARATOR,
            BlockEntity::Container { ty, .. } => match ty {
                ContainerType::Furnace => block_entity_type::FURNACE,
                ContainerType::Barrel => block_entity_type::BARREL,
                ContainerType::Hopper => block_entity_type::HOPPER,
            },
            BlockEntity::Sign(_) => block_entity_type::SIGN,
        }
    }
}

fn stacks(items: &[(Item, u32)], per_slot: u32) -> Vec<InventoryEntry> {
    let mut inventory = Vec::new();
    for &(item, count) in items {
        let mut left = count;
        while left > 0 {
            let count = left.min(per_slot);
            inventory.push(InventoryEntry {
                id: item.get_id(),
                slot: inventory.len() as i8,
                count: count as i8,
                nbt: None,
            });
            left -= count;
        }
    }
    inventory
}
