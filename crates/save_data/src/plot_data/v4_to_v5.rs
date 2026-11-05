//! Migration from version 4 to version 5, which keeps comparator outputs above 15 and reads a
//! container's signal strength from its items alone

use super::{ChunkData, ChunkSectionData, PlotData, SavedTick, Tps, WorldSendRate};
use mchprs_blocks::block_entities::{BlockEntity, ContainerType, InventoryEntry, SignBlockEntity};
use mchprs_blocks::signal_strength::UncappedSignalStrength;
use mchprs_blocks::BlockPos;
use rustc_hash::FxHashMap;
use serde::Deserialize;
use std::borrow::Cow;
use std::path::Path;
use tracing::warn;

#[derive(Debug, Deserialize)]
enum BlockEntityV4 {
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

impl BlockEntityV4 {
    fn upgrade(self, refilled_containers: &mut usize) -> BlockEntity {
        match self {
            BlockEntityV4::Comparator { output_strength } => BlockEntity::Comparator {
                output_strength: UncappedSignalStrength::new(output_strength.into()),
            },
            // Stale item ids: plots store the item ids of the Minecraft version that saved them, so
            // they can name other items now. The saved override is what builds were made against,
            // so containers whose items no longer give it are refilled.
            BlockEntityV4::Container {
                comparator_override,
                inventory,
                ty,
            } => {
                let strength = UncappedSignalStrength::new(comparator_override.into()).capped();
                if ty.signal_strength(&inventory) == strength.into() {
                    BlockEntity::Container { inventory, ty }
                } else {
                    *refilled_containers += 1;
                    BlockEntity::Container {
                        inventory: ty.redstone_for(strength),
                        ty,
                    }
                }
            }
            BlockEntityV4::Sign(sign) => BlockEntity::Sign(sign),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct ChunkDataV4 {
    pub sections: Vec<Option<ChunkSectionData<'static>>>,
    block_entities: FxHashMap<BlockPos, BlockEntityV4>,
}

impl ChunkDataV4 {
    fn upgrade(self, refilled_containers: &mut usize) -> ChunkData<'static> {
        ChunkData {
            sections: self.sections,
            block_entities: Cow::Owned(
                self.block_entities
                    .into_iter()
                    .map(|(pos, block_entity)| (pos, block_entity.upgrade(refilled_containers)))
                    .collect(),
            ),
        }
    }
}

#[derive(Deserialize)]
pub struct PlotDataV4 {
    pub tps: Tps,
    pub world_send_rate: WorldSendRate,
    pub chunk_data: Vec<ChunkDataV4>,
    pub pending_ticks: Vec<SavedTick>,
}

impl PlotDataV4 {
    pub fn upgrade(self, path: &Path) -> PlotData<'static> {
        let mut refilled_containers = 0;
        let chunk_data = self
            .chunk_data
            .into_iter()
            .map(|chunk| chunk.upgrade(&mut refilled_containers))
            .collect();
        if refilled_containers > 0 {
            warn!(
                "Refilled {refilled_containers} containers in {} with redstone, as their stale \
                 item ids no longer gave their signal strength",
                path.display()
            );
        }
        PlotData {
            tps: self.tps,
            world_send_rate: self.world_send_rate,
            chunk_data,
            pending_ticks: self.pending_ticks,
        }
    }
}
