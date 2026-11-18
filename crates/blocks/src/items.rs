use crate::block_entities::ContainerType;
pub use crate::generated::Item;
use serde::de::Error;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

impl Serialize for Item {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(self.get_name())
    }
}

impl<'de> Deserialize<'de> for Item {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let name = String::deserialize(deserializer)?;
        Item::from_name(&name).ok_or_else(|| D::Error::custom(format!("unknown item {name}")))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ItemStack {
    pub item_type: Item,
    pub count: u8,
    pub container_items: Vec<Option<ContainedItem>>,
    #[serde(with = "encoded_nbt")]
    pub block_entity_data: Option<nbt::Blob>,
}

impl ItemStack {
    pub fn new(item_type: Item, count: u8) -> ItemStack {
        ItemStack {
            item_type,
            count,
            container_items: Vec::new(),
            block_entity_data: None,
        }
    }

    pub fn is_same_item(&self, other: &ItemStack) -> bool {
        self.item_type == other.item_type
            && self.container_items == other.container_items
            && self.block_entity_data == other.block_entity_data
    }

    /// Create container item with specified signal strength
    pub fn container_with_ss(container_ty: ContainerType, ss: u8) -> ItemStack {
        let item = match container_ty {
            ContainerType::Barrel => Item::Barrel {},
            ContainerType::Hopper => Item::Hopper {},
            ContainerType::Furnace => Item::Furnace {},
        };
        let slots = container_ty.num_slots() as u32;

        let items_needed = match ss {
            0 => 0,
            15 => slots * 64,
            _ => ((32 * slots * ss as u32) as f32 / 7.0 - 1.0).ceil() as u32,
        };

        let container_items = (0..items_needed)
            .step_by(64)
            .map(|items_added| {
                Some(ContainedItem {
                    item_type: Item::Redstone,
                    count: (items_needed - items_added).min(64) as u8,
                    max_stack_size: Item::Redstone.max_stack_size(),
                })
            })
            .collect();

        ItemStack {
            container_items,
            ..ItemStack::new(item, 1)
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContainedItem {
    pub item_type: Item,
    pub count: u8,
    pub max_stack_size: u8,
}

impl ContainedItem {
    pub fn overrides_max_stack_size(&self) -> bool {
        self.max_stack_size != self.item_type.max_stack_size()
    }
}

mod encoded_nbt {
    use serde::de::Error;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(
        nbt: &Option<nbt::Blob>,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        let encoded = nbt.as_ref().map(|nbt| {
            let mut data = Vec::new();
            nbt.to_writer(&mut data).unwrap();
            data
        });
        encoded.serialize(serializer)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<Option<nbt::Blob>, D::Error> {
        let encoded = Option::<Vec<u8>>::deserialize(deserializer)?;
        encoded
            .map(|data| nbt::Blob::from_reader(&mut data.as_slice()).map_err(D::Error::custom))
            .transpose()
    }
}
