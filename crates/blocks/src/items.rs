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
    #[serde(skip)]
    pub nbt: Option<nbt::Blob>,
    pub container_slots: Vec<Option<ItemStack>>,
}

impl ItemStack {
    pub fn new(item_type: Item, count: u8) -> ItemStack {
        ItemStack {
            item_type,
            count,
            nbt: None,
            container_slots: Vec::new(),
        }
    }

    pub fn is_same_item(&self, other: &ItemStack) -> bool {
        self.item_type == other.item_type
            && self.nbt == other.nbt
            && self.container_slots == other.container_slots
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
        } as usize;

        let mut container_slots = Vec::new();
        if items_needed > 0 {
            for (_slot, items_added) in (0..items_needed).step_by(64).enumerate() {
                let count = (items_needed - items_added).min(64);
                container_slots.push(Some(ItemStack {
                    item_type: Item::Redstone,
                    count: count as u8,
                    nbt: None,
                    container_slots: Vec::new(),
                }));
            }
        }

        ItemStack {
            item_type: item,
            count: 1,
            nbt: None,
            container_slots,
        }
    }
}
