use fastnbt::Value;
use mchprs_blocks::block_entities::ContainerType;
use mchprs_blocks::items::Item;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_network::NBTCompound;
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct ItemStack {
    pub item_type: Item,
    pub count: u8,
    pub nbt: Option<NBTCompound>,
}

impl ItemStack {
    /// Create container item with specified signal strength
    pub fn container_with_ss(container_ty: ContainerType, ss: SignalStrength) -> ItemStack {
        let item = match container_ty {
            ContainerType::Barrel => Item::Barrel {},
            ContainerType::Hopper => Item::Hopper {},
            ContainerType::Furnace => Item::Furnace {},
        };

        let items_needed = container_ty.items_needed_for_signal_strength(ss);

        let list = Value::List({
            let mut items = Vec::new();
            for (slot, items_added) in (0..items_needed).step_by(64).enumerate() {
                let count = (items_needed - items_added).min(64);
                items.push(Value::Compound(HashMap::from([
                    ("Count".to_owned(), Value::Byte(count as i8)),
                    (
                        "id".to_owned(),
                        Value::String("minecraft:redstone".to_owned()),
                    ),
                    ("Slot".to_owned(), Value::Byte(slot as i8)),
                ])));
            }
            items
        });

        let nbt = NBTCompound::from([(
            "BlockEntityTag".to_owned(),
            Value::Compound(HashMap::from([
                ("Items".to_owned(), list),
                ("Id".to_owned(), Value::String(container_ty.to_string())),
            ])),
        )]);

        ItemStack {
            item_type: item,
            count: 1,
            nbt: Some(nbt),
        }
    }
}
