use fastnbt::Value;
use mchprs_blocks::block_entities::{BlockEntity, ContainerType, InventoryEntry};
use mchprs_blocks::items::Item;
use mchprs_network::NBTCompound;

#[derive(Clone, Debug)]
pub struct ItemStack {
    pub item_type: Item,
    pub count: u8,
    pub nbt: Option<NBTCompound>,
}

impl ItemStack {
    pub fn container(ty: ContainerType, inventory: Vec<InventoryEntry>) -> ItemStack {
        let item = match ty {
            ContainerType::Barrel => Item::Barrel {},
            ContainerType::Hopper => Item::Hopper {},
            ContainerType::Furnace => Item::Furnace {},
        };
        let block_entity = BlockEntity::Container { inventory, ty };
        let tag = block_entity
            .to_nbt(false)
            .expect("a container always encodes to NBT");
        let nbt = NBTCompound::from([("BlockEntityTag".to_owned(), Value::Compound(tag))]);

        ItemStack {
            item_type: item,
            count: 1,
            nbt: Some(nbt),
        }
    }
}
