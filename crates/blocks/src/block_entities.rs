use crate::blocks::Block;
use crate::items::{ContainedItem, Item};
use crate::json_text;
use mchprs_proc_macros::protocol_id;
use mchprs_text::{ColorCode, TextComponent};
use mchprs_utils::{map, nbt_unwrap_val};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io::Cursor;
use std::str::FromStr;

const MAX_STACK_SIZE_COMPONENT: &str = "minecraft:max_stack_size";
/// Minecraft 1.20.5, which introduced item components
const ITEM_COMPONENTS_DATA_VERSION: i32 = 3837;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ContainerSlot {
    pub idx: u8,
    pub item: ContainedItem,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct SignBlockEntityText {
    pub messages: [TextComponent; 4],
    pub color: ColorCode,
    pub has_glowing_text: bool,
}

impl SignBlockEntityText {
    pub fn encode(&self) -> Vec<u8> {
        let messages: Vec<nbt::Value> = self
            .messages
            .iter()
            .map(|text| {
                // This is rather inefficient; The NBT library has no way of embedding a serde structure into a Value,
                // so instead we have to completely encode the structure and then deserialize it back into a Value.
                let mut data = Vec::new();
                nbt::to_writer(&mut data, text, None).unwrap();
                let mut cur = Cursor::new(&data[3..]);
                nbt::Value::from_reader(data[0], &mut cur)
            })
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        let color = serde_variant::to_variant_name(&self.color).unwrap();
        let nbt = nbt::Value::Compound(map! {
            "messages" => nbt::Value::List(messages),
            "color" => nbt::Value::String(color.to_owned()),
            "has_glowing_text" => nbt::Value::Byte(self.has_glowing_text as i8)
        });

        let mut data = Vec::new();
        nbt.to_writer(&mut data).unwrap();
        data
    }

    pub fn decode(data: &[u8]) -> nbt::Value {
        decode_compound(data)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SignBlockEntity {
    // Use the `encode` and `decode` methods on `SignBlockEntityText` to read and write these.
    pub front_text: Vec<u8>,
    pub back_text: Vec<u8>,
}

impl SignBlockEntity {
    /// Reads both sides of a sign written by this Minecraft version
    pub fn from_current_nbt(nbt: &HashMap<String, nbt::Value>) -> Option<SignBlockEntity> {
        let data_version = json_text::NBT_TEXT_DATA_VERSION;
        Some(SignBlockEntity {
            front_text: load_sign_side(Some(nbt.get("front_text")?), data_version),
            back_text: load_sign_side(Some(nbt.get("back_text")?), data_version),
        })
    }

    pub fn to_nbt(&self) -> nbt::Blob {
        use nbt::Value;
        nbt::Blob::with_content(map! {
            "is_waxed" => Value::Byte(0),
            "front_text" => SignBlockEntityText::decode(&self.front_text),
            "back_text" => SignBlockEntityText::decode(&self.back_text),
            "id" => Value::String("minecraft:sign".to_owned())
        })
    }
}

impl Default for SignBlockEntity {
    fn default() -> Self {
        let def = SignBlockEntityText::default().encode();
        Self {
            front_text: def.clone(),
            back_text: def,
        }
    }
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

    pub fn window_type(self) -> i32 {
        // https://wiki.vg/Inventory
        match self {
            ContainerType::Furnace => protocol_id!("minecraft:menu", "minecraft:furnace"),
            ContainerType::Barrel => protocol_id!("minecraft:menu", "minecraft:generic_9x3"),
            ContainerType::Hopper => protocol_id!("minecraft:menu", "minecraft:hopper"),
        }
    }

    pub fn for_block(block: &Block) -> Option<Self> {
        match block {
            Block::Furnace { .. } => Some(ContainerType::Furnace),
            Block::Barrel { .. } => Some(ContainerType::Barrel),
            Block::Hopper { .. } => Some(ContainerType::Hopper),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BlockEntity {
    Comparator {
        output_strength: u8,
    },
    Container {
        comparator_override: u8,
        inventory: Vec<ContainerSlot>,
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

    pub fn make_container(ty: ContainerType, inventory: Vec<ContainerSlot>) -> BlockEntity {
        let num_slots = ty.num_slots();
        let fullness_sum: f32 = inventory
            .iter()
            .map(|slot| slot.item.count as f32 / slot.item.max_stack_size as f32)
            .sum();
        BlockEntity::Container {
            comparator_override: (((fullness_sum / num_slots as f32) * 14.0).floor() as u8)
                .saturating_add((fullness_sum > 0.0) as u8),
            inventory,
            ty,
        }
    }

    pub fn container_from_items(ty: ContainerType, items: &[Option<ContainedItem>]) -> BlockEntity {
        let inventory = (0..ty.num_slots())
            .zip(items)
            .filter_map(|(idx, item)| {
                Some(ContainerSlot {
                    idx,
                    item: (*item)?,
                })
            })
            .collect();
        Self::make_container(ty, inventory)
    }

    fn load_container(
        slots_nbt: &[nbt::Value],
        ty: ContainerType,
        data_version: i32,
    ) -> Option<BlockEntity> {
        use nbt::Value;
        let mut inventory = Vec::new();
        for item in slots_nbt {
            let item_compound = nbt_unwrap_val!(item, Value::Compound);
            // Like vanilla's `ContainerHelper.loadAllItems`, drop what this version can't hold
            let count = nbt_get_int(item_compound, "Count").unwrap_or(1);
            let idx = nbt_get_int(item_compound, "Slot").unwrap_or(0);
            if count <= 0 || !(0..ty.num_slots() as i32).contains(&idx) {
                continue;
            }
            let Some(Value::String(item_name)) = nbt_case_insensitive(item_compound, "Id") else {
                continue;
            };
            let item_type = Item::from_name(item_name)
                .or_else(|| Item::from_name(&format!("minecraft:{item_name}")));
            let Some(item_type) = item_type else {
                continue;
            };
            let max_stack_size = match item_compound.get("components") {
                Some(Value::Compound(components))
                    if data_version >= ITEM_COMPONENTS_DATA_VERSION =>
                {
                    nbt_get_int(components, MAX_STACK_SIZE_COMPONENT)
                }
                _ => None,
            };
            inventory.push(ContainerSlot {
                idx: idx as u8,
                item: ContainedItem::new(item_type, count as u8, max_stack_size),
            });
        }
        Some(Self::make_container(ty, inventory))
    }

    pub fn from_nbt(
        id: &str,
        nbt: &HashMap<String, nbt::Value>,
        data_version: i32,
    ) -> Option<BlockEntity> {
        use nbt::Value;
        match id.trim_start_matches("minecraft:") {
            "comparator" => Some(BlockEntity::Comparator {
                output_strength: *nbt_unwrap_val!(&nbt["OutputSignal"], Value::Int) as u8,
            }),
            "furnace" => BlockEntity::load_container(
                nbt_unwrap_val!(&nbt["Items"], Value::List),
                ContainerType::Furnace,
                data_version,
            ),
            "barrel" => BlockEntity::load_container(
                nbt_unwrap_val!(&nbt["Items"], Value::List),
                ContainerType::Barrel,
                data_version,
            ),
            "hopper" => BlockEntity::load_container(
                nbt_unwrap_val!(&nbt["Items"], Value::List),
                ContainerType::Hopper,
                data_version,
            ),
            "sign" => {
                let sign = if nbt.contains_key("Text1") {
                    // This is the pre-1.20 encoding
                    let rows = ["Text1", "Text2", "Text3", "Text4"].map(|row| match nbt.get(row) {
                        Some(Value::String(row)) => row.as_str(),
                        _ => "",
                    });
                    let rows = rows.map(json_text::component_from_json);
                    let mut front_text = map! { "messages" => Value::List(rows.into()) };
                    if let Some(color) = nbt.get("Color") {
                        front_text.insert("color".to_owned(), color.clone());
                    }
                    if let Some(glowing) = nbt.get("GlowingText") {
                        front_text.insert("has_glowing_text".to_owned(), glowing.clone());
                    }
                    SignBlockEntity {
                        front_text: encode_compound(front_text),
                        back_text: SignBlockEntityText::default().encode(),
                    }
                } else {
                    SignBlockEntity {
                        front_text: load_sign_side(nbt.get("front_text"), data_version),
                        back_text: load_sign_side(nbt.get("back_text"), data_version),
                    }
                };
                Some(BlockEntity::Sign(Box::new(sign)))
            }
            _ => None,
        }
    }

    pub fn to_nbt(&self, sign_only: bool) -> Option<nbt::Blob> {
        if sign_only && !matches!(self, BlockEntity::Sign(_)) {
            return None;
        }

        use nbt::Value;
        match self {
            BlockEntity::Sign(sign) => Some(sign.to_nbt()),
            BlockEntity::Comparator { output_strength } => Some({
                nbt::Blob::with_content(map! {
                    "OutputSignal" => Value::Int(*output_strength as i32),
                    "id" => Value::String("minecraft:comparator".to_owned())
                })
            }),
            BlockEntity::Container { inventory, ty, .. } => Some({
                let mut items = Vec::new();
                for slot in inventory {
                    let mut nbt: HashMap<String, Value> = map! {
                        "count" => Value::Int(slot.item.count as i32),
                        "id" => Value::String(slot.item.item_type.get_name().to_owned()),
                        "Slot" => Value::Byte(slot.idx as i8)
                    };
                    if slot.item.overrides_max_stack_size() {
                        let max_stack_size = Value::Int(slot.item.max_stack_size as i32);
                        let components = map! { MAX_STACK_SIZE_COMPONENT => max_stack_size };
                        nbt.insert("components".to_owned(), Value::Compound(components));
                    }
                    items.push(Value::Compound(nbt));
                }
                nbt::Blob::with_content(map! {
                    "id" => Value::String(ty.to_string()),
                    "Items" => Value::List(items)
                })
            }),
        }
    }
}

fn nbt_case_insensitive<'a>(
    compound: &'a HashMap<String, nbt::Value>,
    name: &str,
) -> Option<&'a nbt::Value> {
    // It's not really case insensitive, this is really just checking both cases of the first character.
    compound
        .get(name)
        .or_else(|| compound.get(&name.to_lowercase()))
}

fn nbt_get_int(compound: &HashMap<String, nbt::Value>, name: &str) -> Option<i32> {
    let value = nbt_case_insensitive(compound, name)?;
    match value {
        nbt::Value::Byte(val) => Some(*val as i32),
        nbt::Value::Int(val) => Some(*val),
        _ => None,
    }
}

fn encode_compound(compound: HashMap<String, nbt::Value>) -> Vec<u8> {
    let mut data = Vec::new();
    nbt::Value::Compound(compound).to_writer(&mut data).unwrap();
    data
}

fn decode_compound(data: &[u8]) -> nbt::Value {
    nbt::Value::from_reader(0xa, &mut Cursor::new(data)).unwrap()
}

fn load_sign_side(side: Option<&nbt::Value>, data_version: i32) -> Vec<u8> {
    let Some(nbt::Value::Compound(side)) = side else {
        return SignBlockEntityText::default().encode();
    };
    let mut side = side.clone();
    if data_version < json_text::NBT_TEXT_DATA_VERSION {
        for key in ["messages", "filtered_messages"] {
            if let Some(nbt::Value::List(lines)) = side.remove(key) {
                let components = lines.into_iter().map(|line| match line {
                    nbt::Value::String(json) => json_text::component_from_json(&json),
                    component => component,
                });
                side.insert(key.to_owned(), nbt::Value::List(components.collect()));
            }
        }
    }
    encode_compound(side)
}
