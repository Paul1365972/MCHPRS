use super::{BlockEntity, ContainerType, InventoryEntry, SignBlockEntity};
use crate::items::Item;
use crate::signal_strength::SignalStrength;
use std::collections::HashMap;

const EMPTY_SIGN_ROW: &str = r#""""#;

macro_rules! nbt_unwrap_val {
    ($value:expr, $variant:path) => {
        match $value {
            $variant(value) => value,
            _ => return None,
        }
    };
}

impl BlockEntity {
    fn load_container(slots_nbt: &[nbt::Value], ty: ContainerType) -> Option<BlockEntity> {
        use nbt::Value;
        let num_slots = ty.num_slots();
        let mut fullness_sum: f32 = 0.0;
        let mut inventory = Vec::new();
        let mut occupied_slots = 0u32;
        for item in slots_nbt {
            let item_compound = nbt_unwrap_val!(item, Value::Compound);
            let count = i8::try_from(nbt_get_int(item_compound, "Count")?).ok()?;
            let slot = i8::try_from(nbt_get_int(item_compound, "Slot")?).ok()?;
            let namespaced_name =
                nbt_unwrap_val!(nbt_case_insensitive(item_compound, "Id")?, Value::String);
            let item_type = Item::from_name(namespaced_name);
            if count < 0 || slot < 0 || slot as u8 >= num_slots || occupied_slots & (1 << slot) != 0
            {
                return None;
            }
            occupied_slots |= 1 << slot;

            let tag = match item_compound.get("tag") {
                Some(nbt::Value::Compound(map)) => {
                    let mut blob = nbt::Blob::new();
                    for (k, v) in map {
                        blob.insert(k, v.clone()).ok()?;
                    }

                    let mut data = Vec::new();
                    blob.to_writer(&mut data).ok()?;
                    Some(data)
                }
                _ => None,
            };
            inventory.push(InventoryEntry {
                slot,
                count,
                id: item_type.unwrap_or(Item::Redstone).get_id(),
                nbt: tag,
            });

            fullness_sum += count as f32 / item_type.map_or(64, Item::max_stack_size) as f32;
        }
        Some(BlockEntity::Container {
            comparator_override: SignalStrength::saturating_from(
                ((if fullness_sum > 0.0 { 1.0 } else { 0.0 })
                    + (fullness_sum / num_slots as f32) * 14.0)
                    .floor() as i32,
            ),
            inventory,
            ty,
        })
    }

    pub fn is_supported(id: &str) -> bool {
        matches!(
            id.trim_start_matches("minecraft:"),
            "comparator" | "furnace" | "barrel" | "hopper" | "sign"
        )
    }

    pub fn from_nbt(id: &str, nbt: &HashMap<String, nbt::Value>) -> Option<BlockEntity> {
        use nbt::Value;
        match id.trim_start_matches("minecraft:") {
            "comparator" => Some(BlockEntity::Comparator {
                // Like vanilla, a missing signal reads as 0
                output_strength: SignalStrength::saturating_from(
                    nbt_get_int(nbt, "OutputSignal").unwrap_or(0),
                ),
            }),
            "furnace" => BlockEntity::load_container(
                nbt_unwrap_val!(nbt.get("Items")?, Value::List),
                ContainerType::Furnace,
            ),
            "barrel" => BlockEntity::load_container(
                nbt_unwrap_val!(nbt.get("Items")?, Value::List),
                ContainerType::Barrel,
            ),
            "hopper" => BlockEntity::load_container(
                nbt_unwrap_val!(nbt.get("Items")?, Value::List),
                ContainerType::Hopper,
            ),
            "sign" => {
                let sign = if nbt.contains_key("Text1") {
                    // This is the pre-1.20 encoding
                    SignBlockEntity {
                        front_rows: [
                            // This cloning is really dumb
                            nbt_unwrap_val!(nbt.get("Text1")?.clone(), Value::String),
                            nbt_unwrap_val!(nbt.get("Text2")?.clone(), Value::String),
                            nbt_unwrap_val!(nbt.get("Text3")?.clone(), Value::String),
                            nbt_unwrap_val!(nbt.get("Text4")?.clone(), Value::String),
                        ],
                        back_rows: Default::default(),
                    }
                } else {
                    let get_side = |side| {
                        let messages =
                            nbt_unwrap_val!(nbt.get(side)?, Value::Compound).get("messages")?;
                        let mut messages = nbt_unwrap_val!(messages, Value::List).iter().cloned();
                        Some([
                            nbt_unwrap_val!(messages.next()?, Value::String),
                            nbt_unwrap_val!(messages.next()?, Value::String),
                            nbt_unwrap_val!(messages.next()?, Value::String),
                            nbt_unwrap_val!(messages.next()?, Value::String),
                        ])
                    };
                    SignBlockEntity {
                        front_rows: get_side("front_text")?,
                        back_rows: get_side("back_text")?,
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
            BlockEntity::Sign(sign) => Some({
                // Vanilla's SignText codec rejects an empty string, rows must be text components
                let message = |row: &String| {
                    Value::String(if row.is_empty() {
                        EMPTY_SIGN_ROW.to_owned()
                    } else {
                        row.clone()
                    })
                };
                let front = sign.front_rows.iter().map(message);
                let back = sign.back_rows.iter().map(message);
                nbt::Blob::with_content(HashMap::from([
                    ("is_waxed".to_owned(), Value::Byte(0)),
                    (
                        "front_text".to_owned(),
                        Value::Compound(HashMap::from([
                            ("has_glowing_text".to_owned(), Value::Byte(0)),
                            ("color".to_owned(), Value::String("black".into())),
                            ("messages".to_owned(), Value::List(front.collect())),
                        ])),
                    ),
                    (
                        "back_text".to_owned(),
                        Value::Compound(HashMap::from([
                            ("has_glowing_text".to_owned(), Value::Byte(0)),
                            ("color".to_owned(), Value::String("black".into())),
                            ("messages".to_owned(), Value::List(back.collect())),
                        ])),
                    ),
                    ("id".to_owned(), Value::String("minecraft:sign".to_owned())),
                ]))
            }),
            BlockEntity::Comparator { output_strength } => Some({
                nbt::Blob::with_content(HashMap::from([
                    (
                        "OutputSignal".to_owned(),
                        Value::Int(*output_strength as i32),
                    ),
                    (
                        "id".to_owned(),
                        Value::String("minecraft:comparator".to_owned()),
                    ),
                ]))
            }),
            BlockEntity::Container { inventory, ty, .. } => Some({
                let mut items = Vec::new();
                for entry in inventory {
                    let mut nbt = HashMap::from([
                        ("Count".to_owned(), nbt::Value::Byte(entry.count)),
                        (
                            "id".to_owned(),
                            nbt::Value::String(Item::from_id(entry.id).get_name().to_owned()),
                        ),
                        ("Slot".to_owned(), nbt::Value::Byte(entry.slot)),
                    ]);
                    if let Some(tag) = &entry.nbt {
                        let tag = nbt::Blob::from_reader(&mut tag.as_slice())
                            .expect("item tags are stored as NBT");
                        nbt.insert("tag".to_owned(), Value::Compound(tag.content));
                    }
                    items.push(nbt::Value::Compound(nbt));
                }
                nbt::Blob::with_content(HashMap::from([
                    ("id".to_owned(), Value::String(ty.to_string())),
                    ("Items".to_owned(), Value::List(items)),
                ]))
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
