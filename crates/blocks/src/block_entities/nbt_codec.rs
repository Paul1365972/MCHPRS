use super::{BlockEntity, ContainerType, InventoryEntry, SignBlockEntity};
use crate::items::Item;
use crate::signal_strength::UncappedSignalStrength;
use fastnbt::Value;
use std::collections::HashMap;

const EMPTY_SIGN_ROW: &str = r#""""#;

macro_rules! nbt_unwrap_val {
    // I'm not sure if path is the right type here.
    // It works though!
    ($e:expr, $p:path) => {
        match $e {
            $p(val) => val,
            _ => return None,
        }
    };
}

impl BlockEntity {
    // Lenient like vanilla's ContainerHelper: bad slots are skipped, later entries win.
    fn load_container(items: Option<&Value>, ty: ContainerType) -> BlockEntity {
        let mut inventory: Vec<InventoryEntry> = Vec::new();
        let Some(Value::List(items)) = items else {
            return BlockEntity::Container { inventory, ty };
        };
        for item in items {
            let Value::Compound(item) = item else {
                continue;
            };
            let slot = nbt_get_int(item, "Slot").unwrap_or(0) as u8;
            if slot >= ty.num_slots() {
                continue;
            }
            let slot = slot as i8;
            inventory.retain(|entry| entry.slot != slot);
            inventory.extend(load_stack(item, slot));
        }
        BlockEntity::Container { inventory, ty }
    }

    pub fn is_supported(id: &str) -> bool {
        matches!(
            id.trim_start_matches("minecraft:"),
            "comparator" | "furnace" | "barrel" | "hopper" | "sign"
        )
    }

    pub fn from_nbt(id: &str, nbt: &HashMap<String, Value>) -> Option<BlockEntity> {
        match id.trim_start_matches("minecraft:") {
            "comparator" => Some(BlockEntity::Comparator {
                output_strength: UncappedSignalStrength::saturating_from(
                    nbt_get_int(nbt, "OutputSignal").unwrap_or(0),
                ),
            }),
            "furnace" => Some(BlockEntity::load_container(
                nbt.get("Items"),
                ContainerType::Furnace,
            )),
            "barrel" => Some(BlockEntity::load_container(
                nbt.get("Items"),
                ContainerType::Barrel,
            )),
            "hopper" => Some(BlockEntity::load_container(
                nbt.get("Items"),
                ContainerType::Hopper,
            )),
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

    pub fn to_nbt(&self, sign_only: bool) -> Option<HashMap<String, Value>> {
        if sign_only && !matches!(self, BlockEntity::Sign(_)) {
            return None;
        }

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
                HashMap::from([
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
                ])
            }),
            BlockEntity::Comparator { output_strength } => Some(HashMap::from([
                (
                    "OutputSignal".to_owned(),
                    Value::Int(output_strength.get().into()),
                ),
                (
                    "id".to_owned(),
                    Value::String("minecraft:comparator".to_owned()),
                ),
            ])),
            BlockEntity::Container { inventory, ty, .. } => Some({
                let mut items = Vec::new();
                for entry in inventory {
                    let mut nbt = HashMap::from([
                        ("Count".to_owned(), Value::Byte(entry.count)),
                        (
                            "id".to_owned(),
                            Value::String(Item::from_id(entry.id).get_name().to_owned()),
                        ),
                        ("Slot".to_owned(), Value::Byte(entry.slot)),
                    ]);
                    if let Some(tag) = &entry.nbt {
                        let tag = fastnbt::from_bytes(tag).expect("item tags are stored as NBT");
                        nbt.insert("tag".to_owned(), Value::Compound(tag));
                    }
                    items.push(Value::Compound(nbt));
                }
                HashMap::from([
                    ("id".to_owned(), Value::String(ty.to_string())),
                    ("Items".to_owned(), Value::List(items)),
                ])
            }),
        }
    }
}

fn load_stack(item: &HashMap<String, Value>, slot: i8) -> Option<InventoryEntry> {
    let count = nbt_get_int(item, "Count").unwrap_or(0) as i8;
    if count <= 0 {
        return None;
    }
    let name = nbt_unwrap_val!(nbt_case_insensitive(item, "Id")?, Value::String);
    let item_type = Item::from_name(name).filter(|&item_type| item_type != Item::Air)?;
    let nbt = match item.get("tag") {
        Some(Value::Compound(tag)) => Some(fastnbt::to_bytes(tag).ok()?),
        _ => None,
    };
    Some(InventoryEntry {
        slot,
        count,
        id: item_type.get_id(),
        nbt,
    })
}

fn nbt_case_insensitive<'a>(compound: &'a HashMap<String, Value>, name: &str) -> Option<&'a Value> {
    // It's not really case insensitive, this is really just checking both cases of the first character.
    compound
        .get(name)
        .or_else(|| compound.get(&name.to_lowercase()))
}

fn nbt_get_int(compound: &HashMap<String, Value>, name: &str) -> Option<i32> {
    let value = nbt_case_insensitive(compound, name)?;
    match value {
        Value::Byte(val) => Some(*val as i32),
        Value::Int(val) => Some(*val),
        _ => None,
    }
}
