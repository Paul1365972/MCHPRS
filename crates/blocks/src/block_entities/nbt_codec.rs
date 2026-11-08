use super::{BlockEntity, ContainerType, InventoryEntry, SignBlockEntity};
use crate::items::Item;
use fastnbt::Value;
use std::array;
use std::collections::HashMap;

const EMPTY_TEXT_COMPONENT: &str = r#""""#;
const PRE_1_20_SIGN_ROWS: [&str; 4] = ["Text1", "Text2", "Text3", "Text4"];

impl BlockEntity {
    pub fn is_supported(id: &str) -> bool {
        matches!(
            id.trim_start_matches("minecraft:"),
            "comparator" | "furnace" | "barrel" | "hopper" | "sign"
        )
    }

    pub fn from_nbt(id: &str, nbt: &HashMap<String, Value>) -> Option<BlockEntity> {
        match id.trim_start_matches("minecraft:") {
            "comparator" => Some(BlockEntity::Comparator {
                output_strength: nbt_get_int(nbt, "OutputSignal").unwrap_or(0).clamp(0, 15) as u8,
            }),
            // Lenient: tools write sign NBT in many wrong ways.
            "sign" => {
                let row = |value: Option<&Value>| match value {
                    Some(Value::String(text)) => text.clone(),
                    _ => String::new(),
                };
                let side = |name| {
                    let messages = match nbt.get(name) {
                        Some(Value::Compound(side)) => side.get("messages"),
                        _ => None,
                    };
                    let rows = match messages {
                        Some(Value::List(rows)) => rows.as_slice(),
                        _ => &[],
                    };
                    array::from_fn(|index| row(rows.get(index)))
                };
                let sign = if nbt.contains_key(PRE_1_20_SIGN_ROWS[0]) {
                    SignBlockEntity {
                        front_rows: PRE_1_20_SIGN_ROWS.map(|key| row(nbt.get(key))),
                        back_rows: Default::default(),
                    }
                } else {
                    SignBlockEntity {
                        front_rows: side("front_text"),
                        back_rows: side("back_text"),
                    }
                };
                Some(BlockEntity::Sign(Box::new(sign)))
            }
            name => {
                let ty = name.parse().ok()?;
                Some(BlockEntity::load_container(nbt.get("Items"), ty))
            }
        }
    }

    // Vanilla's ContainerHelper.loadAllItems
    fn load_container(items: Option<&Value>, ty: ContainerType) -> BlockEntity {
        let mut inventory: Vec<InventoryEntry> = Vec::new();
        let Some(Value::List(items)) = items else {
            return BlockEntity::Container {
                inventory,
                ty,
                comparator_override: 0,
            };
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
        let num_slots = ty.num_slots();
        let fullness_sum: f32 = inventory
            .iter()
            .map(|entry| entry.count as f32 / Item::from_id(entry.id).max_stack_size() as f32)
            .sum();
        BlockEntity::Container {
            comparator_override: (if fullness_sum > 0.0 { 1.0 } else { 0.0 }
                + (fullness_sum / num_slots as f32) * 14.0)
                .floor() as u8,
            inventory,
            ty,
        }
    }

    pub fn to_nbt(&self, sign_only: bool) -> Option<HashMap<String, Value>> {
        if sign_only && !matches!(self, BlockEntity::Sign(_)) {
            return None;
        }

        match self {
            BlockEntity::Sign(sign) => Some({
                // Vanilla's SignText codec rejects an empty string.
                let message = |row: &String| {
                    Value::String(if row.is_empty() {
                        EMPTY_TEXT_COMPONENT.to_owned()
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
                    Value::Int((*output_strength).into()),
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
                    ("id".to_owned(), Value::String(format!("minecraft:{ty}"))),
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
    let Value::String(name) = nbt_get_or_lowercase(item, "Id")? else {
        return None;
    };
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

fn nbt_get_or_lowercase<'a>(compound: &'a HashMap<String, Value>, name: &str) -> Option<&'a Value> {
    compound
        .get(name)
        .or_else(|| compound.get(&name.to_lowercase()))
}

fn nbt_get_int(compound: &HashMap<String, Value>, name: &str) -> Option<i32> {
    let value = nbt_get_or_lowercase(compound, name)?;
    match value {
        Value::Byte(val) => Some(*val as i32),
        Value::Short(val) => Some(*val as i32),
        Value::Int(val) => Some(*val),
        Value::Long(val) => Some(*val as i32),
        Value::Float(val) => Some(val.floor() as i32),
        Value::Double(val) => Some(val.floor() as i32),
        _ => None,
    }
}
