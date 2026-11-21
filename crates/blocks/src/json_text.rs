//! Block entities stored text components as JSON strings until Minecraft 1.21.5.

use mchprs_utils::map;
use nbt::Value;
use serde_json::Value as Json;
use std::collections::HashMap;

pub const NBT_TEXT_DATA_VERSION: i32 = 4290;

pub fn component_from_json(json: &str) -> Value {
    let component = match serde_json::from_str(json) {
        Ok(json) => component(&json).unwrap_or_else(|| text("")),
        Err(_) => text(json),
    };
    Value::Compound(component)
}

fn text(text: &str) -> HashMap<String, Value> {
    map! { "text" => Value::String(text.to_owned()) }
}

fn component(json: &Json) -> Option<HashMap<String, Value>> {
    match json {
        Json::String(string) => Some(text(string)),
        Json::Array(parts) => {
            // The first part is the parent of the others, which is what `extra` means
            let mut parts = parts.iter().filter_map(component);
            let mut first = parts.next()?;
            let mut siblings: Vec<Value> = parts.map(Value::Compound).collect();
            if !siblings.is_empty() {
                if let Some(Value::List(extra)) = first.remove("extra") {
                    siblings.splice(0..0, extra);
                }
                first.insert("extra".to_owned(), Value::List(siblings));
            }
            Some(first)
        }
        Json::Object(fields) => {
            let mut compound: HashMap<String, Value> = fields
                .iter()
                .filter_map(|(key, value)| field(key, value))
                .collect();
            compound
                .entry("text".to_owned())
                .or_insert_with(|| Value::String(String::new()));
            Some(compound)
        }
        _ => None,
    }
}

fn field(key: &str, value: &Json) -> Option<(String, Value)> {
    let value = match key {
        "text" | "color" => Value::String(value.as_str()?.to_owned()),
        "bold" | "italic" | "underlined" | "strikethrough" | "obfuscated" => {
            Value::Byte(flag(value)? as i8)
        }
        "extra" => {
            let parts = value.as_array()?.iter().filter_map(component);
            Value::List(parts.map(Value::Compound).collect())
        }
        _ => return None,
    };
    Some((key.to_owned(), value))
}

/// Old data also has these as the strings `"true"` and `"false"`
fn flag(value: &Json) -> Option<bool> {
    match value {
        Json::Bool(flag) => Some(*flag),
        Json::String(flag) => Some(flag.eq_ignore_ascii_case("true")),
        _ => None,
    }
}
