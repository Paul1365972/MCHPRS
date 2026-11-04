use mchprs_commands::flag::{FlagValue, FlagValues};
use mchprs_redpiler::backend::Backend;
use mchprs_redpiler_backends::BACKENDS;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};
use std::fs;
use std::io;
use tracing::warn;

const PATH: &str = "Config.toml";

#[derive(Serialize, Deserialize)]
#[serde(default)]
pub struct ServerConfig {
    pub bind_address: String,
    pub motd: String,
    pub chat_format: String,
    pub max_players: i64,
    pub view_distance: i64,
    pub whitelist: bool,
    pub schemati: bool,
    pub block_in_hitbox: bool,
    pub autosave_interval_seconds: u64,
    pub velocity: VelocityConfig,
    pub redpiler: RedpilerConfig,
    pub command_aliases: HashMap<String, String>,
}

impl Default for ServerConfig {
    fn default() -> Self {
        ServerConfig {
            bind_address: "0.0.0.0:25565".to_owned(),
            motd: "Minecraft High Performance Redstone Server".to_owned(),
            chat_format: "<{username}> {message}".to_owned(),
            max_players: 99999,
            view_distance: 8,
            whitelist: false,
            schemati: false,
            block_in_hitbox: true,
            autosave_interval_seconds: 300,
            velocity: VelocityConfig::default(),
            redpiler: RedpilerConfig::default(),
            command_aliases: HashMap::new(),
        }
    }
}

#[derive(Serialize, Deserialize, Default)]
pub struct VelocityConfig {
    pub enabled: bool,
    pub secret: String,
}

#[derive(Serialize, Deserialize)]
pub struct RedpilerConfig {
    pub default_engine: String,
    #[serde(default)]
    pub backends: BTreeMap<String, toml::Table>,
}

impl Default for RedpilerConfig {
    fn default() -> Self {
        RedpilerConfig {
            default_engine: BACKENDS[0].name().to_owned(),
            backends: BTreeMap::new(),
        }
    }
}

pub struct Engines {
    pub default: &'static dyn Backend,
    pub flags: Vec<(&'static dyn Backend, FlagValues)>,
}

impl RedpilerConfig {
    pub fn engines(&self) -> Result<Engines, Vec<String>> {
        for name in self.backends.keys() {
            if mchprs_redpiler_backends::named(name).is_err() {
                warn!("Ignoring `redpiler.backends.{name}` in {PATH}: unknown engine");
            }
        }
        let mut problems = Vec::new();
        let default = mchprs_redpiler_backends::named(&self.default_engine)
            .map_err(|error| {
                problems.push(format!(
                    "Invalid `redpiler.default_engine` in {PATH}: {error}"
                ))
            })
            .ok();
        let mut flags = Vec::new();
        for &backend in BACKENDS {
            match self.flags(backend) {
                Ok(values) => flags.push((backend, values)),
                Err(problem) => problems.push(problem),
            }
        }
        match default {
            Some(default) if problems.is_empty() => Ok(Engines { default, flags }),
            _ => Err(problems),
        }
    }

    fn flags(&self, backend: &dyn Backend) -> Result<FlagValues, String> {
        let mut flags = FlagValues::default();
        let Some(table) = self.backends.get(backend.name()) else {
            return Ok(flags);
        };
        for (name, value) in table {
            let key = format!("redpiler.backends.{}.{name}", backend.name());
            let Some(&flag) = backend.flags().iter().find(|flag| flag.name() == name) else {
                warn!("Ignoring `{key}` in {PATH}: unknown flag");
                continue;
            };
            let value = match (flag.value_name(), value) {
                (None, toml::Value::Boolean(on)) => FlagValue::Switch(*on),
                (Some(_), toml::Value::String(text)) => FlagValue::Text(text.clone()),
                (Some(_), toml::Value::Integer(number)) => FlagValue::Text(number.to_string()),
                (None, _) => {
                    return Err(format!("Invalid `{key}` in {PATH}: expected true or false"))
                }
                (Some(_), _) => {
                    return Err(format!(
                        "Invalid `{key}` in {PATH}: expected text or a number"
                    ))
                }
            };
            flags.set(flag, value);
        }
        Ok(flags)
    }
}

impl ServerConfig {
    pub fn load() -> Result<ServerConfig, Vec<String>> {
        let text = match fs::read_to_string(PATH) {
            Ok(text) => text,
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                let defaults = toml::to_string(&ServerConfig::default())
                    .expect("the default config serializes");
                if let Err(error) = fs::write(PATH, defaults) {
                    warn!("Failed to write the default {PATH}: {error}");
                }
                return Ok(ServerConfig::default());
            }
            Err(error) => return Err(vec![format!("Failed to read {PATH}: {error}")]),
        };
        let table: toml::Table =
            toml::from_str(&text).map_err(|error| vec![format!("Invalid {PATH}: {error}")])?;
        let mut problems = Vec::new();
        if table.contains_key("luckperms") {
            problems.push(format!("LuckPerms is not supported: every player can use every command and edit every plot. Remove [luckperms] from {PATH} to start anyway"));
        }
        let defaults =
            toml::Table::try_from(ServerConfig::default()).expect("the default config serializes");
        for key in table
            .keys()
            .filter(|key| !defaults.contains_key(*key) && *key != "luckperms")
        {
            warn!("Ignoring unknown setting `{key}` in {PATH}");
        }
        for (key, value) in defaults {
            if !table.contains_key(&key) {
                let mut default = toml::Table::new();
                default.insert(key.clone(), value);
                let default = toml::to_string(&default).expect("a default value serializes");
                warn!(
                    "{PATH} is missing `{key}`, using the default:\n{}",
                    default.trim_end()
                );
            }
        }
        let config: ServerConfig = match table.try_into() {
            Ok(config) => config,
            Err(error) => {
                problems.push(format!("Invalid {PATH}: {error}"));
                return Err(problems);
            }
        };
        #[cfg(not(feature = "whitelist"))]
        if config.whitelist {
            problems.push(format!("`whitelist = true` in {PATH} requires the `whitelist` feature (build with `--features whitelist`, or set it to false)"));
        }
        if problems.is_empty() {
            Ok(config)
        } else {
            Err(problems)
        }
    }
}
