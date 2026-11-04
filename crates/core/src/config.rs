use serde::{Deserialize, Serialize};
use std::collections::HashMap;
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
    pub autosave_interval_seconds: i64,
    pub velocity: VelocityConfig,
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
            autosave_interval_seconds: 0,
            velocity: VelocityConfig::default(),
            command_aliases: HashMap::new(),
        }
    }
}

impl ServerConfig {
    pub fn load() -> (Option<ServerConfig>, Vec<String>) {
        let text = match fs::read_to_string(PATH) {
            Ok(text) => text,
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                let defaults = toml::to_string(&ServerConfig::default())
                    .expect("the default config serializes");
                if let Err(error) = fs::write(PATH, defaults) {
                    warn!("Failed to write the default {PATH}: {error}");
                }
                return (Some(ServerConfig::default()), Vec::new());
            }
            Err(error) => return (None, vec![format!("Failed to read {PATH}: {error}")]),
        };
        let table: toml::Table = match toml::from_str(&text) {
            Ok(table) => table,
            Err(error) => return (None, vec![format!("Invalid {PATH}: {error}")]),
        };
        let mut problems = Vec::new();
        if table.contains_key("luckperms") {
            problems.push(format!(
                "LuckPerms is not supported: every player can use every command and edit every \
                 plot. Remove [luckperms] from {PATH} to start anyway"
            ));
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
                return (None, problems);
            }
        };
        #[cfg(not(feature = "whitelist"))]
        if config.whitelist {
            problems.push(format!(
                "`whitelist = true` in {PATH} requires the `whitelist` feature (build with \
                 `--features whitelist`, or set it to false)"
            ));
        }
        (Some(config), problems)
    }
}

#[derive(Serialize, Deserialize, Default)]
#[serde(default)]
pub struct VelocityConfig {
    pub enabled: bool,
    pub secret: String,
}
