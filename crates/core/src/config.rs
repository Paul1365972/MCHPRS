use mchprs_redpiler::backend::Backend;
use mchprs_redpiler_backends::{BackendSetup, BACKENDS};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};
use std::io;
use std::{fs, sync::LazyLock};
use tracing::warn;

const PATH: &str = "Config.toml";

pub static CONFIG: LazyLock<ServerConfig> = LazyLock::new(|| ServerConfig::load(PATH));

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
            velocity: VelocityConfig::default(),
            redpiler: RedpilerConfig::default(),
            command_aliases: HashMap::new(),
        }
    }
}

impl ServerConfig {
    fn load(path: &str) -> ServerConfig {
        let text = match fs::read_to_string(path) {
            Ok(text) => text,
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                let defaults = toml::to_string(&ServerConfig::default())
                    .expect("the default config serializes");
                if let Err(error) = fs::write(path, defaults) {
                    warn!("Failed to write the default {path}: {error}");
                }
                return ServerConfig::default();
            }
            Err(error) => panic!("Failed to read {path}: {error}"),
        };
        let table: toml::Table = toml::from_str(&text)
            .unwrap_or_else(|error| panic!("{path} is not valid TOML: {error}"));
        if table.contains_key("luckperms") {
            panic!("{path} has a [luckperms] table, but LuckPerms support was removed, so every player could edit every plot and run every command. Remove the table to start the server");
        }
        let defaults =
            toml::Table::try_from(ServerConfig::default()).expect("the default config serializes");
        for key in table.keys().filter(|key| !defaults.contains_key(*key)) {
            warn!("{path} has `{key}`, which is not a setting, so it is ignored");
        }
        for (key, value) in defaults {
            if !table.contains_key(&key) {
                let mut default = toml::Table::new();
                default.insert(key.clone(), value);
                let default = toml::to_string(&default).expect("a default value serializes");
                warn!(
                    "{path} has no `{key}`, so it uses the default:\n{}",
                    default.trim_end()
                );
            }
        }
        table
            .try_into()
            .unwrap_or_else(|error| panic!("{path} is invalid: {error}"))
    }
}

#[derive(Serialize, Deserialize, Default)]
pub struct VelocityConfig {
    pub enabled: bool,
    pub secret: String,
}

#[derive(Serialize, Deserialize)]
pub struct RedpilerConfig {
    default_backend: String,
    #[serde(default)]
    flags: BTreeMap<String, String>,
}

impl Default for RedpilerConfig {
    fn default() -> Self {
        RedpilerConfig {
            default_backend: BACKENDS[0].name().to_owned(),
            flags: BTreeMap::new(),
        }
    }
}

impl RedpilerConfig {
    pub fn backends(&self) -> Backends {
        for name in self.flags.keys() {
            if mchprs_redpiler_backends::named(name).is_err() {
                warn!("Ignoring `redpiler.flags.{name}` in {PATH}: unknown backend");
            }
        }
        let setup = |backend: &'static dyn Backend| {
            self.setup(backend).unwrap_or_else(|error| {
                panic!(
                    "Invalid `redpiler.flags.{}` in {PATH}: {error}",
                    backend.name()
                )
            })
        };
        let default =
            mchprs_redpiler_backends::named(&self.default_backend).unwrap_or_else(|error| {
                panic!("Invalid `redpiler.default_backend` in {PATH}: {error}")
            });
        Backends {
            default: setup(default),
            all: BACKENDS.iter().map(|&backend| setup(backend)).collect(),
        }
    }

    fn setup(&self, backend: &'static dyn Backend) -> Result<BackendSetup, String> {
        let flags = self.flags.get(backend.name()).map_or("", String::as_str);
        let setup = BackendSetup::new(backend, flags)?;
        setup.configure()?;
        Ok(setup)
    }
}

pub struct Backends {
    pub default: BackendSetup,
    pub all: Vec<BackendSetup>,
}
