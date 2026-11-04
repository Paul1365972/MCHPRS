use mchprs_redpiler::backend::Backend;
use mchprs_redpiler_backends::{BackendSetup, BACKENDS};
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
    pub fn backends(&self) -> Result<Backends, Vec<String>> {
        for name in self.flags.keys() {
            if mchprs_redpiler_backends::named(name).is_err() {
                warn!("Ignoring `redpiler.flags.{name}` in {PATH}: unknown backend");
            }
        }
        let mut problems = Vec::new();
        let mut all = Vec::new();
        for &backend in BACKENDS {
            match self.setup(backend) {
                Ok(setup) => all.push(setup),
                Err(error) => problems.push(format!(
                    "Invalid `redpiler.flags.{}` in {PATH}: {error}",
                    backend.name()
                )),
            }
        }
        let default = match mchprs_redpiler_backends::named(&self.default_backend) {
            Ok(backend) => all
                .iter()
                .find(|setup| setup.backend().name() == backend.name())
                .cloned(),
            Err(error) => {
                problems.push(format!(
                    "Invalid `redpiler.default_backend` in {PATH}: {error}"
                ));
                None
            }
        };
        match default {
            Some(default) if problems.is_empty() => Ok(Backends { default, all }),
            _ => Err(problems),
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
