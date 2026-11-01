use crate::utils::HyphenatedUUID;
use anyhow::Result;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct PlayerProfile {
    #[serde(rename = "id")]
    pub uuid: HyphenatedUUID,
    #[serde(rename = "name")]
    pub username: String,
}

impl PlayerProfile {
    pub fn lookup_by_username(username: &str) -> Result<PlayerProfile> {
        let url = format!(
            "https://api.mojang.com/users/profiles/minecraft/{}",
            username
        );
        let profile = ureq::get(&url).call()?.body_mut().read_json()?;
        Ok(profile)
    }
}
