use super::error::{CommandError, CommandResult};
use crate::{config::CONFIG, utils::HyphenatedUUID};
use mchprs_commands::reader::Reader;
use std::{
    fs::{self, File},
    io::{self, Write},
    path::{Path, PathBuf},
};

fn valid_schematic_filename(filename: &str) -> bool {
    let Some(stem) = filename
        .strip_suffix(".schem")
        .or_else(|| filename.strip_suffix(".schematic"))
    else {
        return false;
    };
    !stem.is_empty() && stem.chars().all(Reader::allowed_word)
}

fn schematic_directory(player_uuid: u128) -> PathBuf {
    let mut path = PathBuf::from("./schems");
    if CONFIG.schemati {
        path.push(HyphenatedUUID(player_uuid).to_string());
    }
    path
}

pub(super) fn schematic_path(player_uuid: u128, filename: &str) -> CommandResult<PathBuf> {
    if !valid_schematic_filename(filename) {
        return Err(CommandError::runtime("Filename is invalid"));
    }
    Ok(schematic_directory(player_uuid).join(filename))
}

pub(super) fn save_schematic(path: &Path, bytes: &[u8], overwrite: bool) -> io::Result<()> {
    if let Some(directory) = path.parent() {
        fs::create_dir_all(directory)?;
    }
    File::options()
        .write(true)
        .create(true)
        .create_new(!overwrite)
        .truncate(overwrite)
        .open(path)?
        .write_all(bytes)
}

pub fn schematic_names(player_uuid: u128) -> io::Result<Vec<String>> {
    let entries = match fs::read_dir(schematic_directory(player_uuid)) {
        Ok(entries) => entries,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(error) => return Err(error),
    };
    let mut names = Vec::new();
    for entry in entries {
        let entry = entry?;
        if entry.file_type()?.is_file()
            && let Some(name) = entry.file_name().to_str()
            && valid_schematic_filename(name)
        {
            names.push(name.to_owned());
        }
    }
    names.sort();
    Ok(names)
}
