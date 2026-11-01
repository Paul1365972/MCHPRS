#![deny(rust_2018_idioms)]

#[macro_use]
mod utils;
mod commands;
mod config;
mod interaction;
mod item_stack;
mod permissions;
mod player;
mod plot;
#[cfg(feature = "whitelist")]
mod profile;
pub mod server;
mod worldedit;

#[macro_use]
extern crate bitflags;
