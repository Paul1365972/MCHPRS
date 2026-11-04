#![deny(rust_2018_idioms)]

#[macro_use]
mod utils;
mod commands;
mod config;
mod interaction;
mod item_stack;
mod player;
mod plot;
#[cfg(feature = "whitelist")]
mod profile;
pub mod server;
mod shared;
mod worldedit;

#[macro_use]
extern crate bitflags;
