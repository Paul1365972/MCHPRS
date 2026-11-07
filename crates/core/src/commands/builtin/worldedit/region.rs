use super::selection::update_selection;
use crate::commands::argument::ArgumentType;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandResult, RuntimeError};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::item_stack::ItemStack;
use crate::utils;
use crate::worldedit::mask::WorldEditMask;
use crate::worldedit::pattern::WorldEditPattern;
use crate::worldedit::{clear_area, create_clipboard, paste_clipboard};
use itertools::Itertools;
use mchprs_blocks::block_entities::{BlockEntity, ContainerType};
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::blocks::{Block, HopperFacing};
use mchprs_blocks::items::Item;
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::UncappedSignalStrength;
use mchprs_blocks::{BlockDirection, BlockFacing, BlockPos};
use mchprs_network::packets::clientbound::*;
use mchprs_redstone::update_region;
use rustc_hash::FxHashMap;
use std::time::Instant;

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("/set")
            .arg("pattern", ArgumentType::Pattern)
            .executes(|ctx| {
                let pattern = ctx.arg("pattern")?;
                exec_set(ctx, None, pattern)
            }),
    );

    registry.register(
        Command::new("/replace")
            .alias("/re")
            .alias("/rep")
            .optional("mask", ArgumentType::Mask)
            .arg("pattern", ArgumentType::Pattern)
            .executes(|ctx| {
                let existing = WorldEditMask::existing();
                let mask = ctx.arg_opt("mask")?.unwrap_or(&existing);
                let pattern = ctx.arg("pattern")?;
                exec_set(ctx, Some(mask), pattern)
            }),
    );

    registry.register(
        Command::new("/count")
            .arg("mask", ArgumentType::Mask)
            .executes(|ctx| {
                let mask: &WorldEditMask = ctx.arg("mask")?;
                let (first_pos, second_pos) = ctx.selection()?;

                let start_time = Instant::now();
                let world = ctx.world();
                let blocks_counted = Region::new(first_pos, second_pos)
                    .positions()
                    .filter(|&pos| mask.matches(world, pos))
                    .count();

                ctx.worldedit_message(&format!(
                    "Counted {} block(s) ({:?})",
                    blocks_counted,
                    start_time.elapsed()
                ))
            }),
    );

    registry.register(
        Command::new("/distr")
            .flag('c', "clipboard", "Get distribution of clipboard")
            .executes(|ctx| {
                let start_time = Instant::now();
                let mut block_counts = FxHashMap::default();

                if ctx.flag("clipboard") {
                    let clipboard = ctx.clipboard()?;
                    for pos in clipboard.region().positions() {
                        *block_counts
                            .entry(clipboard.block(pos).get_name())
                            .or_insert(0) += 1;
                    }
                } else {
                    let (first_pos, second_pos) = ctx.selection()?;
                    let world = ctx.world();
                    for pos in Region::new(first_pos, second_pos).positions() {
                        *block_counts.entry(world.block(pos).get_name()).or_insert(0) += 1;
                    }
                }

                let block_counts = block_counts
                    .into_iter()
                    .sorted_by_key(|(_, count)| *count)
                    .collect_vec();

                let total_blocks: u32 = block_counts.iter().map(|(_, count)| *count).sum();
                let mut result = format!("# total blocks: {total_blocks}\n");
                for (name, count) in block_counts {
                    let percentage = (count as f64 / total_blocks as f64) * 100.0;
                    result.push_str(&format!("{count:>6} ({percentage:>5.2}%) {name}\n"));
                }

                result.push_str(&format!("({:?})", start_time.elapsed()));
                ctx.worldedit_message(&result)
            }),
    );

    registry.register(
        Command::new("/move")
            .optional("count", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .flag('a', "ignore-air", "Move without air blocks")
            .flag('s', "shift-selection", "Shift selection with the move")
            .executes(|ctx| {
                let count = ctx.arg_or("count", 1)?;
                let direction = ctx.arg_or("direction", Direction::Me)?;
                let direction = direction.resolve(ctx.player()?.get_facing());
                let ignore_air = ctx.flag("ignore-air");
                let shift_selection = ctx.flag("shift-selection");

                let (first_pos, second_pos) = ctx.selection()?;
                let start_time = Instant::now();

                let offset_amount = direction.offset_pos(BlockPos::zero(), count);
                let selected = Region::new(first_pos, second_pos);
                let destination = selected.translated(offset_amount);
                if ctx.clip_region(destination).is_none() {
                    return Err(RuntimeError::DestinationOutsidePlot.into());
                }

                let clipped = ctx.capture_undo_regions([selected, destination], selected.min())?;

                let origin = BlockPos::zero();
                let clipboard = create_clipboard(ctx.world(), origin, first_pos, second_pos);
                clear_area(ctx.world_mut()?, first_pos, second_pos);
                paste_clipboard(
                    ctx.world_mut()?,
                    &clipboard,
                    origin + offset_amount,
                    ignore_air,
                );

                if shift_selection {
                    update_selection(
                        ctx,
                        Some((first_pos, second_pos)),
                        (first_pos + offset_amount, second_pos + offset_amount),
                    )?;
                }

                let message = if clipped {
                    "Your selection was moved (clipped to plot bounds)."
                } else {
                    "Your selection was moved."
                };
                ctx.worldedit_message(&format!("{message} ({:?})", start_time.elapsed()))
            }),
    );

    registry.register(
        Command::new("/update")
            .flag('p', "plot", "Update the entire plot")
            .executes(|ctx| {
                if ctx.flag("plot") {
                    let region = ctx.plot().world.region();
                    update_region(ctx.world_mut()?, region);
                    ctx.worldedit_message("Updated entire plot.")
                } else {
                    let (first_pos, second_pos) = ctx.selection()?;
                    update_region(ctx.world_mut()?, Region::new(first_pos, second_pos));
                    ctx.worldedit_message("Updated selection.")
                }
            }),
    );

    registry.register(
        Command::new("/replacecontainer")
            .alias("/rc")
            .optional("from", ArgumentType::ContainerType)
            .arg("to", ArgumentType::ContainerType)
            .executes(exec_replace_container),
    );

    registry.register(Command::new("/wand").executes(|ctx| {
        let item = ItemStack {
            count: 1,
            item_type: Item::WoodenAxe,
            nbt: None,
        };
        let player = ctx.player_mut()?;
        let slot = 36 + player.selected_slot;
        let entity_equipment = CSetEquipment {
            entity_id: player.entity_id as i32,
            equipment: vec![CSetEquipmentEquipment {
                slot: 0,
                item: Some(utils::encode_slot_data(&item)),
            }],
        }
        .encode();
        player.set_inventory_slot(slot, Some(item));

        for packet_sender in ctx.plot().world.packet_senders.values() {
            packet_sender.send_packet(&entity_equipment);
        }

        ctx.worldedit_message("Wand item given.")
    }));
}

fn exec_set(
    ctx: &mut ExecutionContext<'_>,
    mask: Option<&WorldEditMask>,
    pattern: &WorldEditPattern,
) -> CommandResult<()> {
    let (first_pos, second_pos) = ctx.selection()?;

    let region = Region::new(first_pos, second_pos);
    ctx.capture_undo_regions([region], region.min())?;

    let start_time = Instant::now();
    let mut blocks_updated = 0;

    let world = ctx.world_mut()?;
    for pos in region.positions() {
        if mask.is_none_or(|mask| mask.matches(world, pos)) && pattern.place(world, pos) {
            blocks_updated += 1;
        }
    }

    ctx.worldedit_message(&format!(
        "Operation completed: {} block(s) affected ({:?})",
        blocks_updated,
        start_time.elapsed()
    ))
}

fn exec_replace_container(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let from: Option<ContainerType> = ctx.arg_opt("from")?;
    let to: ContainerType = ctx.arg("to")?;
    let (first_pos, second_pos) = ctx.selection()?;

    let region = Region::new(first_pos, second_pos);
    ctx.capture_undo_regions([region], region.min())?;

    let start_time = Instant::now();

    let new_block = match to {
        ContainerType::Furnace => Block::Furnace {
            facing: BlockDirection::North,
            lit: false,
        },
        ContainerType::Barrel => Block::Barrel {
            open: false,
            facing: BlockFacing::Up,
        },
        ContainerType::Hopper => Block::Hopper {
            enabled: false,
            facing: HopperFacing::Down,
        },
    };

    let world = ctx.world_mut()?;
    let mut unchanged = 0;
    for pos in Region::new(first_pos, second_pos).positions() {
        let block_ty = match world.block(pos) {
            Block::Furnace { .. } => ContainerType::Furnace,
            Block::Barrel { .. } => ContainerType::Barrel,
            Block::Hopper { .. } => ContainerType::Hopper,
            _ => continue,
        };

        let (current_ty, strength) = match world.block_entity(pos) {
            Some(BlockEntity::Container { inventory, ty }) => (*ty, ty.signal_strength(inventory)),
            _ => (block_ty, UncappedSignalStrength::ZERO),
        };

        if from.is_some_and(|from| from != current_ty) {
            continue;
        }

        let Some(inventory) = to.inventory_for(strength) else {
            unchanged += 1;
            continue;
        };
        let new_entity = BlockEntity::Container { inventory, ty: to };
        world.set_block_entity(pos, Some(new_entity));
        world.set_block(pos, new_block);
    }

    let unchanged = match unchanged {
        0 => String::new(),
        _ => format!(" Kept {unchanged} containers whose strength the new type can't produce."),
    };
    ctx.worldedit_message(&format!(
        "Your selection was replaced successfully.{unchanged} ({:?})",
        start_time.elapsed()
    ))
}
