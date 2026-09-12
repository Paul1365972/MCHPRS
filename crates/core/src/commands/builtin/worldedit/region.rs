use super::selection::update_selection;
use crate::commands::argument::ArgumentType;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandResult, RuntimeError};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::utils;
use crate::worldedit::pattern::WorldEditPattern;
use crate::worldedit::{clear_area, region_positions, update};
use mchprs_blocks::block_entities::{BlockEntity, ContainerType, InventoryEntry};
use mchprs_blocks::blocks::{Block, HopperFacing};
use mchprs_blocks::items::{Item, ItemStack};
use mchprs_blocks::{BlockDirection, BlockFacing, BlockPos};
use mchprs_network::packets::clientbound::*;
use mchprs_schematic::{create_clipboard, paste_clipboard};
use mchprs_world::World;
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
            .arg("from", ArgumentType::Pattern)
            .arg("to", ArgumentType::Pattern)
            .executes(|ctx| {
                let from = ctx.arg("from")?;
                let to = ctx.arg("to")?;
                exec_set(ctx, Some(from), to)
            }),
    );

    registry.register(
        Command::new("/count")
            .arg("mask", ArgumentType::Pattern)
            .executes(|ctx| {
                let mask: &WorldEditPattern = ctx.arg("mask")?;
                let (first_pos, second_pos) = ctx.selection()?;

                let start_time = Instant::now();
                let world = ctx.world();
                let blocks_counted = region_positions(first_pos, second_pos)
                    .filter(|&pos| mask.matches(world.get_block(pos)))
                    .count();

                ctx.worldedit_message(&format!(
                    "Counted {} block(s) ({:?})",
                    blocks_counted,
                    start_time.elapsed()
                ))
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
                let destination = (
                    first_pos.min(second_pos) + offset_amount,
                    first_pos.max(second_pos) + offset_amount,
                );
                if ctx.clip_region(destination.0, destination.1).is_none() {
                    return Err(RuntimeError::DestinationOutsidePlot.into());
                }

                let clipped = ctx.capture_undo_regions(
                    [(first_pos, second_pos), destination],
                    first_pos.min(second_pos),
                )?;

                let origin = BlockPos::zero();
                let clipboard = create_clipboard(ctx.world(), origin, first_pos, second_pos);
                clear_area(ctx.world_mut(), first_pos, second_pos);
                paste_clipboard(
                    ctx.world_mut(),
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
                    let corners = ctx.plot().world.get_corners();
                    update(ctx.world_mut(), corners.0, corners.1);
                    ctx.worldedit_message("Updated entire plot.")
                } else {
                    let (first_pos, second_pos) = ctx.selection()?;
                    update(ctx.world_mut(), first_pos, second_pos);
                    ctx.worldedit_message("Updated selection.")
                }
            }),
    );

    registry.register(
        Command::new("/replacecontainer")
            .alias("/rc")
            .arg("from", ArgumentType::ContainerType)
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
    from: Option<&WorldEditPattern>,
    pattern: &WorldEditPattern,
) -> CommandResult<()> {
    let (first_pos, second_pos) = ctx.selection()?;

    let origin = first_pos.min(second_pos);
    ctx.capture_undo_regions([(first_pos, second_pos)], origin)?;

    let start_time = Instant::now();
    let mut blocks_updated = 0;

    let world = ctx.world_mut();
    for pos in region_positions(first_pos, second_pos) {
        if from.is_none_or(|from| from.matches(world.get_block(pos)))
            && world.set_block_raw(pos, pattern.pick().get_id())
        {
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
    let from: ContainerType = ctx.arg("from")?;
    let to: ContainerType = ctx.arg("to")?;
    let (first_pos, second_pos) = ctx.selection()?;

    let origin = first_pos.min(second_pos);
    ctx.capture_undo_regions([(first_pos, second_pos)], origin)?;

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

    let world = ctx.world_mut();
    for pos in region_positions(first_pos, second_pos) {
        let block_ty = match world.get_block(pos) {
            Block::Furnace { .. } => ContainerType::Furnace,
            Block::Barrel { .. } => ContainerType::Barrel,
            Block::Hopper { .. } => ContainerType::Hopper,
            _ => continue,
        };

        let (current_ty, ss) = match world.get_block_entity(pos) {
            Some(BlockEntity::Container {
                comparator_override,
                ty,
                ..
            }) => (*ty, *comparator_override),
            _ => (block_ty, 0),
        };

        if from != current_ty {
            continue;
        }

        let items_needed = to.items_needed_for_signal_strength(ss);
        let inventory = (0..items_needed)
            .step_by(64)
            .enumerate()
            .map(|(slot, items_added)| InventoryEntry {
                id: Item::Redstone.get_id(),
                slot: slot as i8,
                count: (items_needed - items_added).min(64) as i8,
                nbt: None,
            })
            .collect();

        let new_entity = BlockEntity::Container {
            comparator_override: ss,
            inventory,
            ty: to,
        };
        world.set_block_entity(pos, new_entity);
        world.set_block(pos, new_block);
    }

    ctx.worldedit_message(&format!(
        "Your selection was replaced successfully. ({:?})",
        start_time.elapsed()
    ))
}
