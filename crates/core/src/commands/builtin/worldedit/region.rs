use super::selection::update_selection;
use crate::{
    commands::{
        argument::ArgumentType,
        context::ExecutionContext,
        define::Command,
        error::{CommandError, CommandResult, RuntimeError},
        registry::CommandRegistry,
        value::{Direction, ReplacementOperand},
    },
    item_stack::ItemStack,
    utils,
    worldedit::{
        clear_area, create_clipboard, mask::WorldEditMask, paste_clipboard,
        pattern::WorldEditPattern, region_positions, update,
    },
};
use itertools::Itertools;
use mchprs_blocks::{
    block_entities::{BlockEntity, ContainerType, InventoryEntry},
    block_store::{BlockStore, BlockView},
    blocks::{Block, HopperFacing},
    items::Item,
    signal_strength::SignalStrength,
    BlockDirection, BlockFacing, BlockPos,
};
use mchprs_network::packets::clientbound::*;
use rustc_hash::FxHashMap;
use std::time::Instant;

pub(super) fn register(registry: &mut CommandRegistry) {
    fn exec_set(
        ctx: &mut ExecutionContext<'_>,
        mask: Option<&WorldEditMask>,
        pattern: &WorldEditPattern,
    ) -> CommandResult<()> {
        let (first_pos, second_pos) = ctx.get_selection()?;

        let origin = first_pos.min(second_pos);
        ctx.capture_undo_regions([(first_pos, second_pos)], origin)?;

        let start_time = Instant::now();
        let mut blocks_updated = 0;

        let world = ctx.world_mut()?;
        for pos in region_positions(first_pos, second_pos) {
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
            .arg("maskOrPattern", ArgumentType::Replacement)
            .optional("pattern", ArgumentType::Pattern)
            .executes(|ctx| {
                let first: &ReplacementOperand = ctx.arg("maskOrPattern")?;
                if let Some(pattern) = ctx.arg_opt("pattern")? {
                    let mask = first
                        .mask
                        .as_ref()
                        .ok_or_else(|| CommandError::runtime("The first operand must be a mask"))?;
                    return exec_set(ctx, Some(mask), pattern);
                }

                let pattern = first
                    .pattern
                    .as_ref()
                    .ok_or_else(|| CommandError::runtime("Expected a pattern after the mask"))?;
                exec_set(ctx, Some(&WorldEditMask::existing()), pattern)
            }),
    );

    registry.register(
        Command::new("/count")
            .arg("mask", ArgumentType::Mask)
            .executes(|ctx| {
                let mask: &WorldEditMask = ctx.arg("mask")?;
                let (first_pos, second_pos) = ctx.get_selection()?;

                let start_time = Instant::now();
                let world = ctx.world();
                let blocks_counted = region_positions(first_pos, second_pos)
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
                    let clipboard = ctx.get_clipboard()?;
                    for position in clipboard.region().positions() {
                        *block_counts
                            .entry(clipboard.block(position).get_name())
                            .or_insert(0) += 1;
                    }
                } else {
                    let (first_pos, second_pos) = ctx.get_selection()?;
                    let world = ctx.world();
                    for pos in region_positions(first_pos, second_pos) {
                        *block_counts.entry(world.block(pos).get_name()).or_insert(0) += 1;
                    }
                }

                let block_counts = block_counts
                    .into_iter()
                    .sorted_by_key(|(_, count)| *count)
                    .collect_vec();

                let total_blocks: u32 = block_counts.iter().map(|(_, count)| *count).sum();
                let mut result = format!("# total blocks: {}\n", total_blocks);
                for (name, count) in block_counts {
                    let percentage = (count as f64 / total_blocks as f64) * 100.0;
                    result.push_str(&format!("{:>6} ({:>5.2}%) {}\n", count, percentage, name));
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

                let (first_pos, second_pos) = ctx.get_selection()?;
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
                ctx.worldedit_message(&format!("{} ({:?})", message, start_time.elapsed()))
            }),
    );

    registry.register(
        Command::new("/update")
            .flag('p', "plot", "Update the entire plot")
            .executes(|ctx| {
                if ctx.flag("plot") {
                    let region = ctx.world().region();
                    update(ctx.world_mut()?, region.min(), region.max());
                    ctx.worldedit_message("Updated entire plot.")
                } else {
                    let (first_pos, second_pos) = ctx.get_selection()?;
                    update(ctx.world_mut()?, first_pos, second_pos);
                    ctx.worldedit_message("Updated selection.")
                }
            }),
    );

    registry.register(
        Command::new("/replacecontainer")
            .alias("/rc")
            .arg("fromOrTo", ArgumentType::ContainerType)
            .optional("to", ArgumentType::ContainerType)
            .executes(exec_replacecontainer),
    );

    registry.register(Command::new("/wand").executes(|ctx| {
        let item = ItemStack {
            count: 1,
            item_type: Item::WoodenAxe,
            nbt: None,
        };
        let player = ctx.player_mut()?;
        let slot = 36 + player.selected_slot;
        player.set_inventory_slot(slot, Some(item.clone()));

        let entity_equipment = CSetEquipment {
            entity_id: player.entity_id as i32,
            equipment: vec![CSetEquipmentEquipment {
                slot: 0,
                item: Some(utils::encode_slot_data(&item)),
            }],
        }
        .encode();

        for packet_sender in ctx.world().packet_senders.values() {
            packet_sender.send_packet(&entity_equipment);
        }

        ctx.worldedit_message("Wand item given.")
    }));
}

fn exec_replacecontainer(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let first: ContainerType = ctx.arg("fromOrTo")?;
    let (from, to) = match ctx.arg_opt("to")? {
        Some(to) => (Some(first), to),
        None => (None, first),
    };
    let (first_pos, second_pos) = ctx.get_selection()?;

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

    let world = ctx.world_mut()?;
    for pos in region_positions(first_pos, second_pos) {
        let block_ty = match world.block(pos) {
            Block::Furnace { .. } => ContainerType::Furnace,
            Block::Barrel { .. } => ContainerType::Barrel,
            Block::Hopper { .. } => ContainerType::Hopper,
            _ => continue,
        };

        let (current_ty, ss) = match world.block_entity(pos) {
            Some(BlockEntity::Container {
                comparator_override,
                ty,
                ..
            }) => (*ty, *comparator_override),
            _ => (block_ty, SignalStrength::Zero),
        };

        if from.is_some_and(|from| from != current_ty) {
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
        world.set_block_entity(pos, Some(new_entity));
        world.set_block(pos, new_block);
    }

    ctx.worldedit_message(&format!(
        "Your selection was replaced successfully. ({:?})",
        start_time.elapsed()
    ))
}
