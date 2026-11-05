use super::selection::update_selection;
use crate::commands::argument_parser::{
    ContainerParser, DirectionParser, IntegerParser, MaskParser, PatternParser,
};
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::item_stack::ItemStack;
use crate::worldedit::mask::WorldEditMask;
use crate::worldedit::pattern::WorldEditPattern;
use crate::worldedit::{clear_area, create_clipboard, paste_clipboard};
use itertools::Itertools;
use mchprs_blocks::block_entities::{BlockEntity, ContainerType};
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::items::Item;
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::UncappedSignalStrength;
use mchprs_blocks::{BlockDirection, BlockPos};
use mchprs_network::packets::clientbound::ClientBoundPacket;
use mchprs_redstone::update_region;
use mchprs_syntax::flag::Flag;
use rustc_hash::FxHashMap;
use std::time::Instant;

const CLIPBOARD: Flag = Flag::switch("clipboard", Some('c'), "Get distribution of clipboard");
const IGNORE_AIR: Flag = Flag::switch("ignore-air", Some('a'), "Move without air blocks");
const SHIFT_SELECTION: Flag = Flag::switch(
    "shift-selection",
    Some('s'),
    "Shift selection with the move",
);
const PLOT: Flag = Flag::switch("plot", Some('p'), "Update the entire plot");

pub(super) fn register(registry: &mut CommandRegistry) {
    let mut set = Command::new("/set");
    let pattern = set.required("pattern", PatternParser);
    registry.register(set.executes(move |ctx| {
        let pattern = ctx.get(pattern);
        exec_set(ctx, None, &pattern)
    }));

    let mut replace = Command::new("/replace").alias("/re").alias("/rep");
    let mask = replace.optional("mask", MaskParser);
    let pattern = replace.required("pattern", PatternParser);
    registry.register(replace.executes(move |ctx| {
        let mask = ctx.get(mask).unwrap_or(WorldEditMask::Existing);
        let pattern = ctx.get(pattern);
        exec_set(ctx, Some(&mask), &pattern)
    }));

    let mut count = Command::new("/count");
    let mask = count.required("mask", MaskParser);
    registry.register(count.executes(move |ctx| {
        let mask = ctx.get(mask);
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
    }));

    let mut distribution = Command::new("/distr");
    let of_clipboard = distribution.flag(CLIPBOARD);
    registry.register(distribution.executes(move |ctx| {
        let start_time = Instant::now();
        let mut block_counts = FxHashMap::default();

        if ctx.get(of_clipboard) {
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
    }));

    let mut move_selection = Command::new("/move");
    let count = move_selection.optional("count", IntegerParser::new(1, 1000));
    let direction = move_selection.optional("direction", DirectionParser);
    let ignore_air = move_selection.flag(IGNORE_AIR);
    let shift_selection = move_selection.flag(SHIFT_SELECTION);
    registry.register(move_selection.executes(move |ctx| {
        let count = ctx.get(count).unwrap_or(1);
        let direction = ctx.get(direction).unwrap_or(Direction::Me);
        let direction = direction.resolve(ctx.player()?.get_facing());
        let ignore_air = ctx.get(ignore_air);
        let shift_selection = ctx.get(shift_selection);

        let (first_pos, second_pos) = ctx.selection()?;
        let start_time = Instant::now();

        let offset_amount = direction.offset_pos(BlockPos::zero(), count);
        let selected = Region::new(first_pos, second_pos);
        let destination = selected.translated(offset_amount);
        if ctx.clip_region(destination).is_none() {
            return Err(CommandError::destination_outside_plot());
        }

        let clipped = ctx.capture_undo_regions([selected, destination])?;

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
    }));

    let mut update = Command::new("/update");
    let whole_plot = update.flag(PLOT);
    registry.register(update.executes(move |ctx| {
        if ctx.get(whole_plot) {
            let region = ctx.plot().world.region();
            update_region(ctx.world_mut(), region);
            ctx.worldedit_message("Updated entire plot.")
        } else {
            let (first_pos, second_pos) = ctx.selection()?;
            update_region(ctx.world_mut(), Region::new(first_pos, second_pos));
            ctx.worldedit_message("Updated selection.")
        }
    }));

    let mut replace_container = Command::new("/replacecontainer").alias("/rc");
    let from = replace_container.optional("from", ContainerParser);
    let to = replace_container.required("to", ContainerParser);
    registry.register(replace_container.executes(move |ctx| {
        let (from, to) = (ctx.get(from), ctx.get(to));
        exec_replace_container(ctx, from, to)
    }));

    registry.register(Command::new("/wand").executes(|ctx| {
        let item = ItemStack {
            count: 1,
            item_type: Item::WoodenAxe,
            nbt: None,
        };
        let player = ctx.player_mut()?;
        let slot = 36 + player.selected_slot;
        player.set_inventory_slot(slot, Some(item));
        if let Some(entity_equipment) = player.equippment_packet() {
            let entity_equipment = entity_equipment.encode();
            for packet_sender in ctx.plot().world.packet_senders.values() {
                packet_sender.send_packet(&entity_equipment);
            }
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
    ctx.capture_undo_regions([region])?;

    let start_time = Instant::now();
    let mut blocks_updated = 0;

    let world = ctx.world_mut();
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

fn exec_replace_container(
    ctx: &mut ExecutionContext<'_>,
    from: Option<ContainerType>,
    to: ContainerType,
) -> CommandResult<()> {
    let (first_pos, second_pos) = ctx.selection()?;

    let region = Region::new(first_pos, second_pos);
    ctx.capture_undo_regions([region])?;

    let start_time = Instant::now();

    let new_block = to.block(BlockDirection::North, false);

    let world = ctx.world_mut();
    let mut unchanged = 0;
    for pos in Region::new(first_pos, second_pos).positions() {
        let Some(block_container) = world.block(pos).container_type() else {
            continue;
        };

        let (container, strength) = match world.block_entity(pos) {
            Some(BlockEntity::Container { inventory, ty }) => (*ty, ty.signal_strength(inventory)),
            _ => (block_container, UncappedSignalStrength::ZERO),
        };

        if from.is_some_and(|from| from != container) {
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
