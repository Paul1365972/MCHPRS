use super::selection::update_selection;
use crate::commands::argument_parser::{
    ContainerParser, DirectionParser, IntegerParser, PatternParser,
};
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::item_stack::ItemStack;
use crate::worldedit::pattern::WorldEditPattern;
use crate::worldedit::{clear_area, region_positions, update as update_region};
use mchprs_blocks::block_entities::{BlockEntity, ContainerType, InventoryEntry};
use mchprs_blocks::blocks::{Block, HopperFacing};
use mchprs_blocks::items::Item;
use mchprs_blocks::{BlockDirection, BlockFacing, BlockPos};
use mchprs_network::packets::clientbound::ClientBoundPacket;
use mchprs_schematic::{create_clipboard, paste_clipboard};
use mchprs_syntax::flag::Flag;
use mchprs_world::World;
use std::time::Instant;

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

    let mut replace = Command::new("/replace");
    let mask = replace.required("from", PatternParser);
    let pattern = replace.required("to", PatternParser);
    registry.register(replace.executes(move |ctx| {
        let mask = ctx.get(mask);
        let pattern = ctx.get(pattern);
        exec_set(ctx, Some(&mask), &pattern)
    }));

    let mut count = Command::new("/count");
    let mask = count.required("mask", PatternParser);
    registry.register(count.executes(move |ctx| {
        let mask = ctx.get(mask);
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
        let destination = (
            first_pos.min(second_pos) + offset_amount,
            first_pos.max(second_pos) + offset_amount,
        );
        if ctx.clip_region(destination.0, destination.1).is_none() {
            return Err(CommandError::destination_outside_plot());
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
    }));

    let mut update = Command::new("/update");
    let whole_plot = update.flag(PLOT);
    registry.register(update.executes(move |ctx| {
        if ctx.get(whole_plot) {
            let corners = ctx.plot().world.get_corners();
            update_region(ctx.world_mut(), corners.0, corners.1);
            ctx.worldedit_message("Updated entire plot.")
        } else {
            let (first_pos, second_pos) = ctx.selection()?;
            update_region(ctx.world_mut(), first_pos, second_pos);
            ctx.worldedit_message("Updated selection.")
        }
    }));

    let mut replace_container = Command::new("/replacecontainer").alias("/rc");
    let from = replace_container.required("from", ContainerParser);
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
    mask: Option<&WorldEditPattern>,
    pattern: &WorldEditPattern,
) -> CommandResult<()> {
    let (first_pos, second_pos) = ctx.selection()?;

    let origin = first_pos.min(second_pos);
    ctx.capture_undo_regions([(first_pos, second_pos)], origin)?;

    let start_time = Instant::now();
    let mut blocks_updated = 0;

    let world = ctx.world_mut();
    for pos in region_positions(first_pos, second_pos) {
        if mask.is_none_or(|mask| mask.matches(world.get_block(pos)))
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

fn exec_replace_container(
    ctx: &mut ExecutionContext<'_>,
    from: ContainerType,
    to: ContainerType,
) -> CommandResult<()> {
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
        let block_container = match world.get_block(pos) {
            Block::Furnace { .. } => ContainerType::Furnace,
            Block::Barrel { .. } => ContainerType::Barrel,
            Block::Hopper { .. } => ContainerType::Hopper,
            _ => continue,
        };

        let (container, strength) = match world.get_block_entity(pos) {
            Some(BlockEntity::Container {
                comparator_override,
                ty,
                ..
            }) => (*ty, *comparator_override),
            _ => (block_container, 0),
        };

        if from != container {
            continue;
        }

        let items_needed = to.items_needed_for_signal_strength(strength);
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
            comparator_override: strength,
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
