use super::selection::update_selection;
use crate::commands::argument::ArgumentType;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult, RuntimeError};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::worldedit::{clear_area, create_clipboard, paste_clipboard};
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::{Block, FlipDirection, RotateAmt};
use mchprs_blocks::region::Region;
use mchprs_blocks::{BlockFacing, BlockPos};
use mchprs_redstone::update_region;
use std::time::Instant;

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(Command::new("/copy").alias("/c").executes(|ctx| {
        let (first_pos, second_pos) = ctx.selection()?;

        let start_time = Instant::now();
        let origin = ctx.player()?.pos.block_pos();
        let clipboard = create_clipboard(ctx.world(), origin, first_pos, second_pos);
        ctx.set_clipboard(clipboard)?;

        ctx.worldedit_message(&format!(
            "Your selection was copied. ({:?})",
            start_time.elapsed()
        ))
    }));

    registry.register(Command::new("/cut").alias("/x").executes(|ctx| {
        let (first_pos, second_pos) = ctx.selection()?;

        let region = Region::new(first_pos, second_pos);
        ctx.capture_undo_regions([region], region.min())?;

        let start_time = Instant::now();
        let origin = ctx.player()?.pos.block_pos();
        let clipboard = create_clipboard(ctx.world(), origin, first_pos, second_pos);
        ctx.set_clipboard(clipboard)?;
        clear_area(ctx.world_mut()?, first_pos, second_pos);

        ctx.worldedit_message(&format!(
            "Your selection was cut. ({:?})",
            start_time.elapsed()
        ))
    }));

    registry.register(
        Command::new("/paste")
            .alias("/v")
            .flag('a', "ignore-air", "Paste without air blocks")
            .flag('u', "update", "Update blocks after pasting")
            .flag('s', "select-region", "Select the pasted region")
            .flag('n', "no-paste", "No paste, select only")
            .executes(exec_paste),
    );
    registry.add_custom_alias("/va", "/paste -a");

    registry.register(
        Command::new("/flip")
            .alias("/f")
            .optional("direction", ArgumentType::Direction)
            .executes(|ctx| {
                let direction = ctx.arg_or("direction", Direction::Me)?;
                let direction = direction.resolve(ctx.player()?.get_facing());

                let start_time = Instant::now();
                let clipboard = ctx.clipboard()?;
                let size = clipboard_size(clipboard);

                let flip_pos = |mut pos: BlockPos| {
                    match direction {
                        BlockFacing::East | BlockFacing::West => pos.x = size.x - 1 - pos.x,
                        BlockFacing::North | BlockFacing::South => pos.z = size.z - 1 - pos.z,
                        BlockFacing::Up | BlockFacing::Down => pos.y = size.y - 1 - pos.y,
                    }
                    pos
                };
                let flip_block = |block: &mut Block| match direction {
                    BlockFacing::East | BlockFacing::West => block.flip(FlipDirection::FlipX),
                    BlockFacing::North | BlockFacing::South => block.flip(FlipDirection::FlipZ),
                    _ => {}
                };

                let flipped = transform_clipboard(clipboard, size, flip_pos, flip_block);
                ctx.set_clipboard(flipped)?;
                ctx.worldedit_message(&format!(
                    "The clipboard copy has been flipped. ({:?})",
                    start_time.elapsed()
                ))
            }),
    );

    registry.register(
        Command::new("/rotate")
            .alias("/r")
            .arg("angle", ArgumentType::integer(-360, 360))
            .executes(|ctx| {
                let angle: i32 = ctx.arg("angle")?;

                let start_time = Instant::now();
                let rotate_amt = match angle % 360 {
                    0 => {
                        return ctx.worldedit_message(
                            "Successfully rotated by 0! That took a lot of work.",
                        );
                    }
                    90 | -270 => RotateAmt::Rotate90,
                    180 | -180 => RotateAmt::Rotate180,
                    270 | -90 => RotateAmt::Rotate270,
                    _ => {
                        return Err(CommandError::runtime(
                            "Rotate amount must be a multiple of 90.",
                        ));
                    }
                };

                let clipboard = ctx.clipboard()?;
                let size = clipboard_size(clipboard);
                let rotated_size = match rotate_amt {
                    RotateAmt::Rotate90 | RotateAmt::Rotate270 => {
                        BlockPos::new(size.z, size.y, size.x)
                    }
                    RotateAmt::Rotate180 => size,
                };

                let rotate_pos = |pos: BlockPos| match rotate_amt {
                    RotateAmt::Rotate90 => BlockPos {
                        x: rotated_size.x - 1 - pos.z,
                        y: pos.y,
                        z: pos.x,
                    },
                    RotateAmt::Rotate180 => BlockPos {
                        x: rotated_size.x - 1 - pos.x,
                        y: pos.y,
                        z: rotated_size.z - 1 - pos.z,
                    },
                    RotateAmt::Rotate270 => BlockPos {
                        x: pos.z,
                        y: pos.y,
                        z: rotated_size.z - 1 - pos.x,
                    },
                };

                let rotated = transform_clipboard(clipboard, rotated_size, rotate_pos, |block| {
                    block.rotate(rotate_amt)
                });
                ctx.set_clipboard(rotated)?;
                ctx.worldedit_message(&format!(
                    "The clipboard copy has been rotated. ({:?})",
                    start_time.elapsed()
                ))
            }),
    );
}

fn exec_paste(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let ignore_air = ctx.flag("ignore-air");
    let should_update = ctx.flag("update");
    let select_region = ctx.flag("select-region");
    let no_paste = ctx.flag("no-paste");

    let start_time = Instant::now();

    let pos = ctx.player()?.pos.block_pos();
    let paste_region = ctx.clipboard()?.region().translated(pos);
    let (paste_min, paste_max) = (paste_region.min(), paste_region.max());
    let select =
        |ctx: &mut ExecutionContext<'_>| update_selection(ctx, None, (paste_min, paste_max));

    if no_paste {
        select(ctx)?.ok_or(RuntimeError::DestinationOutsidePlot)?;
        return ctx.worldedit_message(&format!("Region selected. ({:?})", start_time.elapsed()));
    }

    let clipped = ctx.capture_undo_regions([paste_region], paste_min)?;
    let (world, player) = ctx.world_and_player_mut()?;
    let clipboard = player
        .worldedit_clipboard
        .as_ref()
        .ok_or(RuntimeError::EmptyClipboard)?;
    paste_clipboard(world, clipboard, pos, ignore_air);
    if should_update && let Some(region) = ctx.clip_region(paste_region) {
        update_region(ctx.world_mut()?, region);
    }
    if select_region {
        select(ctx)?;
    }

    let message = if clipped {
        "Your clipboard was pasted (clipped to plot bounds)."
    } else {
        "Your clipboard was pasted."
    };
    ctx.worldedit_message(&format!("{message} ({:?})", start_time.elapsed()))
}

fn clipboard_size(clipboard: &BlockVolume) -> BlockPos {
    let region = clipboard.region();
    region.max() - region.min() + BlockPos::new(1, 1, 1)
}

fn transform_clipboard(
    clipboard: &BlockVolume,
    new_size: BlockPos,
    map_pos: impl Fn(BlockPos) -> BlockPos,
    map_block: impl Fn(&mut Block),
) -> BlockVolume {
    let region = clipboard.region();
    let min = BlockPos::zero() - map_pos(BlockPos::zero() - region.min());
    let mut transformed =
        BlockVolume::empty(Region::new(min, min + new_size - BlockPos::new(1, 1, 1)));

    for pos in region.positions() {
        let mut block = clipboard.block(pos);
        map_block(&mut block);
        transformed.set_block(min + map_pos(pos - region.min()), block);
    }
    for (pos, block_entity) in clipboard.block_entities() {
        transformed.set_block_entity(
            min + map_pos(pos - region.min()),
            Some(block_entity.clone()),
        );
    }
    transformed
}
