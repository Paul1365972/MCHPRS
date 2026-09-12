use super::selection::update_selection;
use crate::commands::argument_parser::{DirectionParser, IntegerParser};
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::worldedit::{clear_area, update};
use mchprs_blocks::blocks::{Block, FlipDirection, RotateAmt};
use mchprs_blocks::{BlockFacing, BlockPos};
use mchprs_schematic::{create_clipboard, paste_clipboard, WorldEditClipboard};
use mchprs_syntax::flag::Flag;
use mchprs_world::storage::PalettedBitBuffer;
use std::time::Instant;

const IGNORE_AIR: Flag = Flag::switch("ignore-air", Some('a'), "Paste without air blocks");
const UPDATE: Flag = Flag::switch("update", Some('u'), "Update blocks after pasting");
const SELECT_REGION: Flag = Flag::switch("select-region", Some('s'), "Select the pasted region");

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

        let origin = first_pos.min(second_pos);
        ctx.capture_undo_regions([(first_pos, second_pos)], origin)?;

        let start_time = Instant::now();
        let origin = ctx.player()?.pos.block_pos();
        let clipboard = create_clipboard(ctx.world(), origin, first_pos, second_pos);
        ctx.set_clipboard(clipboard)?;
        clear_area(ctx.world_mut(), first_pos, second_pos);

        ctx.worldedit_message(&format!(
            "Your selection was cut. ({:?})",
            start_time.elapsed()
        ))
    }));

    let mut paste = Command::new("/paste").alias("/v");
    let ignore_air = paste.flag(IGNORE_AIR);
    let update = paste.flag(UPDATE);
    let select_region = paste.flag(SELECT_REGION);
    registry.register(paste.executes(move |ctx| {
        let pasting = Pasting {
            ignore_air: ctx.get(ignore_air),
            update: ctx.get(update),
            select_region: ctx.get(select_region),
        };
        exec_paste(ctx, pasting)
    }));
    registry.alias("/va", "/paste -a");

    let mut flip = Command::new("/flip").alias("/f");
    let direction = flip.optional("direction", DirectionParser);
    registry.register(flip.executes(move |ctx| {
        let direction = ctx.get(direction).unwrap_or(Direction::Me);
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
    }));

    let mut rotate = Command::new("/rotate").alias("/r");
    let angle = rotate.required("angle", IntegerParser::new(i32::MIN, i32::MAX));
    registry.register(rotate.executes(move |ctx| {
        let angle = ctx.get(angle);

        let start_time = Instant::now();
        let rotate_amt = match angle % 360 {
            0 => {
                return ctx
                    .worldedit_message("Successfully rotated by 0! That took a lot of work.");
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
            RotateAmt::Rotate90 | RotateAmt::Rotate270 => BlockPos::new(size.z, size.y, size.x),
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
    }));
}

struct Pasting {
    ignore_air: bool,
    update: bool,
    select_region: bool,
}

fn exec_paste(ctx: &mut ExecutionContext<'_>, pasting: Pasting) -> CommandResult<()> {
    let Pasting {
        ignore_air,
        update: should_update,
        select_region,
    } = pasting;
    let start_time = Instant::now();

    let pos = ctx.player()?.pos.block_pos();
    let clipboard = ctx.clipboard()?;
    let paste_min = pos - BlockPos::new(clipboard.offset_x, clipboard.offset_y, clipboard.offset_z);
    let paste_max = paste_min
        + BlockPos::new(
            clipboard.size_x as i32 - 1,
            clipboard.size_y as i32 - 1,
            clipboard.size_z as i32 - 1,
        );
    let select =
        |ctx: &mut ExecutionContext<'_>| update_selection(ctx, None, (paste_min, paste_max));

    let clipped = ctx.capture_undo_regions([(paste_min, paste_max)], paste_min)?;
    let (world, player) = ctx.world_and_player_mut()?;
    let clipboard = player
        .worldedit_clipboard
        .as_ref()
        .ok_or_else(CommandError::empty_clipboard)?;
    paste_clipboard(world, clipboard, pos, ignore_air);
    if should_update && let Some((region_min, region_max)) = ctx.clip_region(paste_min, paste_max) {
        update(ctx.world_mut(), region_min, region_max);
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

fn clipboard_size(clipboard: &WorldEditClipboard) -> BlockPos {
    BlockPos::new(
        clipboard.size_x as i32,
        clipboard.size_y as i32,
        clipboard.size_z as i32,
    )
}

fn transform_clipboard(
    clipboard: &WorldEditClipboard,
    new_size: BlockPos,
    map_pos: impl Fn(BlockPos) -> BlockPos,
    map_block: impl Fn(&mut Block),
) -> WorldEditClipboard {
    let (size_x, size_y, size_z) = (clipboard.size_x, clipboard.size_y, clipboard.size_z);
    let (n_size_x, n_size_y, n_size_z) = (new_size.x as u32, new_size.y as u32, new_size.z as u32);
    let volume = size_x * size_y * size_z;
    let mut data = PalettedBitBuffer::new(volume as usize, 9);

    for y in 0..size_y {
        for z in 0..size_z {
            for x in 0..size_x {
                let i = y * size_x * size_z + z * size_x + x;
                let n = map_pos(BlockPos::new(x as i32, y as i32, z as i32));
                let n_i = n.y as u32 * n_size_x * n_size_z + n.z as u32 * n_size_x + n.x as u32;

                let mut block = Block::from_id(clipboard.data.get_entry(i as usize));
                map_block(&mut block);
                data.set_entry(n_i as usize, block.get_id());
            }
        }
    }

    let offset = map_pos(BlockPos::new(
        clipboard.offset_x,
        clipboard.offset_y,
        clipboard.offset_z,
    ));
    WorldEditClipboard {
        offset_x: offset.x,
        offset_y: offset.y,
        offset_z: offset.z,
        size_x: n_size_x,
        size_y: n_size_y,
        size_z: n_size_z,
        data,
        block_entities: clipboard
            .block_entities
            .iter()
            .map(|(pos, e)| (map_pos(*pos), e.clone()))
            .collect(),
    }
}
