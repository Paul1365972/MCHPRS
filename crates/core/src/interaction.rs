use crate::item_stack::ItemStack;
use crate::player::Player;
use crate::plot::{PlotWorld, PLOT_BLOCK_HEIGHT};
use fastnbt::Value;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::items::Item;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::{blocks::*, BlockDirection, BlockFacing};
use mchprs_blocks::{BlockFace, BlockPos};
use mchprs_network::packets::clientbound::{COpenSignEditor, ClientBoundPacket};
use mchprs_network::NBTCompound;
use mchprs_redstone::{self as redstone, Notify, RedstoneWorld};

pub fn on_use(
    block: Block,
    world: &mut impl RedstoneWorld,
    player: &mut Player,
    pos: BlockPos,
    item_in_hand: Option<Item>,
) -> ActionResult {
    if redstone::use_block(world, pos) {
        return ActionResult::Success;
    }

    match block {
        Block::SeaPickle {
            pickles,
            waterlogged,
        } => {
            if let Some(Item::SeaPickle) = item_in_hand
                && pickles < 4
            {
                let grown = Block::SeaPickle {
                    pickles: pickles + 1,
                    waterlogged,
                };
                redstone::set_block(world, pos, grown, Notify::All);
            }
            ActionResult::Success
        }
        Block::EndPortalFrame { eye, facing } => {
            if let Some(Item::EnderEye) = item_in_hand
                && !eye
            {
                let filled = Block::EndPortalFrame { eye: true, facing };
                redstone::set_block(world, pos, filled, Notify::Shapes);
                redstone::update_neighbors_for_output_signal(world, pos);
                return ActionResult::Success;
            }
            ActionResult::Pass
        }
        b if b.has_block_entity() => {
            // Open container
            let block_entity = world.block_entity(pos);
            if let Some(BlockEntity::Container { inventory, ty, .. }) = block_entity {
                player.open_container(inventory, *ty);
            }
            ActionResult::Success
        }
        _ => ActionResult::Pass,
    }
}

fn get_sign_placement(
    context: &UseOnBlockContext<'_>,
    standard: fn(u8) -> Block,
    wall: fn(BlockDirection) -> Block,
) -> Block {
    let rotation = (((180.0 + context.player.yaw) * 16.0 / 360.0) + 0.5).floor() as u8 & 15;

    match context.block_face {
        BlockFace::Bottom => Block::Air,
        BlockFace::Top => standard(rotation),
        _ => wall(context.block_face.unwrap_direction()),
    }
}

fn get_button_placement(context: &UseOnBlockContext<'_>) -> Button {
    let face = match context.block_face {
        BlockFace::Top => LeverFace::Floor,
        BlockFace::Bottom => LeverFace::Ceiling,
        _ => LeverFace::Wall,
    };
    let facing = if face == LeverFace::Wall {
        context.block_face.unwrap_direction()
    } else {
        context.player.get_direction()
    };
    Button::new(face, facing, false)
}

pub fn get_state_for_placement(
    world: &impl BlockView,
    pos: BlockPos,
    item: Item,
    context: &UseOnBlockContext<'_>,
) -> Block {
    let simple_block = item.get_simple_placement();
    macro_rules! sign_placement {
        ($standard_block:ident, $wall_block:ident) => {
            get_sign_placement(
                context,
                |rotation| {
                    Block::$standard_block(StandingSign {
                        rotation,
                        waterlogged: false,
                    })
                },
                |facing| Block::$wall_block {
                    waterlogged: false,
                    facing,
                },
            )
        };
    }
    let block = match item {
        Item::OakSign => sign_placement!(OakSign, OakWallSign),
        Item::SpruceSign => sign_placement!(SpruceSign, SpruceWallSign),
        Item::BirchSign => sign_placement!(BirchSign, BirchWallSign),
        Item::AcaciaSign => sign_placement!(AcaciaSign, AcaciaWallSign),
        Item::JungleSign => sign_placement!(JungleSign, JungleWallSign),
        Item::DarkOakSign => sign_placement!(DarkOakSign, DarkOakWallSign),
        Item::CrimsonSign => sign_placement!(CrimsonSign, CrimsonWallSign),
        Item::WarpedSign => sign_placement!(WarpedSign, WarpedWallSign),
        Item::BambooSign => sign_placement!(BambooSign, BambooWallSign),
        Item::CherrySign => sign_placement!(CherrySign, CherryWallSign),
        Item::MangroveSign => sign_placement!(MangroveSign, MangroveWallSign),
        Item::SeaPickle => Block::SeaPickle {
            pickles: 1,
            waterlogged: false,
        },
        Item::Furnace => Block::Furnace {
            facing: context.player.get_direction().opposite(),
            lit: false,
        },
        Item::OakPressurePlate => Block::OakPressurePlate { powered: false },
        Item::SprucePressurePlate => Block::SprucePressurePlate { powered: false },
        Item::BirchPressurePlate => Block::BirchPressurePlate { powered: false },
        Item::JunglePressurePlate => Block::JunglePressurePlate { powered: false },
        Item::AcaciaPressurePlate => Block::AcaciaPressurePlate { powered: false },
        Item::DarkOakPressurePlate => Block::DarkOakPressurePlate { powered: false },
        Item::MangrovePressurePlate => Block::MangrovePressurePlate { powered: false },
        Item::CherryPressurePlate => Block::CherryPressurePlate { powered: false },
        Item::BambooPressurePlate => Block::BambooPressurePlate { powered: false },
        Item::CrimsonPressurePlate => Block::CrimsonPressurePlate { powered: false },
        Item::WarpedPressurePlate => Block::WarpedPressurePlate { powered: false },
        Item::StonePressurePlate => Block::StonePressurePlate { powered: false },
        Item::PolishedBlackstonePressurePlate => {
            Block::PolishedBlackstonePressurePlate { powered: false }
        }
        Item::Lever => {
            let face = match context.block_face {
                BlockFace::Top => LeverFace::Floor,
                BlockFace::Bottom => LeverFace::Ceiling,
                _ => LeverFace::Wall,
            };
            let facing = if face == LeverFace::Wall {
                context.block_face.unwrap_direction()
            } else {
                context.player.get_direction()
            };
            Block::Lever {
                face,
                facing,
                powered: false,
            }
        }
        Item::RedstoneTorch => match context.block_face {
            BlockFace::Top | BlockFace::Bottom => Block::RedstoneTorch { lit: true },
            face => Block::RedstoneWallTorch {
                lit: true,
                facing: face.unwrap_direction(),
            },
        },
        Item::TripwireHook => match context.block_face {
            BlockFace::Bottom | BlockFace::Top => Block::Air,
            direction => Block::TripwireHook {
                facing: direction.unwrap_direction(),
                powered: false,
                attached: false,
            },
        },
        Item::StoneButton => Block::StoneButton(get_button_placement(context)),
        Item::PolishedBlackstoneButton => {
            Block::PolishedBlackstoneButton(get_button_placement(context))
        }
        Item::OakButton => Block::OakButton(get_button_placement(context)),
        Item::SpruceButton => Block::SpruceButton(get_button_placement(context)),
        Item::BirchButton => Block::BirchButton(get_button_placement(context)),
        Item::JungleButton => Block::JungleButton(get_button_placement(context)),
        Item::AcaciaButton => Block::AcaciaButton(get_button_placement(context)),
        Item::DarkOakButton => Block::DarkOakButton(get_button_placement(context)),
        Item::MangroveButton => Block::MangroveButton(get_button_placement(context)),
        Item::CherryButton => Block::CherryButton(get_button_placement(context)),
        Item::BambooButton => Block::BambooButton(get_button_placement(context)),
        Item::CrimsonButton => Block::CrimsonButton(get_button_placement(context)),
        Item::WarpedButton => Block::WarpedButton(get_button_placement(context)),
        Item::RedstoneLamp => Block::RedstoneLamp { lit: false },
        // TODO: Hopper facing
        Item::Hopper => Block::Hopper {
            enabled: false,
            facing: HopperFacing::Down,
        },
        Item::Repeater => Block::Repeater(Repeater::new(
            1,
            context.player.get_direction().opposite(),
            false,
            false,
        )),
        Item::Comparator => Block::Comparator(Comparator::new(
            context.player.get_direction().opposite(),
            ComparatorMode::Compare,
            false,
        )),
        Item::Redstone => Block::RedstoneWire(RedstoneWire::default()),
        // TODO: Barrel facing
        Item::Barrel => Block::Barrel {
            facing: BlockFacing::Up,
            open: false,
        },
        Item::Target => Block::Target {
            power: SignalStrength::Zero,
        },
        Item::SmoothStoneSlab => Block::SmoothStoneSlab {
            ty: SlabType::Top,
            waterlogged: false,
        },
        Item::QuartzSlab => Block::QuartzSlab {
            ty: SlabType::Top,
            waterlogged: false,
        },
        Item::IronTrapdoor => match context.block_face {
            BlockFace::Bottom => Block::IronTrapdoor {
                facing: context.player.get_direction().opposite(),
                half: TrapdoorHalf::Top,
                powered: false,
                open: false,
                waterlogged: false,
            },
            BlockFace::Top => Block::IronTrapdoor {
                facing: context.player.get_direction().opposite(),
                half: TrapdoorHalf::Bottom,
                open: false,
                waterlogged: false,
                powered: false,
            },
            _ => Block::IronTrapdoor {
                facing: context.block_face.unwrap_direction(),
                half: if context.cursor_y > 0.5 {
                    TrapdoorHalf::Top
                } else {
                    TrapdoorHalf::Bottom
                },
                open: false,
                waterlogged: false,
                powered: false,
            },
        },
        Item::NoteBlock => Block::NoteBlock {
            instrument: Instrument::Harp,
            note: 0,
            powered: false,
        },
        Item::BoneBlock => Block::BoneBlock { axis: BlockAxis::Y },
        Item::HayBlock => Block::HayBlock { axis: BlockAxis::Y },
        Item::EndPortalFrame => Block::EndPortalFrame {
            eye: false,
            facing: context.player.get_direction().opposite(),
        },
        _ => Block::Air,
    };
    let block = redstone::state_for_placement(world, pos, simple_block.unwrap_or(block));
    if redstone::can_survive(world, pos, block) {
        block
    } else {
        Block::Air
    }
}

fn read_block_entity_tag(nbt: &NBTCompound, block_id: &str) -> Option<BlockEntity> {
    let Some(Value::Compound(compound)) = nbt.get("BlockEntityTag") else {
        return None;
    };
    let id = match compound.get("Id").or_else(|| compound.get("id")) {
        Some(Value::String(id)) => id,
        Some(_) => return None,
        None => block_id,
    };
    BlockEntity::from_nbt(id, compound)
}

pub fn place_in_world(
    block: Block,
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    nbt: &Option<NBTCompound>,
) {
    redstone::place_block(world, pos, block);
    if block.has_block_entity()
        && let Some(nbt) = nbt
        && let Some(block_entity) = read_block_entity_tag(nbt, block.get_name())
    {
        world.set_block_entity(pos, Some(block_entity));
        redstone::update_neighbors_for_output_signal(world, pos);
    };
}

#[derive(PartialEq, Eq, Copy, Clone)]
pub enum ActionResult {
    Success,
    Pass,
}

impl ActionResult {
    fn is_success(self) -> bool {
        self == ActionResult::Success
    }
}

pub struct UseOnBlockContext<'a> {
    pub block_pos: BlockPos,
    pub block_face: BlockFace,
    pub player: &'a mut Player,
    pub cursor_y: f32,
    pub block_in_hitbox: bool,
}

/// returns true if cancelled
pub fn use_item_on_block(
    item: &ItemStack,
    world: &mut PlotWorld,
    ctx: UseOnBlockContext<'_>,
) -> bool {
    let use_pos = ctx.block_pos;
    let use_block = world.block(use_pos);
    let block_pos = ctx.block_pos.offset(ctx.block_face);
    let mut top_pos = ctx.player.pos.block_pos();
    top_pos.y += 1;
    if (block_pos == ctx.player.pos.block_pos() || block_pos == top_pos) && !ctx.block_in_hitbox {
        return false;
    }
    let can_place = item.item_type.is_block() && world.block(block_pos).can_place_block_in();

    if !ctx.player.crouching
        && on_use(
            use_block,
            world,
            ctx.player,
            ctx.block_pos,
            Some(item.item_type),
        )
        .is_success()
    {
        return false;
    }

    if can_place && (0..PLOT_BLOCK_HEIGHT).contains(&block_pos.y) {
        let block = get_state_for_placement(world, block_pos, item.item_type, &ctx);

        if (block.is_sign() || block.is_wall_sign())
            && !item
                .nbt
                .as_ref()
                .is_some_and(|nbt| nbt.contains_key("BlockEntityTag"))
        {
            let open_sign_editor = COpenSignEditor {
                pos_x: block_pos.x,
                pos_y: block_pos.y,
                pos_z: block_pos.z,
                // TODO: editing back text
                is_front_text: true,
            }
            .encode();
            ctx.player.client.send_packet(&open_sign_editor);
        }

        place_in_world(block, world, block_pos, &item.nbt);
        false
    } else {
        true
    }
}
