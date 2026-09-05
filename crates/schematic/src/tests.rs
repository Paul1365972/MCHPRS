use crate::{read_schematic, write_schematic, WorldEditClipboard};
use mchprs_blocks::block_entities::{BlockEntity, ContainerType, InventoryEntry};
use mchprs_blocks::blocks::{
    Block, Comparator, ComparatorMode, LeverFace, RedstoneWire, RedstoneWireSide, Repeater,
};
use mchprs_blocks::items::Item;
use mchprs_blocks::{BlockDirection, BlockFacing, BlockPos};
use mchprs_world::storage::PalettedBitBuffer;
use rustc_hash::FxHashMap;

const SIZE_X: u32 = 3;
const SIZE_Y: u32 = 2;
const SIZE_Z: u32 = 2;

fn index(x: u32, y: u32, z: u32) -> usize {
    (y * SIZE_Z * SIZE_X + z * SIZE_X + x) as usize
}

fn test_clipboard() -> (WorldEditClipboard, Vec<(u32, u32, u32, Block)>) {
    let blocks = vec![
        (
            0,
            0,
            0,
            Block::Repeater(Repeater::new(3, BlockDirection::West, false, true)),
        ),
        (
            1,
            0,
            0,
            Block::Comparator(Comparator::new(
                BlockDirection::North,
                ComparatorMode::Subtract,
                true,
            )),
        ),
        (
            2,
            0,
            0,
            Block::RedstoneWire(RedstoneWire::new(
                RedstoneWireSide::Side,
                RedstoneWireSide::None,
                RedstoneWireSide::Up,
                RedstoneWireSide::Side,
                9,
            )),
        ),
        (
            0,
            1,
            1,
            Block::Barrel {
                facing: BlockFacing::Up,
                open: false,
            },
        ),
        (
            2,
            1,
            0,
            Block::Lever {
                face: LeverFace::Floor,
                facing: BlockDirection::South,
                powered: true,
            },
        ),
    ];

    let mut data = PalettedBitBuffer::new((SIZE_X * SIZE_Y * SIZE_Z) as usize, 9);
    for (x, y, z, block) in &blocks {
        data.set_entry(index(*x, *y, *z), block.get_id());
    }

    let mut block_entities = FxHashMap::default();
    block_entities.insert(
        BlockPos::new(1, 0, 0),
        BlockEntity::Comparator { output_strength: 5 },
    );
    block_entities.insert(
        BlockPos::new(0, 1, 1),
        BlockEntity::Container {
            comparator_override: 1,
            inventory: vec![InventoryEntry {
                id: Item::Redstone.get_id(),
                slot: 0,
                count: 64,
                nbt: None,
            }],
            ty: ContainerType::Barrel,
        },
    );

    let clipboard = WorldEditClipboard {
        offset_x: -4,
        offset_y: 7,
        offset_z: 13,
        size_x: SIZE_X,
        size_y: SIZE_Y,
        size_z: SIZE_Z,
        data,
        block_entities,
    };
    (clipboard, blocks)
}

#[test]
fn sponge_round_trip() {
    let (clipboard, blocks) = test_clipboard();

    let mut bytes = Vec::new();
    write_schematic(&mut bytes, &clipboard).unwrap();
    let read = read_schematic(bytes.as_slice()).unwrap();

    assert_eq!(read.size_x, clipboard.size_x);
    assert_eq!(read.size_y, clipboard.size_y);
    assert_eq!(read.size_z, clipboard.size_z);
    assert_eq!(read.offset_x, clipboard.offset_x);
    assert_eq!(read.offset_y, clipboard.offset_y);
    assert_eq!(read.offset_z, clipboard.offset_z);

    let mut expected_blocks = vec![Block::Air; (SIZE_X * SIZE_Y * SIZE_Z) as usize];
    for (x, y, z, block) in blocks {
        expected_blocks[index(x, y, z)] = block;
    }
    for (i, expected) in expected_blocks.into_iter().enumerate() {
        assert_eq!(Block::from_id(read.data.get_entry(i)), expected);
    }

    assert_eq!(read.block_entities, clipboard.block_entities);
}
