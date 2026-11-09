use crate::graph::{Graph, Link, LinkKind, NodeIdx};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use mchprs_redstone::diode::is_diode;
use mchprs_redstone::{comparator, emitters, provides_weak_power, wire, Dust};
use rustc_hash::FxHashMap;
use std::cell::Cell;

const DUST_RANGE: u8 = 15;

pub fn add_links(
    graph: &mut Graph,
    world: &impl BlockView,
    nodes_by_position: &FxHashMap<BlockPos, NodeIdx>,
) {
    let blocks = BlockCache::new(world);
    let mut search = LinkSearch {
        blocks: &blocks,
        graph,
        nodes_by_position,
    };
    search.search();
}

type CachedBlock = Cell<Option<(Block, BlockPos)>>;

struct BlockCache<'world, W: BlockView> {
    world: &'world W,
    cache: [[[CachedBlock; 16]; 16]; 16],
}

impl<'world, W: BlockView> BlockCache<'world, W> {
    fn new(world: &'world W) -> Self {
        Self {
            world,
            cache: Default::default(),
        }
    }
}

impl<W: BlockView> BlockView for BlockCache<'_, W> {
    fn block(&self, pos: BlockPos) -> Block {
        let entry = &self.cache[pos.x as usize % 16][pos.y as usize % 16][pos.z as usize % 16];
        match entry.get() {
            Some((block, cached_pos)) if cached_pos == pos => block,
            _ => {
                let block = self.world.block(pos);
                entry.set(Some((block, pos)));
                block
            }
        }
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        self.world.block_entity(pos)
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        self.world.occupied_regions(region)
    }
}

struct LinkSearch<'a, W: BlockView> {
    blocks: &'a BlockCache<'a, W>,
    graph: &'a mut Graph,
    nodes_by_position: &'a FxHashMap<BlockPos, NodeIdx>,
}

impl<W: BlockView> LinkSearch<'_, W> {
    fn add_links_from(
        &mut self,
        pos: BlockPos,
        direction: BlockFace,
        link_kind: LinkKind,
        distance: u8,
        start_node: NodeIdx,
        dust: Dust,
    ) {
        for emitter in emitters(self.blocks, pos, direction, dust) {
            if is_wire(emitter.state) {
                self.search_wire(start_node, emitter.pos, link_kind, distance);
            } else {
                self.add_link(emitter.pos, start_node, Link::new(link_kind, distance));
            }
        }
    }

    fn search_wire(
        &mut self,
        start_node: NodeIdx,
        root_pos: BlockPos,
        link_kind: LinkKind,
        initial_distance: u8,
    ) {
        let mut discovered = vec![(root_pos, initial_distance)];

        // Faster than a set for the few positions a search usually discovers.
        let has_been_discovered =
            |discovered: &[_], new_pos| discovered.iter().any(|(pos, _)| *pos == new_pos);

        let mut idx = 0;
        while idx < discovered.len() {
            let (pos, distance) = discovered[idx];
            idx += 1;

            if distance >= DUST_RANGE {
                continue;
            }

            for side in BlockFace::values() {
                let neighbor_pos = pos.offset(side);
                self.add_links_from(
                    neighbor_pos,
                    side,
                    link_kind,
                    distance,
                    start_node,
                    Dust::Muted,
                );
            }
            for wire_pos in wire::linked_wires(self.blocks, pos) {
                if !has_been_discovered(&discovered, wire_pos) {
                    discovered.push((wire_pos, distance + 1));
                }
            }
        }
    }

    fn add_link(&mut self, source: BlockPos, target: NodeIdx, link: Link) {
        if let Some(&source) = self.nodes_by_position.get(&source) {
            self.graph.add_link(source, target, link);
        }
    }

    fn search_diode_inputs(&mut self, id: NodeIdx, pos: BlockPos, facing: BlockDirection) {
        let input_pos = pos.offset(facing.block_face());
        self.add_links_from(
            input_pos,
            facing.block_face(),
            LinkKind::Default,
            0,
            id,
            Dust::Emits,
        )
    }

    fn search_repeater_side(&mut self, id: NodeIdx, pos: BlockPos, side: BlockDirection) {
        let side_pos = pos.offset(side.block_face());
        let side_block = self.blocks.block(side_pos);
        if is_diode(side_block) && provides_weak_power(side_block, side.block_face()) {
            self.add_link(side_pos, id, Link::new(LinkKind::Side, 0));
        }
    }

    fn search_comparator_side(&mut self, id: NodeIdx, pos: BlockPos, side: BlockDirection) {
        let side_pos = pos.offset(side.block_face());
        let side_block = self.blocks.block(side_pos);
        if (is_diode(side_block) && provides_weak_power(side_block, side.block_face()))
            || matches!(side_block, Block::RedstoneBlock)
        {
            self.add_link(side_pos, id, Link::new(LinkKind::Side, 0));
        } else if matches!(side_block, Block::RedstoneWire { .. }) {
            self.search_wire(id, side_pos, LinkKind::Side, 0)
        }
    }

    fn search_node(&mut self, id: NodeIdx, (pos, block_id): (BlockPos, u32)) {
        match Block::from_id(block_id).expect("extracted nodes hold known block states") {
            Block::RedstoneTorch { .. } => {
                let bottom_pos = pos.offset(BlockFace::Bottom);
                self.add_links_from(
                    bottom_pos,
                    BlockFace::Top,
                    LinkKind::Default,
                    0,
                    id,
                    Dust::Emits,
                );
            }
            Block::RedstoneWallTorch { facing, .. } => {
                let wall_pos = pos.offset(facing.opposite().block_face());
                self.add_links_from(
                    wall_pos,
                    facing.opposite().block_face(),
                    LinkKind::Default,
                    0,
                    id,
                    Dust::Emits,
                );
            }
            Block::Comparator(comparator) => {
                let facing = comparator.facing;

                self.search_comparator_side(id, pos, facing.rotate());
                self.search_comparator_side(id, pos, facing.rotate_ccw());

                let input_pos = pos.offset(facing.block_face());
                let input_block = self.blocks.block(input_pos);
                if comparator::analog_output(self.blocks, input_pos, input_block).is_some() {
                    self.add_link(input_pos, id, Link::new(LinkKind::Default, 0));
                } else {
                    self.search_diode_inputs(id, pos, facing);
                }
            }
            Block::Repeater(repeater) => {
                let facing = repeater.facing;

                self.search_diode_inputs(id, pos, facing);
                self.search_repeater_side(id, pos, facing.rotate());
                self.search_repeater_side(id, pos, facing.rotate_ccw());
            }
            Block::RedstoneWire { .. } => {
                self.search_wire(id, pos, LinkKind::Default, 0);
            }
            Block::RedstoneLamp { .. } | Block::IronTrapdoor { .. } | Block::NoteBlock { .. } => {
                for face in &BlockFace::values() {
                    let neighbor_pos = pos.offset(*face);
                    self.add_links_from(neighbor_pos, *face, LinkKind::Default, 0, id, Dust::Emits);
                }
            }
            _ => {}
        }
    }

    fn search(&mut self) {
        let nodes: Vec<NodeIdx> = self.graph.node_indices().collect();
        for idx in nodes {
            if self.graph[idx].is_input() {
                continue;
            }
            if let Some(block) = self.graph[idx].blocks.first().copied() {
                self.search_node(idx, block);
            }
        }
    }
}

fn is_wire(block: Block) -> bool {
    matches!(block, Block::RedstoneWire { .. })
}
