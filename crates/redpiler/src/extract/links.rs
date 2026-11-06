use crate::graph::{Direction, Graph, Link, LinkKind, NodeIdx, NodeKind};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use mchprs_redstone::{self, comparator, provides_strong_power, provides_weak_power, wire};
use rustc_hash::FxHashMap;

const DUST_RANGE: u8 = 15;

pub fn add_links(
    graph: &mut Graph,
    world: &impl BlockView,
    nodes_by_position: &FxHashMap<BlockPos, NodeIdx>,
) {
    let mut search = LinkSearch {
        world,
        graph,
        nodes_by_position,
        block_cache: BlockCache::new(world),
    };
    search.search();
    for node in (0..graph.node_bound()).map(NodeIdx::new) {
        if graph.contains_node(node) {
            graph.sort_edges_by_key(node, Direction::Outgoing, |graph, edge| {
                (type_order(&graph[edge.target()].kind), edge.target())
            });
        }
    }
}

fn type_order(kind: &NodeKind) -> u8 {
    match kind {
        NodeKind::Repeater { .. } => 0,
        NodeKind::Torch => 1,
        NodeKind::Comparator { .. } => 2,
        NodeKind::Lamp => 3,
        NodeKind::Button { .. } => 4,
        NodeKind::Lever => 5,
        NodeKind::PressurePlate => 6,
        NodeKind::Trapdoor => 7,
        NodeKind::Wire => 8,
        NodeKind::Constant => 9,
        NodeKind::NoteBlock { .. } => 10,
    }
}

/// Querying blocks from the world may be expensive, so we save recently
/// queried blocks in a cache to reduce the number of times we query the world.
struct BlockCache<'world, W: BlockView> {
    world: &'world W,
    // It's not really that complex
    #[allow(clippy::type_complexity)]
    cache: [[[Option<(Block, BlockPos)>; 16]; 16]; 16],
}

impl<'world, W: BlockView> BlockCache<'world, W> {
    fn new(world: &'world W) -> Self {
        Self {
            world,
            cache: Default::default(),
        }
    }

    fn block(&mut self, pos: BlockPos) -> Block {
        let entry = &mut self.cache[pos.x as usize % 16][pos.y as usize % 16][pos.z as usize % 16];
        match entry {
            Some((block, cached_pos)) if *cached_pos == pos => *block,
            _ => {
                let block = self.world.block(pos);
                *entry = Some((block, pos));
                block
            }
        }
    }
}

struct LinkSearch<'a, W: BlockView> {
    world: &'a W,
    graph: &'a mut Graph,
    nodes_by_position: &'a FxHashMap<BlockPos, NodeIdx>,
    block_cache: BlockCache<'a, W>,
}

impl<W: BlockView> LinkSearch<'_, W> {
    fn add_link(&mut self, source: BlockPos, target: NodeIdx, link: Link) {
        if let Some(&source) = self.nodes_by_position.get(&source) {
            self.graph.add_edge(source, target, link);
        }
    }

    // unfortunate
    #[allow(clippy::too_many_arguments)]
    fn get_redstone_links(
        &mut self,
        block: Block,
        side: BlockFace,
        pos: BlockPos,
        link_kind: LinkKind,
        distance: u8,
        start_node: NodeIdx,
        search_wire: bool,
    ) {
        if block.is_conductor() {
            for side in &BlockFace::values() {
                let pos = pos.offset(*side);
                let block = self.block_cache.block(pos);
                if provides_strong_power(block, *side) {
                    self.add_link(pos, start_node, Link::new(link_kind, distance));
                }

                if let Block::RedstoneWire(wire) = block {
                    if !search_wire {
                        continue;
                    }
                    match side {
                        BlockFace::Top => {
                            self.search_wire(start_node, pos, link_kind, distance);
                        }
                        BlockFace::Bottom => {}
                        _ => {
                            let direction = side.unwrap_direction();
                            if !wire::get_current_side(
                                wire::get_regulated_sides(wire, self.world, pos),
                                direction.opposite(),
                            )
                            .is_none()
                            {
                                self.search_wire(start_node, pos, link_kind, distance);
                            }
                        }
                    }
                }
            }
        } else if provides_weak_power(block, side) {
            self.add_link(pos, start_node, Link::new(link_kind, distance));
        } else if let Block::RedstoneWire(wire) = block {
            match side {
                BlockFace::Top => self.search_wire(start_node, pos, link_kind, distance),
                BlockFace::Bottom => {}
                _ => {
                    let direction = side.unwrap_direction();
                    if search_wire
                        && !wire::get_current_side(
                            wire::get_regulated_sides(wire, self.world, pos),
                            direction.opposite(),
                        )
                        .is_none()
                    {
                        self.search_wire(start_node, pos, link_kind, distance);
                    }
                }
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

        // Linear search has better runtime performance than any other lookup when there are
        // only a few elements, which is most of the time.
        let has_been_discovered =
            |discovered: &[_], new_pos| discovered.iter().any(|(pos, _)| *pos == new_pos);

        let mut idx = 0;
        while idx < discovered.len() {
            let (pos, distance) = discovered[idx];
            idx += 1;

            // We can stop looking once we've reached the max ss of a wire.
            // This also prevents overflowing the distance past 255.
            if distance >= DUST_RANGE {
                continue;
            }

            // The block above the wire. If it's solid, we can't connect up diagonally
            let up_pos = pos.offset(BlockFace::Top);
            let up_block = self.block_cache.block(up_pos);

            for side in &BlockFace::values() {
                let neighbor_pos = pos.offset(*side);
                let neighbor = self.block_cache.block(neighbor_pos);

                self.get_redstone_links(
                    neighbor,
                    *side,
                    neighbor_pos,
                    link_kind,
                    distance,
                    start_node,
                    false,
                );

                if is_wire(neighbor) && !has_been_discovered(&discovered, neighbor_pos) {
                    discovered.push((neighbor_pos, distance + 1));
                }

                if side.is_horizontal() {
                    if !neighbor.is_conductor() {
                        let neighbor_down_pos = neighbor_pos.offset(BlockFace::Bottom);
                        if is_wire(self.block_cache.block(neighbor_down_pos))
                            && !has_been_discovered(&discovered, neighbor_down_pos)
                        {
                            discovered.push((neighbor_down_pos, distance + 1));
                        }
                    } else if !up_block.is_conductor() && !neighbor.is_transparent() {
                        let neighbor_up_pos = neighbor_pos.offset(BlockFace::Top);
                        if is_wire(self.block_cache.block(neighbor_up_pos))
                            && !has_been_discovered(&discovered, neighbor_up_pos)
                        {
                            discovered.push((neighbor_up_pos, distance + 1));
                        }
                    }
                }
            }
        }
    }

    fn search_diode_inputs(&mut self, id: NodeIdx, pos: BlockPos, facing: BlockDirection) {
        let input_pos = pos.offset(facing.block_face());
        let input_block = self.block_cache.block(input_pos);
        self.get_redstone_links(
            input_block,
            facing.block_face(),
            input_pos,
            LinkKind::Default,
            0,
            id,
            true,
        )
    }

    fn search_repeater_side(&mut self, id: NodeIdx, pos: BlockPos, side: BlockDirection) {
        let side_pos = pos.offset(side.block_face());
        let side_block = self.block_cache.block(side_pos);
        if mchprs_redstone::is_diode(side_block)
            && provides_weak_power(side_block, side.block_face())
        {
            self.add_link(side_pos, id, Link::side(0));
        }
    }

    fn search_comparator_side(&mut self, id: NodeIdx, pos: BlockPos, side: BlockDirection) {
        let side_pos = pos.offset(side.block_face());
        let side_block = self.block_cache.block(side_pos);
        if (mchprs_redstone::is_diode(side_block)
            && provides_weak_power(side_block, side.block_face()))
            || matches!(side_block, Block::RedstoneBlock)
        {
            self.add_link(side_pos, id, Link::side(0));
        } else if matches!(side_block, Block::RedstoneWire { .. }) {
            self.search_wire(id, side_pos, LinkKind::Side, 0)
        }
    }

    fn search_node(&mut self, id: NodeIdx, (pos, block_id): (BlockPos, u32)) {
        match Block::from_id(block_id) {
            Block::RedstoneTorch { .. } => {
                let bottom_pos = pos.offset(BlockFace::Bottom);
                let bottom_block = self.block_cache.block(bottom_pos);
                self.get_redstone_links(
                    bottom_block,
                    BlockFace::Top,
                    bottom_pos,
                    LinkKind::Default,
                    0,
                    id,
                    true,
                );
            }
            Block::RedstoneWallTorch { facing, .. } => {
                let wall_pos = pos.offset(facing.opposite().block_face());
                let wall_block = self.block_cache.block(wall_pos);
                self.get_redstone_links(
                    wall_block,
                    facing.opposite().block_face(),
                    wall_pos,
                    LinkKind::Default,
                    0,
                    id,
                    true,
                );
            }
            Block::Comparator(comparator) => {
                let facing = comparator.facing;

                self.search_comparator_side(id, pos, facing.rotate());
                self.search_comparator_side(id, pos, facing.rotate_ccw());

                let input_pos = pos.offset(facing.block_face());
                let input_block = self.block_cache.block(input_pos);
                if comparator::analog_output(input_block, self.world, input_pos).is_some() {
                    self.add_link(input_pos, id, Link::default(0));
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
                    let neighbor_block = self.block_cache.block(neighbor_pos);
                    self.get_redstone_links(
                        neighbor_block,
                        *face,
                        neighbor_pos,
                        LinkKind::Default,
                        0,
                        id,
                        true,
                    );
                }
            }
            _ => {}
        }
    }

    fn search(&mut self) {
        for i in 0..self.graph.node_bound() {
            let idx = NodeIdx::new(i);
            if !self.graph.contains_node(idx) || self.graph[idx].is_input() {
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
