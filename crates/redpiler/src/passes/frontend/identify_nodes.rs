//! # [`IdentifyNodes`]
//!
//! This pass populates the graph with nodes using the input given in [`CompilerInput`].
//! This pass is *mandatory*. Without it, the graph will never be populated.
//!
//! If `optimize` is set in [`CompilerOptions`], redstone wires will not be added to the graph.
//!
//! There are no requirements for this pass.

use crate::compile_graph::{Annotations, CompileGraph, CompileNode, NodeIdx, NodeState, NodeType};
use crate::passes::{AnalysisInfos, Pass};
use crate::{CompilerInput, CompilerOptions};
use itertools::Itertools;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use mchprs_redstone::{self, comparator, noteblock, wire};
use rustc_hash::{FxHashMap, FxHashSet};
use serde_json::Value;
use smallvec::smallvec;
use tracing::warn;

pub struct IdentifyNodes;

impl<W: BlockView> Pass<W> for IdentifyNodes {
    fn run_pass(
        &self,
        graph: &mut CompileGraph,
        options: &CompilerOptions,
        input: &CompilerInput<'_, W>,
        _: &mut AnalysisInfos,
    ) {
        let plot = input.world;

        let mut first_pass = FxHashMap::default();
        let mut second_pass = FxHashSet::default();

        for pos in plot
            .occupied_regions(input.region)
            .into_iter()
            .flat_map(Region::positions)
        {
            for_pos(graph, &mut first_pass, &mut second_pass, options, plot, pos);
        }

        for tick in input.ticks {
            if let Some(&idx) = first_pass.get(&tick.position) {
                graph[idx].state.pending_tick = true;
            }
        }

        for pos in second_pass {
            apply_annotations(graph, options, &first_pass, plot, pos);
        }
    }

    fn status_message(&self) -> &'static str {
        "Identifying nodes"
    }

    fn driver_key(&self) -> &'static str {
        "identify-nodes"
    }
}

fn for_pos<W: BlockView>(
    graph: &mut CompileGraph,
    first_pass: &mut FxHashMap<BlockPos, NodeIdx>,
    second_pass: &mut FxHashSet<BlockPos>,
    options: &CompilerOptions,
    world: &W,
    pos: BlockPos,
) {
    let block = world.block(pos);

    if block.is_sign() || block.is_wall_sign() {
        second_pass.insert(pos);
        return;
    }

    let Some((ty, state)) = identify_block(block, pos, world) else {
        return;
    };

    let is_input = ty.is_normally_input();
    let is_output = ty.is_normally_output()
        || matches!(
            block,
            Block::RedstoneWire(wire) if
                options.wire_dot_out && wire::is_dot(wire) ||
                options.illegal_states_out && wire::is_illegal(wire) ||
                options.wire_cross_out && wire::is_cross(wire)
        );

    if options.optimize && ty == NodeType::Wire && !(is_input | is_output) {
        return;
    }

    let node_idx = graph.add_node(CompileNode {
        ty,
        block: smallvec![(pos, block.get_id())],
        name: None,
        state,

        is_input,
        is_output,
        annotations: Annotations::default(),
    });
    first_pass.insert(pos, node_idx);
}

fn identify_block<W: BlockView>(
    block: Block,
    pos: BlockPos,
    world: &W,
) -> Option<(NodeType, NodeState)> {
    if let Some(powered) = block.clone().get_pressure_plate_powered() {
        return Some((NodeType::PressurePlate, NodeState::from_powered(*powered)));
    }
    let (ty, state) = match block {
        Block::Repeater(repeater) => (
            NodeType::Repeater {
                delay: repeater.delay,
                prioritized: mchprs_redstone::diode_should_be_prioritized(
                    world,
                    pos,
                    repeater.facing,
                ),
            },
            NodeState::repeater(repeater.powered, repeater.locked),
        ),
        Block::Comparator(comparator) => (
            NodeType::Comparator {
                mode: comparator.mode,
                far_input: comparator::get_far_input(world, pos, comparator.facing).map(|power| {
                    SignalStrength::try_from(power).expect("invalid comparator far input")
                }),
                prioritized: mchprs_redstone::diode_should_be_prioritized(
                    world,
                    pos,
                    comparator.facing,
                ),
            },
            NodeState::from_power(
                if let Some(BlockEntity::Comparator { output_strength }) = world.block_entity(pos) {
                    *output_strength
                } else {
                    SignalStrength::Zero
                },
            ),
        ),
        Block::RedstoneTorch { lit, .. } | Block::RedstoneWallTorch { lit, .. } => {
            (NodeType::Torch, NodeState::from_powered(lit))
        }
        Block::RedstoneWire(wire) => (
            NodeType::Wire,
            NodeState::from_power(
                SignalStrength::try_from(wire.power).expect("invalid wire power"),
            ),
        ),
        Block::StoneButton { powered, .. } => (NodeType::Button, NodeState::from_powered(powered)),
        Block::RedstoneLamp { lit } => (NodeType::Lamp, NodeState::from_powered(lit)),
        Block::Lever { powered, .. } => (NodeType::Lever, NodeState::from_powered(powered)),
        Block::IronTrapdoor { powered, .. } => {
            (NodeType::Trapdoor, NodeState::from_powered(powered))
        }
        Block::RedstoneBlock => (
            NodeType::Constant,
            NodeState::from_power(SignalStrength::Fifteen),
        ),
        Block::NoteBlock {
            instrument: _,
            note,
            powered,
        } if noteblock::is_noteblock_unblocked(world, pos) => {
            let instrument = noteblock::get_noteblock_instrument(world, pos);
            (
                NodeType::NoteBlock { instrument, note },
                NodeState::from_powered(powered),
            )
        }
        _ => {
            let power = comparator::analog_output(block, world, pos)?;
            (
                NodeType::Constant,
                NodeState::from_power(
                    SignalStrength::try_from(power).expect("invalid comparator override"),
                ),
            )
        }
    };
    Some((ty, state))
}

fn apply_annotations<W: BlockView>(
    graph: &mut CompileGraph,
    options: &CompilerOptions,
    first_pass: &FxHashMap<BlockPos, NodeIdx>,
    world: &W,
    pos: BlockPos,
) {
    let block = world.block(pos);
    let annotations = parse_sign_annotations(world.block_entity(pos));
    if annotations.is_empty() {
        return;
    }

    let targets = match (block.get_sign_rotation(), block.get_wall_sign_facing()) {
        (Some(rotation), None) => {
            if let Some(facing) = BlockDirection::from_rotation(rotation) {
                let behind = pos.offset(facing.opposite().block_face());
                vec![behind]
            } else {
                warn!("Found sign with annotations, but bad rotation at {}", pos);
                return;
            }
        }
        (None, Some(facing)) => {
            let behind = pos.offset(facing.opposite().block_face());
            vec![
                behind,
                behind.offset(BlockFace::Top),
                behind.offset(BlockFace::Bottom),
            ]
        }
        _ => panic!("Block unimplemented for second pass"),
    };

    let target = targets.iter().flat_map(|pos| first_pass.get(pos)).next();
    if let Some(&node_idx) = target {
        for annotation in annotations {
            let result = annotation.apply(graph, node_idx, options);
            if let Err(msg) = result {
                warn!("{} at {}", msg, pos);
            }
        }
    } else {
        warn!("Could not find component for annotation at {}", pos);
    }
}

fn parse_sign_annotations(entity: Option<&BlockEntity>) -> Vec<NodeAnnotation> {
    if let Some(BlockEntity::Sign(sign)) = entity {
        sign.front_rows
            .iter()
            .flat_map(|row| serde_json::from_str(row))
            .flat_map(|json: Value| NodeAnnotation::parse(json.as_object()?.get("text")?.as_str()?))
            .collect_vec()
    } else {
        vec![]
    }
}

pub enum NodeAnnotation {}

impl NodeAnnotation {
    fn parse(s: &str) -> Option<Self> {
        let s = s.trim().to_ascii_lowercase();
        if !(s.starts_with('[') && s.ends_with(']')) {
            return None;
        }
        let _parts = s[1..s.len() - 1].split(' ').collect_vec();
        None
    }

    fn apply(
        self,
        _graph: &mut CompileGraph,
        _node_idx: NodeIdx,
        _options: &CompilerOptions,
    ) -> Result<(), String> {
        match self {}
    }
}
