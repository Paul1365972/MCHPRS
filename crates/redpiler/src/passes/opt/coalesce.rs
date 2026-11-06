//! # [`Coalesce`]
//!
//! Merges nodes with the same type, initial state and inputs, except inputs, nodes with a pending
//! tick and nodes whose inputs could change between their ticks. Comparators have the same inputs
//! when they output the same strength for every combination of their sources' possible strengths.

use crate::compile_graph::{
    comparator_output, CompileGraph, CompileLink, Direction, LinkType, NodeIdx, NodeState,
    NodeType, SignalStrength,
};
use crate::passes::analysis::ss_set_analysis::{SSSetAnalysis, SSSetInfo};
use crate::passes::{AnalysisInfos, AnalysisUsage, Pass};
use crate::{CompilerInput, CompilerOptions};
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_world::World;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;
use std::collections::VecDeque;
use tracing::trace;

/// Comparators whose sources can take more combinations of strengths keep their links as inputs.
const MAX_COMPARATOR_COMBINATIONS: usize = 256;

pub struct Coalesce;

impl<W: World> Pass<W> for Coalesce {
    fn run_pass(
        &self,
        graph: &mut CompileGraph,
        _: &CompilerOptions,
        _: &CompilerInput<'_, W>,
        analysis_infos: &mut AnalysisInfos,
    ) {
        let strengths = analysis_infos
            .get_analysis::<SSSetInfo>()
            .expect("coalesce requires the signal strength set analysis");
        // Signatures of nodes whose inputs changed name a removed node, so they never match again.
        let mut nodes_by_signature = FxHashMap::default();
        let mut queue: VecDeque<NodeIdx> = graph.node_indices().collect();
        let mut queued = vec![true; graph.node_bound()];
        let mut num_coalesced = 0;
        while let Some(node) = queue.pop_front() {
            queued[node.index()] = false;
            if !graph.contains_node(node) {
                continue;
            }
            if graph[node].is_input || graph[node].state.pending_tick {
                continue;
            }

            let signature = signature(graph, strengths, node);
            if !inputs_are_stable(graph, &signature, node) {
                continue;
            }
            let Some(&into) = nodes_by_signature.get(&signature) else {
                nodes_by_signature.insert(signature, node);
                continue;
            };

            for (target, link) in take_outgoing_links(graph, node) {
                graph.add_edge(into, target, link);
                if !queued[target.index()] {
                    queued[target.index()] = true;
                    queue.push_back(target);
                }
            }
            let mut removed = graph.remove_node(node).unwrap();
            graph[into].block.append(&mut removed.block);
            num_coalesced += 1;
        }
        trace!("Coalesced {} nodes", num_coalesced);
    }

    fn analysis_usage(&self, au: &mut AnalysisUsage) {
        au.set_required::<SSSetAnalysis, W>();
    }

    fn status_message(&self) -> &'static str {
        "Combining duplicate logic"
    }

    fn driver_key(&self) -> &'static str {
        "coalesce"
    }
}

#[derive(PartialEq, Eq, Hash)]
struct Signature {
    ty: NodeType,
    state: NodeState,
    is_output: bool,
    inputs: Inputs,
}

#[derive(PartialEq, Eq, Hash)]
enum Inputs {
    /// The strongest link per source and link type, by source, link type and weight
    Links(SmallVec<[(NodeIdx, LinkType, u8); 4]>),
    /// A comparator's output for each combination of its sources' possible strengths, counting
    /// through the first source fastest
    Outputs {
        sources: SmallVec<[NodeIdx; 4]>,
        outputs: SmallVec<[SignalStrength; 16]>,
    },
}

impl Inputs {
    fn sources(&self) -> SmallVec<[NodeIdx; 4]> {
        match self {
            Inputs::Links(links) => links.iter().map(|&(source, ..)| source).collect(),
            Inputs::Outputs { sources, .. } => sources.clone(),
        }
    }
}

fn signature(graph: &CompileGraph, strengths: &SSSetInfo, idx: NodeIdx) -> Signature {
    let node = &graph[idx];
    if let NodeType::Comparator {
        mode,
        far_input,
        facing_diode,
    } = node.ty
        && let Some(inputs) = comparator_outputs(graph, strengths, idx, mode, far_input)
    {
        return Signature {
            ty: NodeType::Comparator {
                mode: ComparatorMode::Compare,
                far_input: None,
                facing_diode,
            },
            state: node.state,
            is_output: node.is_output,
            inputs,
        };
    }
    let mut inputs: SmallVec<_> = graph
        .edges(idx, Direction::Incoming)
        .map(|edge| {
            let link = edge.weight();
            let carries_binary_signal = link.weight < 15
                && node.ty.is_binary_reader()
                && graph[edge.source()].ty.is_binary_source();
            let weight = if carries_binary_signal {
                0
            } else {
                link.weight
            };
            (edge.source(), link.ty, weight)
        })
        .collect();
    inputs.sort_unstable();
    inputs.dedup_by_key(|&mut (source, ty, _)| (source, ty));
    Signature {
        ty: node.ty.clone(),
        state: node.state,
        is_output: node.is_output,
        inputs: Inputs::Links(inputs),
    }
}

fn comparator_outputs(
    graph: &CompileGraph,
    strengths: &SSSetInfo,
    idx: NodeIdx,
    mode: ComparatorMode,
    far_input: Option<SignalStrength>,
) -> Option<Inputs> {
    let mut sources: SmallVec<[NodeIdx; 4]> = graph.neighbors(idx, Direction::Incoming).collect();
    sources.sort_unstable();
    sources.dedup();
    let possible: SmallVec<[SmallVec<[u8; 16]>; 4]> = sources
        .iter()
        .map(|&source| {
            let set = strengths.get_set(source)?;
            Some((0..16).filter(|&ss| set.contains(ss)).collect())
        })
        .collect::<Option<_>>()?;
    let combinations = possible
        .iter()
        .try_fold(1usize, |combinations, strengths| {
            Some(combinations * strengths.len()).filter(|&c| c <= MAX_COMPARATOR_COMBINATIONS)
        })?;

    let mut assignment: SmallVec<[u8; 4]> = SmallVec::from_elem(0, sources.len());
    let mut outputs = SmallVec::new();
    for combination in 0..combinations {
        let mut rest = combination;
        for (strength, choices) in assignment.iter_mut().zip(&possible) {
            *strength = choices[rest % choices.len()];
            rest /= choices.len();
        }
        let strongest = |ty: LinkType| {
            graph
                .edges(idx, Direction::Incoming)
                .filter(|edge| edge.weight().ty == ty)
                .map(|edge| {
                    let source = sources.binary_search(&edge.source()).unwrap();
                    assignment[source].saturating_sub(edge.weight().weight)
                })
                .max()
                .unwrap_or(0)
        };
        let rear = SignalStrength::try_from(strongest(LinkType::Default)).unwrap();
        let side = SignalStrength::try_from(strongest(LinkType::Side)).unwrap();
        outputs.push(comparator_output(mode, far_input, rear, side));
    }
    Some(Inputs::Outputs { sources, outputs })
}

// An input updated by another input can tick between two identical nodes and change in between
fn inputs_are_stable(graph: &CompileGraph, signature: &Signature, node: NodeIdx) -> bool {
    let sources = signature.inputs.sources();
    let is_input = |idx| sources.contains(&idx);
    !is_input(node)
        && sources
            .iter()
            .all(|&source| !graph.neighbors(source, Direction::Incoming).any(is_input))
}

fn take_outgoing_links(graph: &mut CompileGraph, node: NodeIdx) -> Vec<(NodeIdx, CompileLink)> {
    let mut links = Vec::new();
    let mut outgoing = graph.neighbors(node, Direction::Outgoing).detach();
    while let Some((edge_idx, target)) = outgoing.next(graph) {
        links.push((target, graph.remove_edge(edge_idx).unwrap()));
    }
    links
}
