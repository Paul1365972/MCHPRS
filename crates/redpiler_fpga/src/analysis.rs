use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::analysis::ss_sets::{ss_sets, SSSet};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::{Graph, Link, NodeIdx};

#[derive(Debug)]
pub struct CircuitAnalysis {
    strengths: Vec<SSSet>,
}

impl CircuitAnalysis {
    pub fn of(circuit: &Circuit) -> CircuitAnalysis {
        let mut graph = Graph::default();
        let nodes = circuit.ids().map(|id| circuit.node(id).clone());
        let indices: Vec<NodeIdx> = nodes.map(|node| graph.add_node(node)).collect();
        let index = |id: NodeId| indices[id.index()];
        for link in circuit.ids().flat_map(|id| circuit.outputs(id)) {
            let weights = Link::new(link.kind, link.weight);
            graph.add_link(index(link.source), index(link.target), weights);
        }
        let sets = ss_sets(&graph);
        CircuitAnalysis {
            strengths: circuit.ids().map(|id| sets.of(index(id))).collect(),
        }
    }

    pub fn strengths(&self, node: NodeId) -> SSSet {
        self.strengths[node.index()]
    }

    /// The output that the inputs of such a comparator give is always one of the two strengths.
    pub fn binary_strengths(&self, node: NodeId) -> Option<(SignalStrength, SignalStrength)> {
        match strengths_of(self.strengths(node)).as_slice() {
            &[low, high] => Some((low, high)),
            _ => None,
        }
    }
}

pub(crate) fn strengths_of(set: SSSet) -> Vec<SignalStrength> {
    SignalStrength::all()
        .filter(|strength| set.contains(strength.get()))
        .collect()
}
