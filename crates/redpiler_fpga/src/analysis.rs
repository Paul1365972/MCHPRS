use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::analysis::ss_sets::{ss_sets, SSSet};
use mchprs_redpiler::circuit::{Circuit, NodeId};

#[derive(Debug)]
pub struct CircuitAnalysis {
    strengths: Vec<SSSet>,
}

impl CircuitAnalysis {
    pub fn of(circuit: &Circuit) -> CircuitAnalysis {
        let sets = ss_sets(circuit);
        CircuitAnalysis {
            strengths: circuit.ids().map(|id| sets.at(id.index())).collect(),
        }
    }

    pub fn strengths(&self, node: NodeId) -> SSSet {
        self.strengths[node.index()]
    }
}

pub(crate) fn strengths_of(set: SSSet) -> Vec<SignalStrength> {
    SignalStrength::all()
        .filter(|strength| set.contains(strength.get()))
        .collect()
}
