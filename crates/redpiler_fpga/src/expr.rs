use crate::analysis::{strengths_of, CircuitAnalysis};
use crate::lower::{Instance, Register};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::analysis::ss_sets::SSSet;
use mchprs_redpiler::circuit::{Circuit, Link, NodeId};
use mchprs_redpiler::graph::LinkKind;

#[derive(Debug, Clone, Copy)]
enum Output {
    Fixed(u8),
    Binary,
    Strength,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Term<T> {
    Fixed(T),
    Signal(String),
}

pub struct Expressions<'a> {
    circuit: &'a Circuit,
    analysis: &'a CircuitAnalysis,
    instances: &'a [Option<Instance>],
    output_is_read: Vec<bool>,
}

impl<'a> Expressions<'a> {
    pub fn new(
        circuit: &'a Circuit,
        analysis: &'a CircuitAnalysis,
        instances: &'a [Option<Instance>],
    ) -> Expressions<'a> {
        Expressions {
            circuit,
            analysis,
            instances,
            output_is_read: vec![false; circuit.len()],
        }
    }

    pub fn into_read_outputs(self) -> Vec<bool> {
        self.output_is_read
    }

    pub fn on(&mut self, node: NodeId, kind: LinkKind) -> String {
        match self.any_link_reaches(node, kind, 1) {
            Term::Fixed(on) => format!("1'b{}", u8::from(on)),
            Term::Signal(signal) => signal,
        }
    }

    pub fn strength(&mut self, node: NodeId, kind: LinkKind) -> String {
        let links: Vec<_> = self.links(node, kind).collect();
        let carried: Vec<SSSet> = links.iter().map(|link| self.carried_set(link)).collect();
        let strengths = strengths_of(carried.iter().copied().fold(SSSet::ZERO, SSSet::combine));
        let changing: Vec<SSSet> = carried.into_iter().filter(|set| !set.is_single()).collect();
        let levels: usize = changing.iter().map(|&set| nonzero_strengths(set)).sum();
        // Built from its thresholds, a maximum is smaller and faster than a tree of 4-bit maxima
        // while its terms have at most two nonzero strengths each on average (BatPU-1).
        if strengths.len() == 2 || changing.len() >= 2 && levels <= 2 * changing.len() {
            return self.highest_reached(node, kind, &strengths);
        }
        let mut fixed = 0;
        let mut signals = Vec::new();
        for link in links {
            match self.carried_term(link) {
                Term::Fixed(strength) => fixed = fixed.max(strength),
                Term::Signal(signal) => signals.push(signal),
            }
        }
        if fixed != 0 || signals.is_empty() {
            signals.push(format!("4'd{fixed}"));
        }
        maximum(&signals)
    }

    fn highest_reached(
        &mut self,
        node: NodeId,
        kind: LinkKind,
        strengths: &[SignalStrength],
    ) -> String {
        let (lowest, higher) = strengths
            .split_first()
            .expect("a node's input has a strength");
        let mut highest = format!("4'd{lowest}");
        for strength in higher {
            highest = match self.any_link_reaches(node, kind, strength.get()) {
                Term::Fixed(true) => format!("4'd{strength}"),
                Term::Fixed(false) => highest,
                Term::Signal(reached) => format!("({reached} ? 4'd{strength} : {highest})"),
            };
        }
        highest
    }

    fn links(&self, node: NodeId, kind: LinkKind) -> impl Iterator<Item = &'a Link> + use<'a> {
        let circuit = self.circuit;
        let links = circuit.inputs(node).iter();
        links.filter(move |link| link.kind == kind)
    }

    fn carried_set(&self, link: &Link) -> SSSet {
        self.analysis.strengths(link.source).decay(link.weight)
    }

    fn output(&self, node: NodeId) -> Output {
        let strengths = self.analysis.strengths(node);
        let power = self.instances[node.index()]
            .as_ref()
            .filter(|_| !strengths.is_single())
            .map(|instance| instance.power);
        match power {
            Some(Register::Powered) => Output::Binary,
            Some(Register::Strength) => Output::Strength,
            _ => Output::Fixed(strengths.min()),
        }
    }

    fn read_output_wire(&mut self, node: NodeId) -> String {
        self.output_is_read[node.index()] = true;
        output_wire(node.index())
    }

    fn any_link_reaches(&mut self, node: NodeId, kind: LinkKind, threshold: u8) -> Term<bool> {
        let mut signals = Vec::new();
        for link in self.links(node, kind) {
            match self.link_reaches(link, threshold) {
                Term::Fixed(true) => return Term::Fixed(true),
                Term::Fixed(false) => {}
                Term::Signal(signal) => signals.push(signal),
            }
        }
        match signals.is_empty() {
            true => Term::Fixed(false),
            false => Term::Signal(signals.join(" || ")),
        }
    }

    fn link_reaches(&mut self, link: &Link, threshold: u8) -> Term<bool> {
        let floor = threshold + link.weight;
        let strengths = self.analysis.strengths(link.source);
        match self.output(link.source) {
            Output::Fixed(strength) => Term::Fixed(strength >= floor),
            Output::Binary if floor > 15 => Term::Fixed(false),
            Output::Binary => Term::Signal(self.read_output_wire(link.source)),
            Output::Strength if strengths.min() >= floor => Term::Fixed(true),
            Output::Strength if strengths.max() < floor => Term::Fixed(false),
            Output::Strength if (1..floor).any(|weak| strengths.contains(weak)) => Term::Signal(
                format!("{} >= 4'd{floor}", self.read_output_wire(link.source)),
            ),
            Output::Strength => Term::Signal(format!("|{}", self.read_output_wire(link.source))),
        }
    }

    fn carried_term(&mut self, link: &Link) -> Term<u8> {
        let carried = |strength: u8| strength.saturating_sub(link.weight);
        match self.output(link.source) {
            Output::Fixed(strength) => Term::Fixed(carried(strength)),
            _ if self.carried_set(link).max() == 0 => Term::Fixed(0),
            Output::Binary => {
                let high = carried(15);
                Term::Signal(format!(
                    "({} ? 4'd{high} : 4'd0)",
                    self.read_output_wire(link.source)
                ))
            }
            Output::Strength if link.weight == 0 => {
                Term::Signal(self.read_output_wire(link.source))
            }
            Output::Strength => {
                let (wire, weight) = (self.read_output_wire(link.source), link.weight);
                Term::Signal(format!("pkg::attenuated({wire}, 4'd{weight})"))
            }
        }
    }
}

pub fn output_wire(node: usize) -> String {
    format!("next_{node}")
}

fn nonzero_strengths(set: SSSet) -> usize {
    strengths_of(set)
        .iter()
        .filter(|strength| !strength.is_zero())
        .count()
}

fn maximum(strengths: &[String]) -> String {
    match strengths {
        [strength] => strength.clone(),
        strengths => {
            let (first, second) = strengths.split_at(strengths.len() / 2);
            format!("pkg::maximum({}, {})", maximum(first), maximum(second))
        }
    }
}
