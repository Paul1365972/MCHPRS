use crate::circuit::StrengthSet;
use mchprs_redpiler::compile_graph::SignalStrength;

#[derive(Clone, Default)]
pub(super) struct Aggregate {
    counts: [usize; 16],
    occupied: StrengthSet,
}

impl Aggregate {
    pub(super) fn insert(&mut self, strength: SignalStrength) {
        self.counts[usize::from(strength.get())] += 1;
        self.occupied.insert(strength);
    }

    pub(super) fn replace(&mut self, previous: SignalStrength, strength: SignalStrength) {
        let count = &mut self.counts[usize::from(previous.get())];
        *count -= 1;
        if *count == 0 {
            self.occupied.remove(previous);
        }
        self.insert(strength);
    }

    pub(super) fn maximum(&self) -> SignalStrength {
        self.occupied.maximum().unwrap_or_default()
    }
}
