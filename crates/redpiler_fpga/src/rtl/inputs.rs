use super::expression::Expression;
use crate::circuit::{Link, StrengthSet};
use crate::registers::Registers;
use mchprs_redpiler::compile_graph::SignalStrength;

pub(super) struct Inputs<'a> {
    registers: &'a Registers<'a>,
}

impl<'a> Inputs<'a> {
    pub(super) fn new(registers: &'a Registers<'a>) -> Self {
        Self { registers }
    }

    fn reachable(&self, link: &Link) -> StrengthSet {
        self.registers
            .strengths(link.source)
            .attenuated(link.attenuation)
    }

    fn signal(&self, source: usize) -> Expression {
        match self.registers.literal(source) {
            Some(strength) => Expression::Strength(strength),
            None => Expression::Signal(source),
        }
    }

    fn powers(&self, link: &Link) -> Option<Expression> {
        let reachable = self.reachable(link);
        if reachable == StrengthSet::OFF {
            return None;
        }
        if !reachable.contains(SignalStrength::Zero) {
            return Some(Expression::Bit(true));
        }
        let signal = self.signal(link.source);
        let weak = self
            .registers
            .strengths(link.source)
            .iter()
            .any(|strength| (1..=link.attenuation).contains(&strength.get()));
        Some(if weak {
            signal.above(link.attenuation)
        } else {
            signal.nonzero()
        })
    }

    pub(super) fn power(&self, links: &[Link]) -> Expression {
        Expression::any(links.iter().filter_map(|link| self.powers(link)))
    }

    pub(super) fn strength(&self, links: &[Link]) -> Expression {
        let values = links.iter().fold(StrengthSet::OFF, |values, link| {
            values | self.reachable(link)
        });
        let Some(high) = values.maximum().filter(|high| !high.is_zero()) else {
            return Expression::Strength(SignalStrength::Zero);
        };
        if values.len() == 2 {
            return self.power(links).select(
                Expression::Strength(high),
                Expression::Strength(SignalStrength::Zero),
            );
        }
        let mut constant = SignalStrength::Zero;
        let mut terms = Vec::new();
        for link in links
            .iter()
            .filter(|link| self.reachable(link) != StrengthSet::OFF)
        {
            match self.registers.literal(link.source) {
                Some(strength) => {
                    constant = constant.max(strength.saturating_sub(link.attenuation));
                }
                None => terms.push(self.signal(link.source).attenuated(link.attenuation)),
            }
        }
        if !constant.is_zero() {
            terms.push(Expression::Strength(constant));
        }
        Expression::maximum(&terms)
    }
}
