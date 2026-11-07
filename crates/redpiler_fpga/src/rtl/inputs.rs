use super::expression::Expression;
use crate::circuit::{Link, StrengthSet};
use crate::registers::Registers;
use mchprs_redpiler::compile_graph::SignalStrength;

/// A maximum is built from its thresholds instead of a tree when its terms deliver at most this
/// many nonzero strengths each on average, so that each threshold is an OR of few comparisons.
const NONZERO_STRENGTHS_PER_TERM: usize = 2;

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

    /// Whether the link delivers at least `threshold`, or `None` if it never does.
    fn reaches(&self, link: &Link, threshold: u8) -> Option<Expression> {
        let reachable = self.reachable(link);
        if reachable.maximum()?.get() < threshold {
            return None;
        }
        if reachable.minimum()?.get() >= threshold {
            return Some(Expression::Bit(true));
        }
        let floor = threshold + link.attenuation;
        let signal = self.signal(link.source);
        let weak = self
            .registers
            .strengths(link.source)
            .iter()
            .any(|strength| (1..floor).contains(&strength.get()));
        Some(if weak {
            signal.above(floor - 1)
        } else {
            signal.nonzero()
        })
    }

    pub(super) fn power(&self, links: &[Link]) -> Expression {
        Expression::any(links.iter().filter_map(|link| self.reaches(link, 1)))
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
        let mut nonzero_strengths = 0;
        for link in links
            .iter()
            .filter(|link| self.reachable(link) != StrengthSet::OFF)
        {
            match self.registers.literal(link.source) {
                Some(strength) => {
                    constant = constant.max(strength.saturating_sub(link.attenuation));
                }
                None => {
                    nonzero_strengths += self.reachable(link).len() as usize - 1;
                    terms.push(self.signal(link.source).attenuated(link.attenuation));
                }
            }
        }
        if terms.len() > 1 && nonzero_strengths <= NONZERO_STRENGTHS_PER_TERM * terms.len() {
            return Expression::HighestReached(
                (1..=15)
                    .rev()
                    .map(|threshold| {
                        Expression::any(
                            links
                                .iter()
                                .filter_map(|link| self.reaches(link, threshold)),
                        )
                    })
                    .collect(),
            );
        }
        if !constant.is_zero() {
            terms.push(Expression::Strength(constant));
        }
        Expression::maximum(&terms)
    }
}
