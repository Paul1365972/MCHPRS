use super::expression::{Expression, Moment};
use crate::circuit::Link;
use crate::registers::Registers;

pub(super) struct Inputs<'a> {
    registers: &'a Registers<'a>,
}

impl<'a> Inputs<'a> {
    pub(super) fn new(registers: &'a Registers<'a>) -> Self {
        Self { registers }
    }

    fn reachable(&self, link: &Link) -> u16 {
        let strengths = self.registers.strengths(link.source);
        (0_u8..=15)
            .filter(|strength| strengths & (1 << strength) != 0)
            .fold(0, |reachable, strength| {
                reachable | (1 << strength.saturating_sub(link.attenuation))
            })
    }

    fn signal(&self, source: usize, moment: Moment) -> Expression {
        match self.registers.literal(source) {
            Some(strength) => Expression::Strength(strength),
            None => Expression::Signal(source, moment),
        }
    }

    fn powers(&self, link: &Link, moment: Moment) -> Option<Expression> {
        let reachable = self.reachable(link);
        if reachable == 1 {
            return None;
        }
        if reachable & 1 == 0 {
            return Some(Expression::Bit(true));
        }
        let strengths = self.registers.strengths(link.source);
        let signal = self.signal(link.source, moment);
        let weak = (1..=link.attenuation).any(|strength| strengths & (1 << strength) != 0);
        Some(if weak {
            signal.above(link.attenuation)
        } else {
            signal.nonzero()
        })
    }

    pub(super) fn power(&self, links: &[Link]) -> Expression {
        Expression::any(
            links
                .iter()
                .filter_map(|link| self.powers(link, Moment::AfterFiring)),
        )
    }

    pub(super) fn strength(&self, links: &[Link]) -> Expression {
        let values = links
            .iter()
            .fold(1, |values, link| values | self.reachable(link));
        if values == 1 {
            return Expression::Strength(0);
        }
        if values.count_ones() == 2 {
            let high = (15 - values.leading_zeros()) as u8;
            return self
                .power(links)
                .select(Expression::Strength(high), Expression::Strength(0));
        }
        let mut constant = 0;
        let mut terms = Vec::new();
        for link in links.iter().filter(|link| self.reachable(link) != 1) {
            match self.registers.literal(link.source) {
                Some(strength) => {
                    constant = constant.max(strength.saturating_sub(link.attenuation));
                }
                None => terms.push(
                    self.signal(link.source, Moment::AfterFiring)
                        .attenuated(link.attenuation),
                ),
            }
        }
        if constant != 0 {
            terms.push(Expression::Strength(constant));
        }
        Expression::maximum(&terms)
    }

    fn link_changed(&self, link: &Link) -> Option<Expression> {
        if self.reachable(link).count_ones() < 2 {
            return None;
        }
        let strengths = self.registers.strengths(link.source);
        let read = |moment| {
            let signal = self.signal(link.source, moment);
            if strengths.count_ones() == 2 && strengths & 1 != 0 {
                signal.nonzero()
            } else {
                signal.attenuated(link.attenuation)
            }
        };
        Some(read(Moment::AfterFiring).differs(read(Moment::BeforeFiring)))
    }

    pub(super) fn changed(&self, inputs: &[Link], sides: &[Link]) -> Expression {
        Expression::any(
            inputs
                .iter()
                .chain(sides)
                .filter_map(|link| self.link_changed(link)),
        )
    }
}
