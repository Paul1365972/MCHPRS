use mchprs_redpiler::compile_graph::SignalStrength;
use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Expression {
    Bit(bool),
    Strength(SignalStrength),
    Signal(usize),
    Nonzero(Box<Expression>),
    Above(Box<Expression>, u8),
    Attenuated(Box<Expression>, u8),
    Maximum(Box<Expression>, Box<Expression>),
    Select(Box<Expression>, Box<Expression>, Box<Expression>),
    Any(Vec<Expression>),
}

impl Expression {
    pub(super) fn nonzero(self) -> Self {
        Self::Nonzero(Box::new(self))
    }

    pub(super) fn above(self, strength: u8) -> Self {
        Self::Above(Box::new(self), strength)
    }

    pub(super) fn attenuated(self, attenuation: u8) -> Self {
        match attenuation {
            0 => self,
            attenuation => Self::Attenuated(Box::new(self), attenuation),
        }
    }

    pub(super) fn select(self, then: Self, otherwise: Self) -> Self {
        match self {
            Self::Bit(true) => then,
            Self::Bit(false) => otherwise,
            condition => Self::Select(Box::new(condition), Box::new(then), Box::new(otherwise)),
        }
    }

    pub(super) fn any(terms: impl IntoIterator<Item = Self>) -> Self {
        let mut flattened = Vec::new();
        for term in terms {
            match term {
                Self::Any(nested) => flattened.extend(nested),
                term => flattened.push(term),
            }
        }
        flattened.retain(|term| *term != Self::Bit(false));
        if flattened.contains(&Self::Bit(true)) {
            return Self::Bit(true);
        }
        flattened.sort();
        flattened.dedup();
        match flattened.len() {
            0 => Self::Bit(false),
            1 => flattened.remove(0),
            _ => Self::Any(flattened),
        }
    }

    pub(super) fn maximum(terms: &[Self]) -> Self {
        match terms {
            [] => Self::Strength(SignalStrength::Zero),
            [term] => term.clone(),
            terms => {
                let (left, right) = terms.split_at(terms.len() / 2);
                Self::Maximum(
                    Box::new(Self::maximum(left)),
                    Box::new(Self::maximum(right)),
                )
            }
        }
    }

    pub(super) fn visit_signals(&self, visit: &mut impl FnMut(usize)) {
        match self {
            Self::Bit(_) | Self::Strength(_) => {}
            Self::Signal(node) => visit(*node),
            Self::Nonzero(term) | Self::Above(term, _) | Self::Attenuated(term, _) => {
                term.visit_signals(visit)
            }
            Self::Maximum(left, right) => {
                left.visit_signals(visit);
                right.visit_signals(visit);
            }
            Self::Select(condition, then, otherwise) => {
                for term in [condition, then, otherwise] {
                    term.visit_signals(visit);
                }
            }
            Self::Any(terms) => terms.iter().for_each(|term| term.visit_signals(visit)),
        }
    }

    fn is_operation(&self) -> bool {
        matches!(self, Self::Above(..) | Self::Select(..) | Self::Any(_))
    }

    fn operand(&self) -> Operand<'_> {
        Operand(self)
    }
}

pub(super) fn signal_name(node: usize) -> String {
    format!("next_signal_{node}")
}

struct Operand<'a>(&'a Expression);

impl fmt::Display for Operand<'_> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0.is_operation() {
            write!(formatter, "({})", self.0)
        } else {
            write!(formatter, "{}", self.0)
        }
    }
}

impl fmt::Display for Expression {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Bit(value) => write!(formatter, "1'b{}", u8::from(*value)),
            Self::Strength(strength) => write!(formatter, "4'd{strength}"),
            Self::Signal(node) => formatter.write_str(&signal_name(*node)),
            Self::Nonzero(term) => write!(formatter, "|{}", term.operand()),
            Self::Above(term, strength) => write!(formatter, "{} > 4'd{strength}", term.operand()),
            Self::Attenuated(term, attenuation) => {
                write!(formatter, "attenuated({term}, 4'd{attenuation})")
            }
            Self::Maximum(left, right) => write!(formatter, "maximum({left}, {right})"),
            Self::Select(condition, then, otherwise) => write!(
                formatter,
                "{} ? {} : {}",
                condition.operand(),
                then.operand(),
                otherwise.operand()
            ),
            Self::Any(terms) => {
                for (position, term) in terms.iter().enumerate() {
                    if position != 0 {
                        formatter.write_str(" || ")?;
                    }
                    write!(formatter, "{}", term.operand())?;
                }
                Ok(())
            }
        }
    }
}
