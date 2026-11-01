use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Moment {
    BeforeFiring,
    AfterFiring,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Expression {
    Bit(bool),
    Strength(u8),
    Signal(usize, Moment),
    Nonzero(Box<Expression>),
    Above(Box<Expression>, u8),
    Attenuated(Box<Expression>, u8),
    Maximum(Box<Expression>, Box<Expression>),
    Select(Box<Expression>, Box<Expression>, Box<Expression>),
    Differs(Box<Expression>, Box<Expression>),
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

    pub(super) fn differs(self, other: Self) -> Self {
        Self::Differs(Box::new(self), Box::new(other))
    }

    pub(super) fn select(self, then: Self, otherwise: Self) -> Self {
        match self {
            Self::Bit(true) => then,
            Self::Bit(false) => otherwise,
            condition => Self::Select(Box::new(condition), Box::new(then), Box::new(otherwise)),
        }
    }

    pub(super) fn any(terms: impl IntoIterator<Item = Self>) -> Self {
        let mut terms: Vec<_> = terms
            .into_iter()
            .flat_map(|term| match term {
                Self::Any(terms) => terms,
                term => vec![term],
            })
            .filter(|term| *term != Self::Bit(false))
            .collect();
        if terms.contains(&Self::Bit(true)) {
            return Self::Bit(true);
        }
        terms.sort();
        terms.dedup();
        match terms.len() {
            0 => Self::Bit(false),
            1 => terms.remove(0),
            _ => Self::Any(terms),
        }
    }

    pub(super) fn maximum(terms: &[Self]) -> Self {
        match terms {
            [] => Self::Strength(0),
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

    pub(super) fn visit_signals(&self, visit: &mut impl FnMut(usize, Moment)) {
        match self {
            Self::Bit(_) | Self::Strength(_) => {}
            Self::Signal(node, moment) => visit(*node, *moment),
            Self::Nonzero(term) | Self::Above(term, _) | Self::Attenuated(term, _) => {
                term.visit_signals(visit)
            }
            Self::Maximum(left, right) | Self::Differs(left, right) => {
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
        matches!(
            self,
            Self::Above(..) | Self::Select(..) | Self::Differs(..) | Self::Any(_)
        )
    }

    fn operand(&self) -> Operand<'_> {
        Operand(self)
    }
}

pub(super) fn signal_name(node: usize, moment: Moment) -> String {
    match moment {
        Moment::BeforeFiring => format!("signal_{node}"),
        Moment::AfterFiring => format!("next_signal_{node}"),
    }
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
            Self::Signal(node, moment) => formatter.write_str(&signal_name(*node, *moment)),
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
            Self::Differs(left, right) => {
                write!(formatter, "{} != {}", left.operand(), right.operand())
            }
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
