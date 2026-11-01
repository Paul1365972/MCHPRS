use super::expression::{signal_name, Expression};
use crate::registers::{Field, NodeLayout};
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct StatesSlice {
    pub(super) offset: usize,
    pub(super) width: usize,
}

impl fmt::Display for StatesSlice {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.width {
            1 => write!(formatter, "states[{}]", self.offset),
            width => write!(formatter, "states[{} +: {width}]", self.offset),
        }
    }
}

pub(super) enum Connection {
    Expression(Expression),
    Net(&'static str),
    Signal(usize),
    States(StatesSlice),
}

impl From<Expression> for Connection {
    fn from(expression: Expression) -> Self {
        Self::Expression(expression)
    }
}

impl From<&'static str> for Connection {
    fn from(net: &'static str) -> Self {
        Self::Net(net)
    }
}

impl From<StatesSlice> for Connection {
    fn from(slice: StatesSlice) -> Self {
        Self::States(slice)
    }
}

impl fmt::Display for Connection {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Expression(expression) => write!(formatter, "{expression}"),
            Self::Net(net) => formatter.write_str(net),
            Self::Signal(node) => formatter.write_str(&signal_name(*node)),
            Self::States(slice) => write!(formatter, "{slice}"),
        }
    }
}

pub(super) struct Instance {
    module: &'static str,
    index: usize,
    parameters: Vec<(&'static str, u32)>,
    ports: Vec<(&'static str, Connection)>,
}

impl Instance {
    pub(super) fn new(module: &'static str, index: usize) -> Self {
        Self {
            module,
            index,
            parameters: Vec::new(),
            ports: Vec::new(),
        }
    }

    /// Node module parameters default to zero, so zero values are left out.
    pub(super) fn parameter(&mut self, name: &'static str, value: impl Into<u32>) {
        let value = value.into();
        if value != 0 {
            self.parameters.push((name, value));
        }
    }

    pub(super) fn port(&mut self, name: &'static str, connection: impl Into<Connection>) {
        self.ports.push((name, connection.into()));
    }

    pub(super) fn index(&self) -> usize {
        self.index
    }

    pub(super) fn visit_signals(&self, visit: &mut impl FnMut(usize)) {
        for (_, connection) in &self.ports {
            if let Connection::Expression(expression) = connection {
                expression.visit_signals(visit);
            }
        }
    }

    pub(super) fn signal(&mut self) {
        self.port("next_signal", Connection::Signal(self.index));
    }

    pub(super) fn readback(&mut self, layout: &NodeLayout) {
        for (field, offset) in layout.field_offsets() {
            let width = field.width();
            self.port(field_port(field), StatesSlice { offset, width });
        }
    }
}

impl fmt::Display for Instance {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.module)?;
        for (position, (name, value)) in self.parameters.iter().enumerate() {
            let separator = if position == 0 { " #(" } else { ", " };
            write!(formatter, "{separator}.{name}({value})")?;
        }
        if !self.parameters.is_empty() {
            formatter.write_str(")")?;
        }
        write!(formatter, " node_{} (", self.index)?;
        for (position, (name, connection)) in self.ports.iter().enumerate() {
            if position != 0 {
                formatter.write_str(", ")?;
            }
            write!(formatter, ".{name}({connection})")?;
        }
        formatter.write_str(");")
    }
}

fn field_port(field: Field) -> &'static str {
    match field {
        Field::Powered => "powered",
        Field::High { .. } => "high",
        Field::Strength => "strength",
        Field::Locked => "locked",
        Field::Countdown(_) => "countdown",
        Field::PulseCountdown(_) => "pulse_countdown",
    }
}
