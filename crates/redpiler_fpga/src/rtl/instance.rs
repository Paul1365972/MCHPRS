use super::expression::{signal_name, Expression, Moment};
use crate::registers::Field;
use std::fmt;

pub(super) enum Connection {
    Expression(Expression),
    Net(String),
}

impl From<Expression> for Connection {
    fn from(expression: Expression) -> Self {
        Self::Expression(expression)
    }
}

impl From<&str> for Connection {
    fn from(net: &str) -> Self {
        Self::Net(net.to_owned())
    }
}

impl From<String> for Connection {
    fn from(net: String) -> Self {
        Self::Net(net)
    }
}

impl fmt::Display for Connection {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Expression(expression) => write!(formatter, "{expression}"),
            Self::Net(net) => formatter.write_str(net),
        }
    }
}

pub(super) struct Instance {
    module: &'static str,
    index: usize,
    parameters: Vec<(&'static str, String)>,
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
            self.parameters.push((name, value.to_string()));
        }
    }

    pub(super) fn port(&mut self, name: &'static str, connection: impl Into<Connection>) {
        self.ports.push((name, connection.into()));
    }

    pub(super) fn index(&self) -> usize {
        self.index
    }

    pub(super) fn visit_signals(&self, visit: &mut impl FnMut(usize, Moment)) {
        for (_, connection) in &self.ports {
            if let Connection::Expression(expression) = connection {
                expression.visit_signals(visit);
            }
        }
    }

    pub(super) fn signal(&mut self, moment: Moment) {
        let port = match moment {
            Moment::BeforeFiring => "signal",
            Moment::AfterFiring => "next_signal",
        };
        self.port(port, signal_name(self.index, moment));
    }

    pub(super) fn readback(&mut self, fields: &[Field], offset: usize) {
        let mut offset = offset;
        for &field in fields {
            let width = field.width() as usize;
            let slice = match width {
                1 => format!("states[{offset}]"),
                width => format!("states[{offset} +: {width}]"),
            };
            self.port(field_port(field), slice);
            offset += width;
        }
    }
}

impl fmt::Display for Instance {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.module)?;
        if !self.parameters.is_empty() {
            let parameters: Vec<_> = self
                .parameters
                .iter()
                .map(|(name, value)| format!(".{name}({value})"))
                .collect();
            write!(formatter, " #({})", parameters.join(", "))?;
        }
        let ports: Vec<_> = self
            .ports
            .iter()
            .map(|(name, connection)| format!(".{name}({connection})"))
            .collect();
        write!(formatter, " node_{} ({});", self.index, ports.join(", "))
    }
}

fn field_port(field: Field) -> &'static str {
    match field {
        Field::Powered => "powered",
        Field::High => "high",
        Field::Strength => "strength",
        Field::Locked => "locked",
        Field::Countdown(_) => "countdown",
        Field::PulseCountdown(_) => "pulse_countdown",
    }
}
