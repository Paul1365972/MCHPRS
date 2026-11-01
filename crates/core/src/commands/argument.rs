use super::suggestions::SuggestionSource;
use mchprs_network::packets::clientbound::CDeclareCommandsNodeParser as ProtocolParser;
use mchprs_syntax::expect::Expect;
use mchprs_syntax::flag::{self, Flag, FlagValues};
use mchprs_syntax::reader::{Reader, SyntaxError};
use std::any::Any;
use std::marker::PhantomData;
use std::sync::Arc;

#[derive(Clone, Copy)]
pub(super) enum ClientArgument {
    Client(ProtocolParser),
    AskServer(ProtocolParser),
    Opaque,
}

pub(super) trait ArgParser: Send + Sync + 'static {
    type Output: Clone + Send + Sync + 'static;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<Self::Output, SyntaxError>;

    fn client(&self) -> ClientArgument;

    fn consumes_remaining(&self) -> bool {
        false
    }

    fn source(&self) -> Option<SuggestionSource> {
        None
    }
}

type Value = Arc<dyn Any + Send + Sync>;

type ParseValue =
    Arc<dyn Fn(&mut Reader<'_>, &mut Expect) -> Result<Value, SyntaxError> + Send + Sync>;

#[derive(Clone)]
pub(super) enum Parameter {
    Value {
        slot: usize,
        parse: ParseValue,
        client: ClientArgument,
        consumes_remaining: bool,
        source: Option<SuggestionSource>,
    },
    Flags(Arc<[Flag]>),
}

impl Parameter {
    pub(super) fn value<P: ArgParser>(slot: usize, parser: P) -> Self {
        Self::Value {
            slot,
            consumes_remaining: parser.consumes_remaining(),
            source: parser.source(),
            client: parser.client(),
            parse: Arc::new(move |reader: &mut Reader<'_>, expect: &mut Expect| {
                Ok(Arc::new(parser.parse(reader, expect)?) as Value)
            }),
        }
    }

    pub(super) fn client(&self) -> ClientArgument {
        match self {
            Self::Value { client, .. } => *client,
            Self::Flags(_) => ClientArgument::AskServer(ProtocolParser::String(2)),
        }
    }

    pub(super) fn source(&self) -> Option<SuggestionSource> {
        match self {
            Self::Value { source, .. } => *source,
            Self::Flags(_) => None,
        }
    }

    pub(super) fn consumes_remaining(&self) -> bool {
        match self {
            Self::Value {
                consumes_remaining, ..
            } => *consumes_remaining,
            Self::Flags(_) => true,
        }
    }

    pub(super) fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<Parsed, SyntaxError> {
        match self {
            Self::Value { slot, parse, .. } => Ok(Parsed::Value {
                slot: *slot,
                value: parse(reader, expect)?,
            }),
            Self::Flags(flags) => flag::parse(reader, flags, expect).map(Parsed::Flags),
        }
    }
}

impl PartialEq for Parameter {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Value { slot, .. }, Self::Value { slot: other, .. }) => slot == other,
            (Self::Flags(flags), Self::Flags(other)) => flags == other,
            _ => false,
        }
    }
}

#[derive(Clone)]
pub(super) enum Parsed {
    Value { slot: usize, value: Value },
    Flags(FlagValues),
}

pub(super) struct Arguments {
    values: Vec<(usize, Value)>,
    flags: FlagValues,
}

impl Arguments {
    pub(super) fn new(parsed: impl IntoIterator<Item = Parsed>, mut flags: FlagValues) -> Self {
        let mut values = Vec::new();
        for parsed in parsed {
            match parsed {
                Parsed::Value { slot, value } => values.push((slot, value)),
                Parsed::Flags(given) => flags.extend(given),
            }
        }
        Self { values, flags }
    }

    fn value<T: Clone + 'static>(&self, slot: usize) -> Option<T> {
        let (_, value) = self.values.iter().find(|(parsed, _)| *parsed == slot)?;
        value.downcast_ref::<T>().cloned()
    }
}

pub(super) trait Handle: Copy {
    type Value;

    fn read(self, arguments: &Arguments) -> Self::Value;
}

pub(super) struct Required<T> {
    slot: usize,
    output: PhantomData<fn() -> T>,
}

impl<T> Required<T> {
    pub(super) fn new(slot: usize) -> Self {
        Self {
            slot,
            output: PhantomData,
        }
    }
}

impl<T> Clone for Required<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for Required<T> {}

impl<T: Clone + 'static> Handle for Required<T> {
    type Value = T;

    fn read(self, arguments: &Arguments) -> T {
        arguments
            .value(self.slot)
            .expect("a required argument is parsed on every path to its executor")
    }
}

pub(super) struct Optional<T> {
    slot: usize,
    output: PhantomData<fn() -> T>,
}

impl<T> Optional<T> {
    pub(super) fn new(slot: usize) -> Self {
        Self {
            slot,
            output: PhantomData,
        }
    }
}

impl<T> Clone for Optional<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for Optional<T> {}

impl<T: Clone + 'static> Handle for Optional<T> {
    type Value = Option<T>;

    fn read(self, arguments: &Arguments) -> Option<T> {
        arguments.value(self.slot)
    }
}

#[derive(Clone, Copy)]
pub(super) struct Switch(pub(super) Flag);

impl Handle for Switch {
    type Value = bool;

    fn read(self, arguments: &Arguments) -> bool {
        arguments.flags.is_on(&self.0)
    }
}

#[derive(Clone, Copy)]
pub(super) struct OneOf<T, const N: usize>(pub(super) [(Flag, T); N]);

impl<T: Copy, const N: usize> Handle for OneOf<T, N> {
    type Value = Option<T>;

    fn read(self, arguments: &Arguments) -> Option<T> {
        self.0
            .iter()
            .find(|(flag, _)| arguments.flags.is_on(flag))
            .map(|&(_, value)| value)
    }
}

#[derive(Clone, Copy)]
pub(super) struct AllFlags;

impl Handle for AllFlags {
    type Value = FlagValues;

    fn read(self, arguments: &Arguments) -> FlagValues {
        arguments.flags.clone()
    }
}
