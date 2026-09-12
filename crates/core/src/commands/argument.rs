use super::{argument_parser, argument_suggestions, suggestions::SuggestionSource, value::Value};
use mchprs_commands::flag::{self, Flag};
use mchprs_commands::graph::Argument;
use mchprs_commands::reader::{Reader, SyntaxError};
use mchprs_commands::suggestion::SuggestionsBuilder;
use mchprs_network::packets::clientbound::CDeclareCommandsNodeParser as Parser;

pub(super) enum ClientArgument {
    Client(Parser),
    AskServer(Parser),
    Opaque,
}

#[derive(Clone, PartialEq)]
pub enum ArgumentType {
    Word,
    Integer { min: i32, max: i32 },
    Float { min: f32, max: f32 },
    PlayerTarget,
    Direction,
    Vec3,
    PlotPos,
    ContainerType,
    Pattern,
    DirectionWithDiagonals,
    SchematicFile,
    GreedyString,
    Flags { flags: Vec<Flag> },
}

impl ArgumentType {
    pub fn integer(min: i32, max: i32) -> Self {
        ArgumentType::Integer { min, max }
    }

    pub fn float(min: f32, max: f32) -> Self {
        ArgumentType::Float { min, max }
    }

    pub(super) fn client_argument(&self) -> ClientArgument {
        match self {
            Self::Integer { min, max } => ClientArgument::Client(Parser::Integer(*min, *max)),
            Self::Float { min, max } => ClientArgument::Client(Parser::Float(*min, *max)),
            Self::Word => ClientArgument::Client(Parser::String(0)),
            Self::GreedyString => ClientArgument::Client(Parser::String(2)),
            Self::Vec3 => ClientArgument::Client(Parser::Vec3),
            Self::Flags { .. } => ClientArgument::AskServer(Parser::String(2)),
            Self::Direction
            | Self::DirectionWithDiagonals
            | Self::ContainerType
            | Self::SchematicFile => ClientArgument::AskServer(Parser::String(0)),
            Self::PlotPos => ClientArgument::AskServer(Parser::ColumnPos),
            Self::Pattern | Self::PlayerTarget => ClientArgument::Opaque,
        }
    }

    pub(super) fn suggest(
        &self,
        builder: &mut SuggestionsBuilder<'_, '_>,
    ) -> Option<SuggestionSource> {
        match self {
            Self::Direction => builder.suggest_matching(
                argument_parser::DIRECTIONS
                    .iter()
                    .flat_map(|(names, _)| *names),
            ),
            Self::DirectionWithDiagonals => builder.suggest_matching(
                argument_parser::DIRECTIONS_WITH_DIAGONALS
                    .iter()
                    .flat_map(|(names, _)| *names),
            ),
            Self::ContainerType => builder.suggest_matching(
                argument_parser::CONTAINER_TYPES
                    .iter()
                    .map(|(name, _)| name),
            ),
            Self::PlayerTarget => {
                builder.suggest_matching(["@s"]);
                return Some(SuggestionSource::PlayerNames);
            }
            Self::Flags { flags } => argument_suggestions::flags(flags, builder),
            Self::Vec3 => argument_suggestions::coordinates(self, &["~ ~ ~", "^ ^ ^"], builder),
            Self::PlotPos => argument_suggestions::coordinates(self, &["~ ~"], builder),
            Self::SchematicFile => return Some(SuggestionSource::SchematicFiles),
            Self::Word
            | Self::GreedyString
            | Self::Integer { .. }
            | Self::Float { .. }
            | Self::Pattern => {}
        }
        None
    }
}

impl Argument for ArgumentType {
    type Value = Value;

    fn consumes_remaining(&self) -> bool {
        matches!(self, Self::GreedyString | Self::Flags { .. })
    }

    fn parse(&self, reader: &mut Reader<'_>) -> Result<Value, SyntaxError> {
        match self {
            ArgumentType::Word | ArgumentType::SchematicFile => Ok(Value::String(reader.word())),
            ArgumentType::PlayerTarget => argument_parser::parse_player(reader),
            ArgumentType::Integer { min, max } => reader.number(*min, *max).map(Value::Integer),
            ArgumentType::Float { min, max } => reader.number(*min, *max).map(Value::Float),
            ArgumentType::Direction => argument_parser::parse_direction(reader),
            ArgumentType::Vec3 => argument_parser::parse_vec3(reader).map(Value::Position),
            ArgumentType::PlotPos => argument_parser::parse_plot_pos(reader),
            ArgumentType::ContainerType => argument_parser::parse_container(reader),
            ArgumentType::Pattern => argument_parser::parse_pattern(reader),
            ArgumentType::DirectionWithDiagonals => {
                argument_parser::parse_direction_with_diagonals(reader)
            }
            ArgumentType::GreedyString => Ok(Value::String(reader.greedy())),
            ArgumentType::Flags { flags } => flag::parse(reader, flags).map(Value::Flags),
        }
    }
}
