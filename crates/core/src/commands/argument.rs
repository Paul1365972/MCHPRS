use super::suggestions::SuggestionSource;
use super::value::{BlockCoordinates, Value};
use super::{argument_parser, argument_suggestions};
use crate::worldedit::mask::WorldEditMask;
use crate::worldedit::pattern::WorldEditPattern;
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
pub(super) enum ArgumentType {
    Word,
    Integer { min: i32, max: i32 },
    Float { min: f32, max: f32 },
    PlayerTarget,
    Direction,
    Vec3,
    PlotPos,
    ContainerType,
    Pattern,
    Mask,
    DirectionWithDiagonals,
    BlockPos,
    SchematicFile,
    GreedyString,
    Flags { flags: Vec<Flag> },
}

impl ArgumentType {
    pub(super) fn integer(min: i32, max: i32) -> Self {
        Self::Integer { min, max }
    }

    pub(super) fn float(min: f32, max: f32) -> Self {
        Self::Float { min, max }
    }

    pub(super) fn client_argument(&self) -> ClientArgument {
        match self {
            Self::Integer { min, max } => ClientArgument::Client(Parser::Integer(*min, *max)),
            Self::Float { min, max } => ClientArgument::Client(Parser::Float(*min, *max)),
            Self::Word => ClientArgument::Client(Parser::String(0)),
            Self::GreedyString => ClientArgument::Client(Parser::String(2)),
            Self::Vec3 => ClientArgument::Client(Parser::Vec3),
            Self::BlockPos => ClientArgument::Client(Parser::BlockPos),
            Self::Flags { .. } => ClientArgument::AskServer(Parser::String(2)),
            Self::Direction
            | Self::DirectionWithDiagonals
            | Self::ContainerType
            | Self::SchematicFile => ClientArgument::AskServer(Parser::String(0)),
            Self::PlotPos => ClientArgument::AskServer(Parser::ColumnPos),
            Self::Pattern | Self::Mask | Self::PlayerTarget => ClientArgument::Opaque,
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
            Self::Flags { flags } => argument_suggestions::suggest_flags(flags, builder),
            Self::Pattern => {
                argument_suggestions::suggest_quoted(self, builder, WorldEditPattern::suggest)
            }
            Self::Mask => {
                argument_suggestions::suggest_quoted(self, builder, WorldEditMask::suggest)
            }
            Self::Vec3 | Self::BlockPos => {
                argument_suggestions::suggest_coordinates(self, &["~ ~ ~", "^ ^ ^"], builder)
            }
            Self::PlotPos => argument_suggestions::suggest_coordinates(self, &["~ ~"], builder),
            Self::SchematicFile => return Some(SuggestionSource::SchematicFiles),
            Self::Word | Self::GreedyString | Self::Integer { .. } | Self::Float { .. } => {}
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
            Self::Word | Self::SchematicFile => Ok(Value::String(reader.word())),
            Self::PlayerTarget => argument_parser::parse_player(reader),
            Self::Integer { min, max } => reader.number(*min, *max).map(Value::Integer),
            Self::Float { min, max } => reader.number(*min, *max).map(Value::Float),
            Self::Direction => argument_parser::parse_direction(reader),
            Self::Vec3 => argument_parser::parse_vec3(reader, true).map(Value::Position),
            Self::PlotPos => argument_parser::parse_plot_pos(reader),
            Self::ContainerType => argument_parser::parse_container(reader),
            Self::Pattern => argument_parser::parse_pattern(reader),
            Self::Mask => argument_parser::parse_mask(reader),
            Self::DirectionWithDiagonals => argument_parser::parse_direction_with_diagonals(reader),
            Self::BlockPos => argument_parser::parse_vec3(reader, false)
                .map(|position| Value::BlockPos(BlockCoordinates(position))),
            Self::GreedyString => Ok(Value::String(reader.greedy())),
            Self::Flags { flags } => flag::parse(reader, flags).map(Value::Flags),
        }
    }
}
