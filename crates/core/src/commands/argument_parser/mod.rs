mod coordinates;

use super::argument::{ArgParser, ClientArgument};
use super::suggestions::SuggestionSource;
use super::value::{Direction, DirectionWithDiagonals, PlayerTarget};
use crate::worldedit::mask::WorldEditMask;
use crate::worldedit::pattern::WorldEditPattern;
use mchprs_blocks::block_entities::ContainerType;
use mchprs_network::packets::clientbound::CDeclareCommandsNodeParser as ProtocolParser;
use mchprs_syntax::expect::Expect;
use mchprs_syntax::reader::{Reader, SyntaxError};

pub(super) use coordinates::{BlockPositionParser, PlotPositionParser, PositionParser};

const DIRECTIONS: &[(&[&str], Direction)] = &[
    (&["me"], Direction::Me),
    (&["l", "left"], Direction::Left),
    (&["r", "right"], Direction::Right),
    (&["u", "up"], Direction::Up),
    (&["d", "down"], Direction::Down),
    (&["n", "north"], Direction::North),
    (&["s", "south"], Direction::South),
    (&["e", "east"], Direction::East),
    (&["w", "west"], Direction::West),
];

const DIRECTIONS_WITH_DIAGONALS: &[(&[&str], DirectionWithDiagonals)] = &[
    (&["me"], DirectionWithDiagonals::Me),
    (&["l", "left"], DirectionWithDiagonals::Left),
    (&["r", "right"], DirectionWithDiagonals::Right),
    (&["u", "up"], DirectionWithDiagonals::Up),
    (&["d", "down"], DirectionWithDiagonals::Down),
    (&["n", "north"], DirectionWithDiagonals::North),
    (&["s", "south"], DirectionWithDiagonals::South),
    (&["e", "east"], DirectionWithDiagonals::East),
    (&["w", "west"], DirectionWithDiagonals::West),
    (&["lu", "leftup"], DirectionWithDiagonals::LeftUp),
    (&["ld", "leftdown"], DirectionWithDiagonals::LeftDown),
    (&["ru", "rightup"], DirectionWithDiagonals::RightUp),
    (&["rd", "rightdown"], DirectionWithDiagonals::RightDown),
    (&["nu", "northup"], DirectionWithDiagonals::NorthUp),
    (&["nd", "northdown"], DirectionWithDiagonals::NorthDown),
    (&["su", "southup"], DirectionWithDiagonals::SouthUp),
    (&["sd", "southdown"], DirectionWithDiagonals::SouthDown),
    (&["eu", "eastup"], DirectionWithDiagonals::EastUp),
    (&["ed", "eastdown"], DirectionWithDiagonals::EastDown),
    (&["wu", "westup"], DirectionWithDiagonals::WestUp),
    (&["wd", "westdown"], DirectionWithDiagonals::WestDown),
];

pub(super) struct WordParser;

impl ArgParser for WordParser {
    type Output = String;

    fn parse(&self, reader: &mut Reader<'_>, _: &mut Expect) -> Result<String, SyntaxError> {
        reader.word()
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::Client(ProtocolParser::String(0))
    }
}

pub(super) struct SchematicFileParser;

impl ArgParser for SchematicFileParser {
    type Output = String;

    fn parse(&self, reader: &mut Reader<'_>, expect: &mut Expect) -> Result<String, SyntaxError> {
        let start = reader.cursor();
        let name = reader.word();
        expect.complete_from(reader, start);
        name
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::AskServer(ProtocolParser::String(0))
    }

    fn source(&self) -> Option<SuggestionSource> {
        Some(SuggestionSource::SchematicFiles)
    }
}

pub(super) struct GreedyParser;

impl ArgParser for GreedyParser {
    type Output = String;

    fn parse(&self, reader: &mut Reader<'_>, _: &mut Expect) -> Result<String, SyntaxError> {
        Ok(reader.greedy())
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::Client(ProtocolParser::String(2))
    }

    fn consumes_remaining(&self) -> bool {
        true
    }
}

pub(super) struct IntegerParser {
    min: i32,
    max: i32,
}

impl IntegerParser {
    pub(super) const fn new(min: i32, max: i32) -> Self {
        Self { min, max }
    }
}

impl ArgParser for IntegerParser {
    type Output = i32;

    fn parse(&self, reader: &mut Reader<'_>, _: &mut Expect) -> Result<i32, SyntaxError> {
        reader.number(self.min, self.max)
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::Client(ProtocolParser::Integer(self.min, self.max))
    }
}

pub(super) struct FloatParser {
    min: f32,
    max: f32,
}

impl FloatParser {
    pub(super) const fn new(min: f32, max: f32) -> Self {
        Self { min, max }
    }
}

impl ArgParser for FloatParser {
    type Output = f32;

    fn parse(&self, reader: &mut Reader<'_>, _: &mut Expect) -> Result<f32, SyntaxError> {
        reader.number(self.min, self.max)
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::Client(ProtocolParser::Float(self.min, self.max))
    }
}

pub(super) struct PlayerParser;

impl ArgParser for PlayerParser {
    type Output = PlayerTarget;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<PlayerTarget, SyntaxError> {
        let start = reader.cursor();
        let token = reader.take_token();
        expect.offer(reader, start, || ["@s"]);
        if token == "@s" {
            return Ok(PlayerTarget::SelfPlayer);
        }
        if token.starts_with('@') {
            return Err(expect.fail_recognized(
                start,
                "Only @s, player names, and player UUIDs are supported",
            ));
        }
        if token.len() == 36
            && token.bytes().enumerate().all(|(i, ch)| {
                if [8, 13, 18, 23].contains(&i) {
                    ch == b'-'
                } else {
                    ch.is_ascii_hexdigit()
                }
            })
            && let Ok(uuid) = u128::from_str_radix(&token.replace('-', ""), 16)
        {
            return Ok(PlayerTarget::Uuid(uuid));
        }
        if token.is_empty()
            || token.len() > 16
            || !token
                .bytes()
                .all(|ch| ch.is_ascii_alphanumeric() || ch == b'_')
        {
            return Err(expect.fail(start, "Expected a player name, UUID, or @s"));
        }
        Ok(PlayerTarget::Name(token.to_owned()))
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::Opaque
    }

    fn source(&self) -> Option<SuggestionSource> {
        Some(SuggestionSource::PlayerNames)
    }
}

pub(super) struct DirectionParser;

impl ArgParser for DirectionParser {
    type Output = Direction;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<Direction, SyntaxError> {
        keyword(reader, expect, DIRECTIONS, "Expected a direction")
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::AskServer(ProtocolParser::String(0))
    }
}

pub(super) struct DiagonalDirectionParser;

impl ArgParser for DiagonalDirectionParser {
    type Output = DirectionWithDiagonals;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<DirectionWithDiagonals, SyntaxError> {
        keyword(
            reader,
            expect,
            DIRECTIONS_WITH_DIAGONALS,
            "Expected a direction",
        )
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::AskServer(ProtocolParser::String(0))
    }
}

pub(super) struct ContainerParser;

impl ArgParser for ContainerParser {
    type Output = ContainerType;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<ContainerType, SyntaxError> {
        let start = reader.cursor();
        let name = reader.take_while(Reader::allowed_word);
        expect.offer(reader, start, || {
            ContainerType::ALL.map(ContainerType::name)
        });
        name.parse()
            .map_err(|()| expect.fail(start, "Expected barrel, furnace, or hopper"))
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::AskServer(ProtocolParser::String(0))
    }
}

pub(super) struct PatternParser;

impl ArgParser for PatternParser {
    type Output = WorldEditPattern;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<WorldEditPattern, SyntaxError> {
        WorldEditPattern::parse(reader, expect)
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::Opaque
    }
}

pub(super) struct MaskParser;

impl ArgParser for MaskParser {
    type Output = WorldEditMask;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<WorldEditMask, SyntaxError> {
        WorldEditMask::parse(reader, expect)
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::Opaque
    }
}

fn keyword<T: Copy>(
    reader: &mut Reader<'_>,
    expect: &mut Expect,
    table: &[(&[&str], T)],
    expected: &str,
) -> Result<T, SyntaxError> {
    let start = reader.cursor();
    let name = reader.take_while(Reader::allowed_word);
    expect.offer(reader, start, || {
        table.iter().flat_map(|(names, _)| names.iter().copied())
    });
    table
        .iter()
        .find(|(names, _)| names.iter().any(|known| known.eq_ignore_ascii_case(name)))
        .map(|(_, value)| *value)
        .ok_or_else(|| expect.fail(start, expected))
}
