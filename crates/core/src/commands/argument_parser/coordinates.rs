use crate::commands::argument::{ArgParser, ClientArgument};
use crate::commands::value::{Coordinate, PlotCoordinates, PositionCoordinates};
use mchprs_network::packets::clientbound::CDeclareCommandsNodeParser as ProtocolParser;
use mchprs_syntax::expect::Expect;
use mchprs_syntax::reader::{Reader, SyntaxError};
use std::iter;

pub(in crate::commands) struct PositionParser;

impl ArgParser for PositionParser {
    type Output = PositionCoordinates;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        _: &mut Expect,
    ) -> Result<PositionCoordinates, SyntaxError> {
        parse_vec3(reader, true)
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::Client(ProtocolParser::Vec3)
    }
}

pub(in crate::commands) struct PlotPositionParser;

impl ArgParser for PlotPositionParser {
    type Output = PlotCoordinates;

    fn parse(
        &self,
        reader: &mut Reader<'_>,
        expect: &mut Expect,
    ) -> Result<PlotCoordinates, SyntaxError> {
        let start = reader.cursor();
        let offer_template = |reader: &Reader<'_>, expect: &mut Expect| {
            expect.offer(reader, start, || {
                let entered = reader.input()[start..].split_whitespace();
                [entered
                    .chain(iter::repeat("~"))
                    .take(2)
                    .collect::<Vec<_>>()
                    .join(" ")]
            });
        };
        let mut coordinates = [Coordinate::Absolute(0); 2];
        for (i, coordinate) in coordinates.iter_mut().enumerate() {
            let separated = if i == 0 { Ok(()) } else { separator(reader, 2) };
            offer_template(reader, expect);
            separated?;
            let relative = reader.peek() == Some('~');
            if relative {
                reader.read();
            }
            let value = if relative && reader.peek().is_none_or(|ch| ch == ' ') {
                0
            } else {
                reader.number(i32::MIN, i32::MAX)?
            };
            *coordinate = if relative {
                Coordinate::Relative(value)
            } else {
                Coordinate::Absolute(value)
            };
        }
        offer_template(reader, expect);
        Ok(PlotCoordinates {
            x: coordinates[0],
            z: coordinates[1],
        })
    }

    fn client(&self) -> ClientArgument {
        ClientArgument::AskServer(ProtocolParser::ColumnPos)
    }
}

fn parse_vec3(reader: &mut Reader<'_>, center: bool) -> Result<PositionCoordinates, SyntaxError> {
    if reader.peek() == Some('^') {
        let mut local = [0.0; 3];
        for (i, value) in local.iter_mut().enumerate() {
            if i != 0 {
                separator(reader, 3)?;
            }
            if reader.read() != Some('^') {
                return Err(reader.error("Cannot mix local and world coordinates"));
            }
            if reader.peek().is_some_and(|ch| ch != ' ') {
                *value = reader.number(-f64::MAX, f64::MAX)?;
            }
        }
        let [left, up, forward] = local;
        return Ok(PositionCoordinates::ViewAxes { left, up, forward });
    }

    let mut coordinates = [Coordinate::Absolute(0.0); 3];
    for (i, coordinate) in coordinates.iter_mut().enumerate() {
        if i != 0 {
            separator(reader, 3)?;
        }
        let relative = reader.peek() == Some('~');
        if relative {
            reader.read();
        }
        let start = reader.cursor();
        let value = if relative && reader.peek().is_none_or(|ch| ch == ' ') {
            0.0
        } else if !center && !relative {
            reader.number(i32::MIN, i32::MAX)? as f64
        } else {
            reader.number(-f64::MAX, f64::MAX)?
        };
        let integer = !reader.input()[start..reader.cursor()].contains('.');
        *coordinate = if relative {
            Coordinate::Relative(value)
        } else {
            Coordinate::Absolute(
                value
                    + if center && i != 1 && integer {
                        0.5
                    } else {
                        0.0
                    },
            )
        };
    }
    let [x, y, z] = coordinates;
    Ok(PositionCoordinates::WorldAxes { x, y, z })
}

fn separator(reader: &mut Reader<'_>, count: usize) -> Result<(), SyntaxError> {
    if reader.peek().is_some_and(|ch| ch != ' ') {
        return Err(reader.error("Expected whitespace between coordinates"));
    }
    if reader.read().is_none() || !reader.can_read() {
        return Err(reader.error(format!("Incomplete (expected {count} coordinates)")));
    }
    Ok(())
}
