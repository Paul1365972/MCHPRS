use super::error::{CommandError, CommandResult};
use mchprs_blocks::{BlockFacing, BlockPos};
use std::array;
use std::f64::consts::FRAC_PI_2;

#[derive(Debug, Clone, Copy)]
pub(super) enum Coordinate<T> {
    Absolute(T),
    Relative(T),
}

impl Coordinate<i32> {
    pub(super) fn resolve(&self, reference: i32) -> CommandResult<i32> {
        match self {
            Self::Absolute(value) => Some(*value),
            Self::Relative(offset) => reference.checked_add(*offset),
        }
        .ok_or_else(CommandError::coordinate_out_of_range)
    }
}

impl Coordinate<f64> {
    pub(super) fn resolve(&self, reference: f64) -> f64 {
        match self {
            Self::Absolute(value) => *value,
            Self::Relative(offset) => reference + *offset,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) enum Direction {
    Me,
    Left,
    Right,
    Up,
    Down,
    North,
    South,
    East,
    West,
}

impl Direction {
    pub(super) fn resolve(&self, player_facing: BlockFacing) -> BlockFacing {
        match self {
            Self::Up => BlockFacing::Up,
            Self::Down => BlockFacing::Down,
            Self::North => BlockFacing::North,
            Self::South => BlockFacing::South,
            Self::East => BlockFacing::East,
            Self::West => BlockFacing::West,
            Self::Me => player_facing,
            Self::Left => player_facing.rotate_ccw(),
            Self::Right => player_facing.rotate(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) enum DirectionWithDiagonals {
    Me,
    Left,
    Right,
    Up,
    Down,
    North,
    South,
    East,
    West,
    LeftUp,
    LeftDown,
    RightUp,
    RightDown,
    NorthUp,
    NorthDown,
    SouthUp,
    SouthDown,
    EastUp,
    EastDown,
    WestUp,
    WestDown,
}

impl DirectionWithDiagonals {
    pub(super) fn resolve(&self, player_facing: BlockFacing, player_pitch: f32) -> BlockPos {
        match self {
            Self::Up => BlockPos::new(0, 1, 0),
            Self::Down => BlockPos::new(0, -1, 0),
            Self::North => BlockPos::new(0, 0, -1),
            Self::South => BlockPos::new(0, 0, 1),
            Self::East => BlockPos::new(1, 0, 0),
            Self::West => BlockPos::new(-1, 0, 0),
            Self::NorthUp => BlockPos::new(0, 1, -1),
            Self::NorthDown => BlockPos::new(0, -1, -1),
            Self::SouthUp => BlockPos::new(0, 1, 1),
            Self::SouthDown => BlockPos::new(0, -1, 1),
            Self::EastUp => BlockPos::new(1, 1, 0),
            Self::EastDown => BlockPos::new(1, -1, 0),
            Self::WestUp => BlockPos::new(-1, 1, 0),
            Self::WestDown => BlockPos::new(-1, -1, 0),
            Self::Me => {
                let mut offset = player_facing.offset_pos(BlockPos::zero(), 1);
                if !matches!(player_facing, BlockFacing::Down | BlockFacing::Up) {
                    if player_pitch > 22.5 {
                        offset.y -= 1;
                    } else if player_pitch < -22.5 {
                        offset.y += 1;
                    }
                }
                offset
            }
            Self::Left => player_facing.rotate_ccw().offset_pos(BlockPos::zero(), 1),
            Self::Right => player_facing.rotate().offset_pos(BlockPos::zero(), 1),
            Self::LeftUp => player_facing
                .rotate_ccw()
                .offset_pos(BlockPos::new(0, 1, 0), 1),
            Self::LeftDown => player_facing
                .rotate_ccw()
                .offset_pos(BlockPos::new(0, -1, 0), 1),
            Self::RightUp => player_facing.rotate().offset_pos(BlockPos::new(0, 1, 0), 1),
            Self::RightDown => player_facing
                .rotate()
                .offset_pos(BlockPos::new(0, -1, 0), 1),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) enum PositionCoordinates {
    WorldAxes {
        x: Coordinate<f64>,
        y: Coordinate<f64>,
        z: Coordinate<f64>,
    },
    ViewAxes {
        left: f64,
        up: f64,
        forward: f64,
    },
}

impl PositionCoordinates {
    pub(super) fn resolve(
        &self,
        reference: (f64, f64, f64),
        yaw: f32,
        pitch: f32,
    ) -> CommandResult<(f64, f64, f64)> {
        let [x, y, z] = match self {
            Self::WorldAxes { x, y, z } => [
                x.resolve(reference.0),
                y.resolve(reference.1),
                z.resolve(reference.2),
            ],
            Self::ViewAxes { left, up, forward } => {
                let yaw = (yaw as f64 + 90.0).to_radians();
                let pitch = (-pitch as f64).to_radians();
                let forward_axis = [
                    yaw.cos() * pitch.cos(),
                    pitch.sin(),
                    yaw.sin() * pitch.cos(),
                ];
                let up_pitch = pitch + FRAC_PI_2;
                let up_axis = [
                    yaw.cos() * up_pitch.cos(),
                    up_pitch.sin(),
                    yaw.sin() * up_pitch.cos(),
                ];
                let left_axis = [
                    forward_axis[2] * up_axis[1] - forward_axis[1] * up_axis[2],
                    forward_axis[0] * up_axis[2] - forward_axis[2] * up_axis[0],
                    forward_axis[1] * up_axis[0] - forward_axis[0] * up_axis[1],
                ];
                let origin = [reference.0, reference.1, reference.2];
                array::from_fn(|i| {
                    origin[i] + left * left_axis[i] + up * up_axis[i] + forward * forward_axis[i]
                })
            }
        };
        if [x, y, z].into_iter().all(f64::is_finite) {
            Ok((x, y, z))
        } else {
            Err(CommandError::coordinate_out_of_range())
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) struct PlotCoordinates {
    pub(super) x: Coordinate<i32>,
    pub(super) z: Coordinate<i32>,
}

impl PlotCoordinates {
    pub(super) fn resolve(&self, reference: (i32, i32)) -> CommandResult<(i32, i32)> {
        Ok((self.x.resolve(reference.0)?, self.z.resolve(reference.1)?))
    }
}

#[derive(Debug, Clone)]
pub enum PlayerTarget {
    Name(String),
    Uuid(u128),
    SelfPlayer,
}
