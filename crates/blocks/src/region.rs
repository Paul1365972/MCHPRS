use crate::BlockPos;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Region {
    min: BlockPos,
    max: BlockPos,
}

impl Region {
    pub fn new(first: BlockPos, second: BlockPos) -> Region {
        Region {
            min: first.min(second),
            max: first.max(second),
        }
    }

    pub fn min(self) -> BlockPos {
        self.min
    }

    pub fn max(self) -> BlockPos {
        self.max
    }

    pub fn lengths(self) -> [usize; 3] {
        let length = |min: i32, max: i32| {
            usize::try_from(i64::from(max) - i64::from(min) + 1).unwrap_or_default()
        };
        [
            length(self.min.x, self.max.x),
            length(self.min.y, self.max.y),
            length(self.min.z, self.max.z),
        ]
    }

    pub fn volume(self) -> usize {
        self.lengths().into_iter().product()
    }

    pub fn contains(self, pos: BlockPos) -> bool {
        (self.min.x..=self.max.x).contains(&pos.x)
            && (self.min.y..=self.max.y).contains(&pos.y)
            && (self.min.z..=self.max.z).contains(&pos.z)
    }

    pub fn intersection(self, other: Region) -> Option<Region> {
        let min = self.min.max(other.min);
        let max = self.max.min(other.max);
        (min.x <= max.x && min.y <= max.y && min.z <= max.z).then_some(Region { min, max })
    }

    pub fn translated(self, offset: BlockPos) -> Region {
        Region {
            min: self.min + offset,
            max: self.max + offset,
        }
    }

    pub fn positions(self) -> impl Iterator<Item = BlockPos> {
        (self.min.x..=self.max.x).flat_map(move |x| {
            (self.min.y..=self.max.y)
                .flat_map(move |y| (self.min.z..=self.max.z).map(move |z| BlockPos::new(x, y, z)))
        })
    }
}
