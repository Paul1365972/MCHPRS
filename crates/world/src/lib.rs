pub mod storage;

use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use storage::Chunk;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct ChunkSectionIdx {
    x: i32,
    y: i32,
    z: i32,
}

impl ChunkSectionIdx {
    fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    fn region(self) -> Region {
        let min = BlockPos::new(self.x * 16, self.y * 16, self.z * 16);
        Region::new(min, min + BlockPos::new(15, 15, 15))
    }
}

/// Returns an iterator over the chunk section indexes between two block positions,
/// ie, over x,y,z triples (where x,z are chunk indexes and y are section indexes)
/// such that the bounding box defined by the two block positions intersect
/// section y of the x,z chunk.
///
/// The iterator yields the chunks in x,z order, and the sections in y order,
/// i.e., if visiting sections 1,2 of chunks 0,0 and 0,1, the iterator will yield
/// - section 1 of chunk 0,0 (0,1,0)
/// - section 2 of chunk 0,0 (0,1,1)
/// - section 1 of chunk 0,1 (0,2,0)
/// - section 2 of chunk 0,1 (0,2,1)
///
/// The iterator will not yield any chunks or sections
/// that are entirely outside the bounding box.
fn chunk_section_idxs_between(
    fst: BlockPos,
    snd: BlockPos,
) -> impl Iterator<Item = ChunkSectionIdx> {
    // for each coordinate, we find the start and end value based on the relative
    // position of the two blocks, and then divide by 16 to get the chunk/section index
    let start_x = i32::min(fst.x, snd.x).div_euclid(16);
    let end_x = i32::max(fst.x, snd.x).div_euclid(16);

    let start_y = i32::min(fst.y, snd.y).div_euclid(16);
    let end_y = i32::max(fst.y, snd.y).div_euclid(16);

    let start_z = i32::min(fst.z, snd.z).div_euclid(16);
    let end_z = i32::max(fst.z, snd.z).div_euclid(16);

    // it's possible that putting y in the final flat_map would let the compiler optimise the
    // iterations better (after inlining the iterator loop) if we then do operations
    // on the chunk that don't depend on the sections... should be tested
    (start_x..=end_x)
        .flat_map(move |x| (start_z..=end_z).map(move |z| (x, z)))
        .flat_map(move |(x, z)| (start_y..=end_y).map(move |y| ChunkSectionIdx::new(x, y, z)))
}

pub fn occupied_regions<'a>(
    region: Region,
    chunk_at: impl Fn(i32, i32) -> Option<&'a Chunk>,
) -> Vec<Region> {
    chunk_section_idxs_between(region.min(), region.max())
        .filter(|idx| {
            chunk_at(idx.x, idx.z)
                .and_then(|chunk| chunk.sections.get(usize::try_from(idx.y).ok()?))
                .is_some_and(|section| section.block_count() > 0)
        })
        .filter_map(|idx| idx.region().intersection(region))
        .collect()
}

#[cfg(test)]
mod test {
    use std::collections::HashMap;

    use super::*;

    fn positions_in_section(
        fst: BlockPos,
        snd: BlockPos,
        chunk_section_idx: ChunkSectionIdx,
    ) -> Vec<BlockPos> {
        chunk_section_idx
            .region()
            .intersection(Region::new(fst, snd))
            .into_iter()
            .flat_map(Region::positions)
            .collect()
    }

    fn visit_occupied(chunks: &[Chunk], fst: BlockPos, snd: BlockPos) -> HashMap<BlockPos, usize> {
        let chunk_at = |x, z| chunks.iter().find(|chunk| chunk.x == x && chunk.z == z);
        let mut visited = HashMap::new();
        for pos in occupied_regions(Region::new(fst, snd), chunk_at)
            .into_iter()
            .flat_map(Region::positions)
        {
            *visited.entry(pos).or_insert(0) += 1;
        }
        visited
    }

    fn set_block(chunks: &mut [Chunk], pos: BlockPos, block: u32) {
        let chunk = chunks
            .iter_mut()
            .find(|chunk| chunk.x == pos.x.div_euclid(16) && chunk.z == pos.z.div_euclid(16))
            .unwrap();
        chunk.set_block(
            pos.x.rem_euclid(16) as u32,
            pos.y as u32,
            pos.z.rem_euclid(16) as u32,
            block,
        );
    }

    #[test]
    fn test_chunk_section_idxs_between_simple() {
        let fst = BlockPos::new(0, 0, 0);
        let snd = BlockPos::new(31, 31, 31);
        let mut result = chunk_section_idxs_between(fst, snd).collect::<Vec<_>>();
        result.sort_by_key(|pos| (pos.x, pos.y, pos.z));
        let expected = vec![
            ChunkSectionIdx::new(0, 0, 0),
            ChunkSectionIdx::new(0, 0, 1),
            ChunkSectionIdx::new(0, 1, 0),
            ChunkSectionIdx::new(0, 1, 1),
            ChunkSectionIdx::new(1, 0, 0),
            ChunkSectionIdx::new(1, 0, 1),
            ChunkSectionIdx::new(1, 1, 0),
            ChunkSectionIdx::new(1, 1, 1),
        ];
        assert_eq!(result, expected);
    }

    #[test]
    fn test_chunk_section_idxs_between_negative() {
        let fst = BlockPos::new(-16, 0, -16);
        let snd = BlockPos::new(31, 31, 31);
        let mut result = chunk_section_idxs_between(fst, snd).collect::<Vec<_>>();
        result.sort_by_key(|pos| (pos.x, pos.y, pos.z));
        let expected = vec![
            ChunkSectionIdx::new(-1, 0, -1),
            ChunkSectionIdx::new(-1, 0, 0),
            ChunkSectionIdx::new(-1, 0, 1),
            ChunkSectionIdx::new(-1, 1, -1),
            ChunkSectionIdx::new(-1, 1, 0),
            ChunkSectionIdx::new(-1, 1, 1),
            ChunkSectionIdx::new(0, 0, -1),
            ChunkSectionIdx::new(0, 0, 0),
            ChunkSectionIdx::new(0, 0, 1),
            ChunkSectionIdx::new(0, 1, -1),
            ChunkSectionIdx::new(0, 1, 0),
            ChunkSectionIdx::new(0, 1, 1),
            ChunkSectionIdx::new(1, 0, -1),
            ChunkSectionIdx::new(1, 0, 0),
            ChunkSectionIdx::new(1, 0, 1),
            ChunkSectionIdx::new(1, 1, -1),
            ChunkSectionIdx::new(1, 1, 0),
            ChunkSectionIdx::new(1, 1, 1),
        ];
        assert_eq!(result, expected);
    }

    #[test]
    fn test_chunk_section_idxs_between_unordered() {
        let fst = BlockPos::new(31, 15, 31);
        let snd = BlockPos::new(0, 0, 0);
        let mut result = chunk_section_idxs_between(fst, snd).collect::<Vec<_>>();
        result.sort_by_key(|pos| (pos.x, pos.y, pos.z));
        let expected = vec![
            ChunkSectionIdx::new(0, 0, 0),
            ChunkSectionIdx::new(0, 0, 1),
            ChunkSectionIdx::new(1, 0, 0),
            ChunkSectionIdx::new(1, 0, 1),
        ];
        assert_eq!(result, expected);
    }

    #[test]
    fn test_chunk_section_idxs_misaligned() {
        let fst = BlockPos::new(-5, 20, 20);
        let snd = BlockPos::new(20, 35, -5);
        let mut result = chunk_section_idxs_between(fst, snd).collect::<Vec<_>>();
        result.sort_by_key(|pos| (pos.x, pos.y, pos.z));
        let expected = vec![
            ChunkSectionIdx::new(-1, 1, -1),
            ChunkSectionIdx::new(-1, 1, 0),
            ChunkSectionIdx::new(-1, 1, 1),
            ChunkSectionIdx::new(-1, 2, -1),
            ChunkSectionIdx::new(-1, 2, 0),
            ChunkSectionIdx::new(-1, 2, 1),
            ChunkSectionIdx::new(0, 1, -1),
            ChunkSectionIdx::new(0, 1, 0),
            ChunkSectionIdx::new(0, 1, 1),
            ChunkSectionIdx::new(0, 2, -1),
            ChunkSectionIdx::new(0, 2, 0),
            ChunkSectionIdx::new(0, 2, 1),
            ChunkSectionIdx::new(1, 1, -1),
            ChunkSectionIdx::new(1, 1, 0),
            ChunkSectionIdx::new(1, 1, 1),
            ChunkSectionIdx::new(1, 2, -1),
            ChunkSectionIdx::new(1, 2, 0),
            ChunkSectionIdx::new(1, 2, 1),
        ];
        assert_eq!(result, expected);
    }

    #[test]
    fn test_positions_in_section_contained() {
        let fst = BlockPos::new(0, 0, 0);
        let snd = BlockPos::new(64, 64, 64);
        let chunk_section_idx = ChunkSectionIdx::new(1, 1, 1);
        let mut result = positions_in_section(fst, snd, chunk_section_idx);
        result.sort_by_key(|pos| (pos.x, pos.y, pos.z));
        let mut expected = (16..=31)
            .flat_map(|x| (16..=31).map(move |y| (x, y)))
            .flat_map(|(x, y)| (16..=31).map(move |z| BlockPos::new(x, y, z)))
            .collect::<Vec<_>>();
        expected.sort_by_key(|pos| (pos.x, pos.y, pos.z));
        assert_eq!(result, expected);
    }

    #[test]
    fn test_positions_in_section_nonintersecting() {
        let fst = BlockPos::new(0, 0, 0);
        let snd = BlockPos::new(64, 64, 64);
        let chunk_section_idx = ChunkSectionIdx::new(-2, 2, -2);
        let result = positions_in_section(fst, snd, chunk_section_idx);
        assert!(result.is_empty());
    }

    #[test]
    fn test_positions_in_section_partial_intersection() {
        let fst = BlockPos::new(-3, 20, 17);
        let snd = BlockPos::new(20, 34, -10);
        let chunk_section_idx = ChunkSectionIdx::new(-1, 2, 1);
        let mut result = positions_in_section(fst, snd, chunk_section_idx);
        result.sort_by_key(|pos| (pos.x, pos.y, pos.z));
        let expected = vec![
            BlockPos::new(-3, 32, 16),
            BlockPos::new(-3, 32, 17),
            BlockPos::new(-3, 33, 16),
            BlockPos::new(-3, 33, 17),
            BlockPos::new(-3, 34, 16),
            BlockPos::new(-3, 34, 17),
            BlockPos::new(-2, 32, 16),
            BlockPos::new(-2, 32, 17),
            BlockPos::new(-2, 33, 16),
            BlockPos::new(-2, 33, 17),
            BlockPos::new(-2, 34, 16),
            BlockPos::new(-2, 34, 17),
            BlockPos::new(-1, 32, 16),
            BlockPos::new(-1, 32, 17),
            BlockPos::new(-1, 33, 16),
            BlockPos::new(-1, 33, 17),
            BlockPos::new(-1, 34, 16),
            BlockPos::new(-1, 34, 17),
        ];
        assert_eq!(result, expected);
    }

    #[test]
    fn test_occupied_regions_empty() {
        let chunks = vec![
            Chunk::empty(0, 0, 2),
            Chunk::empty(0, 1, 2),
            Chunk::empty(1, 0, 2),
            Chunk::empty(1, 1, 2),
        ];
        let fst = BlockPos::new(0, 0, 0);
        let snd = BlockPos::new(31, 31, 31);
        let visited = visit_occupied(&chunks, fst, snd);
        assert!(visited.is_empty());
    }

    #[test]
    fn test_occupied_regions_partial() {
        let mut chunks = vec![
            Chunk::empty(0, -1, 8),
            Chunk::empty(0, 0, 8),
            Chunk::empty(0, 1, 8),
            Chunk::empty(1, -1, 8),
            Chunk::empty(1, 0, 8),
            Chunk::empty(1, 1, 8),
        ];
        let pos = BlockPos::new(12, 45, 2);
        set_block(&mut chunks, pos, 3);
        let fst = BlockPos::new(0, 0, -10);
        let snd = BlockPos::new(31, 60, 15);
        let visited = visit_occupied(&chunks, fst, snd);
        assert_eq!(visited.get(&pos), Some(&1));
        assert_eq!(visited.len(), 16 * 16 * 16, "should visit a single section");
    }

    #[test]
    fn test_occupied_regions_partial_not_included() {
        let mut chunks = vec![
            Chunk::empty(0, -1, 8),
            Chunk::empty(0, 0, 8),
            Chunk::empty(0, 1, 8),
            Chunk::empty(0, 2, 8),
            Chunk::empty(1, -1, 8),
            Chunk::empty(1, 0, 8),
            Chunk::empty(1, 1, 8),
            Chunk::empty(1, 2, 8),
        ];
        let pos = BlockPos::new(12, 45, 2);
        set_block(&mut chunks, pos, 3);
        let fst = BlockPos::new(0, 0, 4);
        let snd = BlockPos::new(31, 60, 31);
        let visited = visit_occupied(&chunks, fst, snd);
        assert_eq!(
            visited.get(&pos),
            None,
            "position is not in bounds and should not be visited"
        );
        assert_eq!(
            visited.len(),
            16 * 16 * 12,
            "should visit the part of the non-empty section that's in bounds"
        );
    }
}
