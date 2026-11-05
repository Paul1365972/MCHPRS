use mchprs_blocks::BlockPos;

const WIDTH_PIXELS: usize = 640;
const HEIGHT_PIXELS: usize = 480;
// Keeps the cell mask below Verilator's 65,536-bit limit on numbers, since Verilator parses the
// board system along with the circuit.
const MINIMUM_CELL_PIXELS: usize = 4;

type Vector = [i32; 3];

const X: usize = 0;
const Y: usize = 1;
const Z: usize = 2;

/// A lamp or trapdoor with the blocks that show it and the first block of each node powering it,
/// except levers, buttons and pressure plates, which sit on the player's side.
pub(crate) struct ScreenNode {
    pub(crate) index: usize,
    pub(crate) blocks: Vec<BlockPos>,
    pub(crate) input_blocks: Vec<BlockPos>,
}

/// The lamps and trapdoors projected along the axis they vary least on, seen from the side away
/// from the nodes powering them, as a grid of cells scaled to fill a VGA picture.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Screen {
    columns: usize,
    rows: usize,
    cell_pixels: usize,
    cells: Vec<Vec<usize>>,
}

impl Default for Screen {
    fn default() -> Screen {
        Screen {
            columns: 1,
            rows: 1,
            cell_pixels: 1,
            cells: vec![Vec::new()],
        }
    }
}

impl Screen {
    pub(crate) fn new(nodes: &[ScreenNode]) -> Screen {
        let blocks: Vec<(usize, Vector)> = nodes
            .iter()
            .flat_map(|node| node.blocks.iter().map(|&block| (node.index, vector(block))))
            .collect();
        if blocks.is_empty() {
            return Screen::default();
        }
        let depth = [X, Z, Y]
            .into_iter()
            .min_by_key(|&axis| distinct_coordinates(&blocks, axis))
            .expect("three axes");
        let upward = if depth == Y { [0, 0, -1] } else { [0, 1, 0] };
        let mut forward = [0; 3];
        forward[depth] = if input_side(nodes, depth) > 0 { 1 } else { -1 };
        let rightward = cross(forward, upward);
        let projected: Vec<(usize, i32, i32)> = blocks
            .iter()
            .map(|&(index, position)| (index, dot(position, rightward), dot(position, upward)))
            .collect();
        let (mut left, mut right, mut bottom, mut top) = (i32::MAX, i32::MIN, i32::MAX, i32::MIN);
        for &(_, horizontal, vertical) in &projected {
            left = left.min(horizontal);
            right = right.max(horizontal);
            bottom = bottom.min(vertical);
            top = top.max(vertical);
        }
        let width = (right - left) as usize + 1;
        let height = (top - bottom) as usize + 1;
        let blocks_per_cell = width
            .div_ceil(WIDTH_PIXELS / MINIMUM_CELL_PIXELS)
            .max(height.div_ceil(HEIGHT_PIXELS / MINIMUM_CELL_PIXELS));
        let columns = width.div_ceil(blocks_per_cell);
        let rows = height.div_ceil(blocks_per_cell);
        let mut cells = vec![Vec::new(); columns * rows];
        for (index, horizontal, vertical) in projected {
            let column = (horizontal - left) as usize / blocks_per_cell;
            let row = (top - vertical) as usize / blocks_per_cell;
            cells[row * columns + column].push(index);
        }
        for cell in &mut cells {
            cell.sort_unstable();
            cell.dedup();
        }
        Screen {
            columns,
            rows,
            cell_pixels: (WIDTH_PIXELS / columns).min(HEIGHT_PIXELS / rows),
            cells,
        }
    }

    pub(crate) fn columns(&self) -> usize {
        self.columns
    }

    pub(crate) fn rows(&self) -> usize {
        self.rows
    }

    pub(crate) fn cell_pixels(&self) -> usize {
        self.cell_pixels
    }

    /// The nodes in each cell, row by row from the top left.
    pub(crate) fn cells(&self) -> &[Vec<usize>] {
        &self.cells
    }
}

fn vector(position: BlockPos) -> Vector {
    [position.x, position.y, position.z]
}

fn dot(left: Vector, right: Vector) -> i32 {
    left.iter()
        .zip(right)
        .map(|(left, right)| left * right)
        .sum()
}

fn cross(left: Vector, right: Vector) -> Vector {
    [
        left[Y] * right[Z] - left[Z] * right[Y],
        left[Z] * right[X] - left[X] * right[Z],
        left[X] * right[Y] - left[Y] * right[X],
    ]
}

fn distinct_coordinates(blocks: &[(usize, Vector)], axis: usize) -> usize {
    let mut coordinates: Vec<i32> = blocks.iter().map(|(_, position)| position[axis]).collect();
    coordinates.sort_unstable();
    coordinates.dedup();
    coordinates.len()
}

/// Positive when the powering nodes mostly lie on the positive side along the depth axis.
fn input_side(nodes: &[ScreenNode], depth: usize) -> i32 {
    nodes
        .iter()
        .filter_map(|node| Some((node.blocks.first()?, &node.input_blocks)))
        .flat_map(|(block, input_blocks)| {
            input_blocks
                .iter()
                .map(move |input| (vector(*input)[depth] - vector(*block)[depth]).signum())
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(index: usize, blocks: &[[i32; 3]], input_blocks: &[[i32; 3]]) -> ScreenNode {
        let position = |&[x, y, z]: &[i32; 3]| BlockPos::new(x, y, z);
        ScreenNode {
            index,
            blocks: blocks.iter().map(position).collect(),
            input_blocks: input_blocks.iter().map(position).collect(),
        }
    }

    #[test]
    fn wall_seen_from_the_side_away_from_its_inputs() {
        let facing_south = Screen::new(&[
            node(0, &[[0, 1, 5]], &[[0, 1, 6]]),
            node(1, &[[1, 1, 5]], &[[1, 1, 6]]),
            node(2, &[[0, 0, 5]], &[[0, 0, 6]]),
        ]);
        assert_eq!((facing_south.columns(), facing_south.rows()), (2, 2));
        assert_eq!(facing_south.cells(), [vec![1], vec![0], vec![], vec![2]]);
        let facing_north = Screen::new(&[
            node(0, &[[0, 1, 5]], &[[0, 1, 4]]),
            node(1, &[[1, 1, 5]], &[[1, 1, 4]]),
            node(2, &[[0, 0, 5]], &[[0, 0, 4]]),
        ]);
        assert_eq!(facing_north.cells(), [vec![0], vec![1], vec![2], vec![]]);
    }

    #[test]
    fn floor_seen_from_above_with_north_up() {
        let screen = Screen::new(&[
            node(0, &[[3, 7, -2]], &[[3, 6, -2]]),
            node(1, &[[4, 7, -1]], &[[4, 6, -1]]),
        ]);
        assert_eq!(screen.cells(), [vec![0], vec![], vec![], vec![1]]);
    }

    #[test]
    fn gaps_and_merged_blocks_keep_their_place() {
        let screen = Screen::new(&[
            node(0, &[[0, 10, 0], [0, 14, 0]], &[]),
            node(1, &[[0, 12, 0]], &[]),
        ]);
        assert_eq!((screen.columns(), screen.rows()), (1, 5));
        assert_eq!(screen.cell_pixels(), HEIGHT_PIXELS / 5);
        assert_eq!(screen.cells(), [vec![0], vec![], vec![1], vec![], vec![0]]);
    }

    #[test]
    fn wide_layouts_share_cells() {
        let screen = Screen::new(&[
            node(0, &[[0, 0, 0]], &[]),
            node(1, &[[7, 0, 0]], &[]),
            node(2, &[[1279, 0, 0]], &[]),
        ]);
        assert_eq!(
            (screen.columns(), screen.rows(), screen.cell_pixels()),
            (160, 1, MINIMUM_CELL_PIXELS)
        );
        assert_eq!(screen.cells()[0], [0, 1]);
        assert_eq!(screen.cells()[159], [2]);
    }

    #[test]
    fn no_lamps_leave_one_empty_cell() {
        assert_eq!(Screen::new(&[]), Screen::default());
    }
}
