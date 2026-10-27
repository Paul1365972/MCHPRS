use crate::scenario::Step;
use std::sync::{Mutex, MutexGuard, PoisonError};

const FURTHER_RUNS: [Step; 20] = [Step::Run(100); 20];

fn the_board() -> MutexGuard<'static, ()> {
    static BOARD: Mutex<()> = Mutex::new(());
    BOARD.lock().unwrap_or_else(PoisonError::into_inner)
}

macro_rules! board_tests {
    ($($name:ident($path:literal, $scenario:expr) simulation[$($simulation:ident),*] board[$($board:ident),*];)*) => {
        $(mod $name {
            $(#[test]
            #[ignore = "needs the board"]
            fn $board() {
                let _board = crate::board::the_board();
                let further = &crate::board::FURTHER_RUNS;
                let flags = crate::scenario::flags!($board);
                crate::comparison::assert_fpga_runs_as_phase(stringify!($name), flags, further);
            })*
        })*
    };
}

crate::scenario::cases!(board_tests);
