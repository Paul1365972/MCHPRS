use mchprs_blocks::blocks::{Block, Button, ComparatorMode, Instrument};
use mchprs_blocks::registries::sound_event;

#[derive(Clone, Copy, Debug)]
pub enum SoundSource {
    Records = 2,
    Blocks = 4,
}

#[derive(Clone, Copy, Debug)]
pub struct Sound {
    pub event: i32,
    pub source: SoundSource,
    pub volume: f32,
    pub pitch: f32,
}

impl Sound {
    pub fn note(instrument: Instrument, note: u8) -> Sound {
        Sound {
            event: instrument.to_sound_id(),
            source: SoundSource::Records,
            volume: 3.0,
            pitch: f32::exp2((f32::from(note) - 12.0) / 12.0),
        }
    }

    pub fn of_use(block: Block) -> Option<Sound> {
        let (event, volume, pitch) = match block {
            _ if let Some(Button { powered: false, .. }) = block.button() => {
                (button_click(block, true), 1.0, 1.0)
            }
            Block::Comparator(comparator) => {
                let pitch = match comparator.mode.toggle() {
                    ComparatorMode::Compare => 0.5,
                    ComparatorMode::Subtract => 0.55,
                };
                (sound_event::BLOCK_COMPARATOR_CLICK, 0.3, pitch)
            }
            _ => return None,
        };
        Some(Sound {
            event,
            source: SoundSource::Blocks,
            volume,
            pitch,
        })
    }

    pub fn of_change(old: Block, new: Block) -> Option<Sound> {
        let (event, volume, pitch) = match (old, new) {
            (Block::Lever { powered: was, .. }, Block::Lever { powered, .. }) if was != powered => {
                let pitch = if powered { 0.6 } else { 0.5 };
                (sound_event::BLOCK_LEVER_CLICK, 0.3, pitch)
            }
            _ if let (Some(Button { powered: true, .. }), Some(Button { powered: false, .. })) =
                (old.button(), new.button()) =>
            {
                (button_click(new, false), 1.0, 1.0)
            }
            (Block::IronTrapdoor { open: was, .. }, Block::IronTrapdoor { open, .. })
                if was != open =>
            {
                let event = if open {
                    sound_event::BLOCK_IRON_TRAPDOOR_OPEN
                } else {
                    sound_event::BLOCK_IRON_TRAPDOOR_CLOSE
                };
                (event, 1.0, 0.95)
            }
            _ if old != new => {
                let (powered, off, on) = match new {
                    Block::OakPressurePlate { powered }
                    | Block::SprucePressurePlate { powered }
                    | Block::BirchPressurePlate { powered }
                    | Block::JunglePressurePlate { powered }
                    | Block::AcaciaPressurePlate { powered }
                    | Block::DarkOakPressurePlate { powered }
                    | Block::MangrovePressurePlate { powered } => (
                        powered,
                        sound_event::BLOCK_WOODEN_PRESSURE_PLATE_CLICK_OFF,
                        sound_event::BLOCK_WOODEN_PRESSURE_PLATE_CLICK_ON,
                    ),
                    Block::CherryPressurePlate { powered } => (
                        powered,
                        sound_event::BLOCK_CHERRY_WOOD_PRESSURE_PLATE_CLICK_OFF,
                        sound_event::BLOCK_CHERRY_WOOD_PRESSURE_PLATE_CLICK_ON,
                    ),
                    Block::BambooPressurePlate { powered } => (
                        powered,
                        sound_event::BLOCK_BAMBOO_WOOD_PRESSURE_PLATE_CLICK_OFF,
                        sound_event::BLOCK_BAMBOO_WOOD_PRESSURE_PLATE_CLICK_ON,
                    ),
                    Block::CrimsonPressurePlate { powered }
                    | Block::WarpedPressurePlate { powered } => (
                        powered,
                        sound_event::BLOCK_NETHER_WOOD_PRESSURE_PLATE_CLICK_OFF,
                        sound_event::BLOCK_NETHER_WOOD_PRESSURE_PLATE_CLICK_ON,
                    ),
                    Block::StonePressurePlate { powered }
                    | Block::PolishedBlackstonePressurePlate { powered } => (
                        powered,
                        sound_event::BLOCK_STONE_PRESSURE_PLATE_CLICK_OFF,
                        sound_event::BLOCK_STONE_PRESSURE_PLATE_CLICK_ON,
                    ),
                    _ => return None,
                };
                (if powered { on } else { off }, 1.0, 1.0)
            }
            _ => return None,
        };
        Some(Sound {
            event,
            source: SoundSource::Blocks,
            volume,
            pitch,
        })
    }

    pub fn range(self) -> f64 {
        16.0 * f64::from(self.volume.max(1.0))
    }
}

fn button_click(button: Block, on: bool) -> i32 {
    let (off_event, on_event) = match button {
        Block::StoneButton(_) | Block::PolishedBlackstoneButton(_) => (
            sound_event::BLOCK_STONE_BUTTON_CLICK_OFF,
            sound_event::BLOCK_STONE_BUTTON_CLICK_ON,
        ),
        Block::CherryButton(_) => (
            sound_event::BLOCK_CHERRY_WOOD_BUTTON_CLICK_OFF,
            sound_event::BLOCK_CHERRY_WOOD_BUTTON_CLICK_ON,
        ),
        Block::BambooButton(_) => (
            sound_event::BLOCK_BAMBOO_WOOD_BUTTON_CLICK_OFF,
            sound_event::BLOCK_BAMBOO_WOOD_BUTTON_CLICK_ON,
        ),
        Block::CrimsonButton(_) | Block::WarpedButton(_) => (
            sound_event::BLOCK_NETHER_WOOD_BUTTON_CLICK_OFF,
            sound_event::BLOCK_NETHER_WOOD_BUTTON_CLICK_ON,
        ),
        _ => (
            sound_event::BLOCK_WOODEN_BUTTON_CLICK_OFF,
            sound_event::BLOCK_WOODEN_BUTTON_CLICK_ON,
        ),
    };
    if on {
        on_event
    } else {
        off_event
    }
}
