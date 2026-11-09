use mchprs_blocks::blocks::Instrument;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SoundSource {
    Records = 2,
    Blocks = 4,
}

#[derive(Clone, Copy, Debug, PartialEq)]
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

    pub fn range(self) -> f64 {
        16.0 * f64::from(self.volume.max(1.0))
    }
}
