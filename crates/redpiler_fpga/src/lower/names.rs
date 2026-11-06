#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeModule {
    Lever,
    StrengthInput,
    Button,
    Plate,
    Torch,
    Repeater,
    RepeaterPrioritized,
    ComparatorBinary,
    Comparator,
    Lamp,
    Trapdoor,
    RedstoneWire,
    NoteBlock,
}

impl NodeModule {
    pub fn name(self) -> &'static str {
        match self {
            Self::Lever => "lever",
            Self::StrengthInput => "strength_input",
            Self::Button => "button",
            Self::Plate => "plate",
            Self::Torch => "torch",
            Self::Repeater => "repeater",
            Self::RepeaterPrioritized => "repeater_prioritized",
            Self::ComparatorBinary => "comparator_binary",
            Self::Comparator => "comparator",
            Self::Lamp => "lamp",
            Self::Trapdoor => "trapdoor",
            Self::RedstoneWire => "redstone_wire",
            Self::NoteBlock => "note_block",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parameter {
    Powered,
    Strength,
    High,
    Locked,
    Touched,
    Delay,
    PendingDelay,
    Pulsing,
    Index,
    Subtract,
    FarValid,
    Far,
}

impl Parameter {
    pub fn name(self) -> &'static str {
        match self {
            Self::Powered => "POWERED",
            Self::Strength => "STRENGTH",
            Self::High => "HIGH",
            Self::Locked => "LOCKED",
            Self::Touched => "TOUCHED",
            Self::Delay => "DELAY",
            Self::PendingDelay => "PENDING_DELAY",
            Self::Pulsing => "PULSING",
            Self::Index => "INDEX",
            Self::Subtract => "SUBTRACT",
            Self::FarValid => "FAR_VALID",
            Self::Far => "FAR",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FiringPort {
    Firing,
    FiringOff,
    FiringOn,
    FiringPulse,
    TickStart,
}

impl FiringPort {
    pub fn name(self) -> &'static str {
        match self {
            Self::Firing => "firing",
            Self::FiringOff => "firing_off",
            Self::FiringOn => "firing_on",
            Self::FiringPulse => "firing_pulse",
            Self::TickStart => "tick_start",
        }
    }
}
