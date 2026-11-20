use strum::Display;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Display)]
#[strum(serialize_all = "snake_case")]
pub enum NodeModule {
    BinaryInput,
    StrengthInput,
    Button,
    Plate,
    Torch,
    Repeater,
    RepeaterPulseFree,
    RepeaterPrioritized,
    RepeaterPrioritizedPulseFree,
    ComparatorBinary,
    Comparator,
    Lamp,
    Trapdoor,
    RedstoneWire,
    NoteBlock,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Display)]
#[strum(serialize_all = "SCREAMING_SNAKE_CASE")]
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
    Fires,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Display)]
#[strum(serialize_all = "snake_case")]
pub enum FiringPort {
    Firing,
    FiringOff,
    FiringOn,
    FiringPulse,
    TickStart,
}
