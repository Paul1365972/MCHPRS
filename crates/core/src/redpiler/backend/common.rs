use std::num::NonZeroU8;

use mchprs_blocks::blocks::ComparatorMode;

use super::nodes::NodeId;

#[repr(align(16))]
#[derive(Debug, Clone, Default)]
pub struct NodeInput {
    pub ss_counts: [u8; 16],
}

impl NodeInput {
    const BOOL_INPUT_MASK: u128 = u128::from_ne_bytes([
        0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    ]);

    pub fn get_bool(&self) -> bool {
        u128::from_le_bytes(self.ss_counts) & Self::BOOL_INPUT_MASK != 0
    }

    pub fn get_ss(&self) -> u8 {
        Self::last_index_positive(&self.ss_counts) as u8
    }

    fn last_index_positive(array: &[u8; 16]) -> u32 {
        // Note: this might be slower on big-endian systems
        let value = u128::from_le_bytes(*array);
        if value == 0 {
            0
        } else {
            15 - (value.leading_zeros() >> 3)
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct NonMaxU8(NonZeroU8);

impl NonMaxU8 {
    pub fn new(value: u8) -> Option<Self> {
        NonZeroU8::new(value + 1).map(|x| Self(x))
    }

    pub fn get(self) -> u8 {
        self.0.get() - 1
    }
}

#[derive(Clone, Copy)]
pub struct ForwardLink {
    data: u32,
}

impl ForwardLink {
    pub fn new(id: NodeId, side: bool, ss: u8) -> Self {
        assert!(id.index() < (1 << 27));
        // the clamp_weights compile pass should ensure ss < 15
        assert!(ss < 15);
        Self {
            data: (id.index() as u32) << 5 | if side { 1 << 4 } else { 0 } | ss as u32,
        }
    }

    pub fn node(self) -> NodeId {
        unsafe {
            // safety: ForwardLink is constructed using a NodeId
            NodeId::from_index((self.data >> 5) as usize)
        }
    }

    pub fn side(self) -> bool {
        self.data & (1 << 4) != 0
    }

    pub fn ss(self) -> u8 {
        (self.data & 0b1111) as u8
    }
}

impl std::fmt::Debug for ForwardLink {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ForwardLink")
            .field("node", &self.node())
            .field("side", &self.side())
            .field("ss", &self.ss())
            .finish()
    }
}

// TODO: Duplication, this is identical to ForwardLink, but serves a different purpose
#[derive(Debug, Clone, Copy)]
pub struct NodeIdWithData {
    data: u32,
}

impl NodeIdWithData {
    pub fn new(node_id: NodeId) -> Self {
        debug_assert!(node_id.index() < (1 << 27));
        Self {
            data: (node_id.index() as u32) << 5,
        }
    }

    pub fn new_with_data(node_id: NodeId, bool: bool, ss: u8) -> Self {
        debug_assert!(node_id.index() < (1 << 27));
        debug_assert!(ss < (1 << 4));
        Self {
            data: (node_id.index() as u32) << 5 | if bool { 1 << 4 } else { 0 } | ss as u32,
        }
    }

    pub fn node(self) -> NodeId {
        unsafe {
            // safety: NodeIdWithData is contructed using a NodeId
            NodeId::from_index((self.data >> 5) as usize)
        }
    }

    pub fn internal_tick(self) -> bool {
        self.data & (1 << 4) != 0
    }

    pub fn ss(self) -> u8 {
        (self.data & 0b1111) as u8
    }
}

// This function is optimized for input values from 0 to 15 and does not work correctly outside that range
pub fn calculate_comparator_output(
    mode: ComparatorMode,
    input_strength: u8,
    power_on_sides: u8,
) -> u8 {
    let difference = input_strength.wrapping_sub(power_on_sides);
    if difference <= 15 {
        match mode {
            ComparatorMode::Compare => input_strength,
            ComparatorMode::Subtract => difference,
        }
    } else {
        0
    }
}
