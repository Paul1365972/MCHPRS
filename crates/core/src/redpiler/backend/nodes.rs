use std::ops::{Index, IndexMut};

// This is Pretty Bad:tm: because one can create a NodeId using another instance of NodeStorage,
// but at least some type system protection is better than none.
pub struct NodeStorage<T> {
    pub nodes: Box<[T]>,
}

impl<T> Default for NodeStorage<T> {
    fn default() -> Self {
        Self {
            nodes: Default::default(),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct NodeId(u32);

impl<T> NodeStorage<T> {
    pub fn new(nodes: Box<[T]>) -> NodeStorage<T> {
        NodeStorage { nodes }
    }

    pub fn get(&self, idx: usize) -> NodeId {
        if self.nodes.get(idx).is_some() {
            NodeId(idx as u32)
        } else {
            panic!("node index out of bounds: {}", idx)
        }
    }

    pub fn inner(&self) -> &[T] {
        &self.nodes
    }

    pub fn inner_mut(&mut self) -> &mut [T] {
        &mut self.nodes
    }

    pub fn into_inner(self) -> Box<[T]> {
        self.nodes
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }
}

impl NodeId {
    pub fn index(self) -> usize {
        self.0 as usize
    }

    /// Safety: index must be within bounds of nodes array
    pub unsafe fn from_index(index: usize) -> NodeId {
        NodeId(index as u32)
    }
}

impl<T> Index<NodeId> for NodeStorage<T> {
    type Output = T;

    // The index here MUST have been created by this instance, otherwise scary things will happen !
    fn index(&self, index: NodeId) -> &Self::Output {
        unsafe { self.nodes.get_unchecked(index.0 as usize) }
    }
}

impl<T> IndexMut<NodeId> for NodeStorage<T> {
    fn index_mut(&mut self, index: NodeId) -> &mut Self::Output {
        unsafe { self.nodes.get_unchecked_mut(index.0 as usize) }
    }
}
