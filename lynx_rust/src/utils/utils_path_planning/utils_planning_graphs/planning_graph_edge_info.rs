

#[derive(Clone, Debug)]
pub struct PlanningGraphEdgeInfo {
    pub edge_idx: usize,
    pub start_node_idx: usize,
    pub end_node_idx: usize,
    pub ancestor_root_node_idx: usize
}

impl PlanningGraphEdgeInfo {
    pub fn new(edge_idx: usize, start_node_idx: usize, end_node_idx: usize, ancestor_root_node_idx: usize) -> Self {
        return Self {edge_idx, start_node_idx, end_node_idx, ancestor_root_node_idx}
    }
}