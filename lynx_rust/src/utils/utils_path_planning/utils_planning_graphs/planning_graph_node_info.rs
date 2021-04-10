

#[derive(Clone, Debug)]
pub struct PlanningGraphNodeInfo {
    pub node_idx: usize,
    pub outflow_edge_idxs: Vec<usize>,
    pub inflow_edge_idxs: Vec<usize>,
    pub is_leaf_node: bool,
    pub is_root_node: bool,
}

impl PlanningGraphNodeInfo {
    pub fn new(node_idx: usize, successor_edge_idxs: Vec<usize>, predecessor_edge_idxs: Vec<usize>, is_root_node: bool) -> Self {
        let mut out_self = Self {node_idx, outflow_edge_idxs: successor_edge_idxs,
            inflow_edge_idxs: predecessor_edge_idxs, is_leaf_node: false, is_root_node};

        out_self._set_is_leaf_node();

        return out_self;
    }

    pub fn new_empty() -> Self {
        let node_idx = usize::max_value();
        let successor_edge_idxs = Vec::new();
        let predecessor_edge_idxs = Vec::new();
        return Self::new(node_idx, successor_edge_idxs, predecessor_edge_idxs, false);
    }

    fn _set_is_leaf_node(&mut self) {
        if self.outflow_edge_idxs.len() == 0 { self.is_leaf_node = true; }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_outflow_edge_idx(&mut self, outflow_edge_idx: usize) {
        if self.is_leaf_node { self.is_leaf_node = false; }
        self.outflow_edge_idxs.push(outflow_edge_idx);
    }

    pub fn add_inflow_edge_idx(&mut self, inflow_edge_idx: usize) {
        self.inflow_edge_idxs.insert(0, inflow_edge_idx);
    }
}



