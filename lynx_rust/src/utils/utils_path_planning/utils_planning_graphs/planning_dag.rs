use crate::utils::utils_path_planning::utils_planning_graphs::{planning_graph_edge_info::PlanningGraphEdgeInfo, planning_graph_node_info::PlanningGraphNodeInfo};
use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::utils::utils_math::interpolation_utils::get_linear_interpolation_with_stepsize;
use crate::utils::utils_math::nalgebra_utils::vec_to_dvec;
use nalgebra::{DVector};
use termion::{color, style};

#[derive(Debug, Clone)]
pub struct PlanningDAG {
    _nodes: Vec<DVector<f64>>,
    _edges: Vec<LinearSplinePath>,
    _node_infos: Vec<PlanningGraphNodeInfo>,
    _edge_infos: Vec<PlanningGraphEdgeInfo>,
    _num_roots: usize
}

impl PlanningDAG {
    pub fn new_unidirectional(root_node: &DVector<f64>) -> Self {
        let nodes = Vec::new();
        let edges = Vec::new();
        let node_infos = Vec::new();
        let edge_infos = Vec::new();

        let mut out_self =  Self { _nodes: nodes, _edges: edges, _node_infos: node_infos, _edge_infos: edge_infos, _num_roots: 1 };

        out_self._add_node(root_node, true);

        return out_self;
    }

    pub fn new_bidirectional(root_node1: &DVector<f64>, root_node2: &DVector<f64>) -> Self {
        let nodes = Vec::new();
        let edges = Vec::new();
        let node_infos = Vec::new();
        let edge_infos = Vec::new();

        let mut out_self =  Self { _nodes: nodes, _edges: edges, _node_infos: node_infos, _edge_infos: edge_infos, _num_roots: 2 };

        out_self._add_node(root_node1, true);
        out_self._add_node(root_node2, true);

        return out_self;
    }

    pub fn new_unidirectional_vec(root_node: &Vec<f64>) -> Self {
        return Self::new_unidirectional(&vec_to_dvec(root_node));
    }

    pub fn new_bidirectional_vec(root_node1: &Vec<f64>, root_node2: &Vec<f64>) -> Self {
        return Self::new_bidirectional(&vec_to_dvec(root_node1), &vec_to_dvec(root_node2));
    }

    pub fn new_empty() -> Self {
        let nodes = Vec::new();
        let edges = Vec::new();
        let node_infos = Vec::new();
        let edge_infos = Vec::new();

        let mut out_self =  Self { _nodes: nodes, _edges: edges, _node_infos: node_infos, _edge_infos: edge_infos, _num_roots: 0 };

        return out_self;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_another_root_node(&mut self, new_root_node: &DVector<f64>) -> usize {
        let node_idx = self._add_node(new_root_node, true);
        self._num_roots += 1;
        return node_idx;
    }

    pub fn add_another_root_node_vec(&mut self, new_root_node: &Vec<f64>) -> usize {
        return self.add_another_root_node(&vec_to_dvec(new_root_node));
    }

    pub fn add_node_with_auto_two_waypoint_inflow_edge(&mut self, node: &DVector<f64>, start_node_idx: usize) -> Result<usize, String> {
        if start_node_idx >= self._nodes.len() {
            println!("{}{}WARNING: tried to add a node to planning tree where start_node_idx ({:?}) was >= than number of nodes ({:?}).  {}", color::Fg(color::Yellow), style::Bold, start_node_idx, self._nodes.len(), style::Reset);
            return Err( format!( "tried to add a node to planning tree where start_node_idx ({:?}) was >= than number of nodes ({:?}).",  start_node_idx, self._nodes.len()) );
        }

        let node_idx = self._add_node(node, false);
        self._add_edge_with_just_two_endpoint_nodes(start_node_idx, node_idx);
        return Ok(node_idx);
    }

    pub fn add_node_vec_with_auto_two_waypoint_inflow_edge(&mut self, node: &Vec<f64>, start_node_idx: usize) -> Result<usize, String> {
        return self.add_node_with_auto_two_waypoint_inflow_edge(&vec_to_dvec(node), start_node_idx );
    }

    pub fn add_node_with_auto_linear_inflow_edge(&mut self, node: &DVector<f64>, start_node_idx: usize, lambda: f64) -> Result<usize, String> {
        let linear_interpolation = get_linear_interpolation_with_stepsize( self.get_node_ref(start_node_idx)?, node, lambda );
        let linear_spline_path = LinearSplinePath::new(linear_interpolation);
        return self.add_node_using_inflow_edge(&linear_spline_path, start_node_idx);
    }

    pub fn add_node_vec_with_auto_linear_inflow_edge(&mut self, node: &Vec<f64>, start_node_idx: usize, lambda: f64) -> Result<usize, String> {
        return self.add_node_with_auto_linear_inflow_edge(&vec_to_dvec(node), start_node_idx, lambda);
    }

    pub fn add_node_using_inflow_edge(&mut self, inflow_edge: &LinearSplinePath, start_node_idx: usize) -> Result<usize, String> {
        if start_node_idx >= self._nodes.len() {
            println!("{}{}WARNING: tried to add a node to planning tree where start_node_idx ({:?}) was >= than number of nodes ({:?}).  {}", color::Fg(color::Yellow), style::Bold, start_node_idx, self._nodes.len(), style::Reset);
            return Err( format!( "tried to add a node to planning tree where start_node_idx ({:?}) was >= than number of nodes ({:?}).",  start_node_idx, self._nodes.len()) );
        }

        let node_idx = self._add_node(&inflow_edge.waypoints[inflow_edge.get_num_waypoints()-1], false);
        let edge_res = self.add_edge(inflow_edge, start_node_idx, node_idx);

        if edge_res.is_err() {
            let last_idx = self._nodes.len()-1;
            self._nodes.remove(last_idx);
            self._node_infos.remove(last_idx);

            return Err(format!("edge addition to planning tree failed with following error: {:?}", edge_res.err().unwrap()));
        }

        return Ok(node_idx);
    }

    pub fn add_edge(&mut self, edge: &LinearSplinePath, start_node_idx: usize, end_node_idx: usize) -> Result<(), String> {
        if start_node_idx == end_node_idx { return Err("start_node_idx and end_node_idx are the same".to_string()); }

        if self._node_infos[end_node_idx].is_root_node && !self._node_infos[start_node_idx].is_root_node { return Err("tried to connect tree to root node".to_string()); }

        let d1 = (&edge.waypoints[0] - &self._nodes[start_node_idx]).norm();
        if d1 > 0.000001 { return Err("the edge start point is too far from the given start node".to_string()); }
        let d2 = (&edge.waypoints[edge.waypoints.len() - 1] - &self._nodes[end_node_idx]).norm();
        if d2 > 0.000001 { return Err("the edge end point is too far from the given end node".to_string()); }

        let mut ancestor_root_node_idxs: Vec<usize> = self._node_infos[start_node_idx].inflow_edge_idxs.iter().map(|x|  self._edge_infos[*x].ancestor_root_node_idx  ).collect();
        let mut outflow_edge_ancestor_root_node_idxs: Vec<usize> = self._node_infos[end_node_idx].outflow_edge_idxs.iter().map(|x|  self._edge_infos[*x].ancestor_root_node_idx  ).collect();

        let l1 = ancestor_root_node_idxs.len();
        let l2 = outflow_edge_ancestor_root_node_idxs.len();
        for i in 0..l1 {
            for j in 0..l2 {
                if ancestor_root_node_idxs[i] == outflow_edge_ancestor_root_node_idxs[j] { return Err("would have created a cycle in the tree.  Not allowed.".to_string()); }
            }
        }

        if ancestor_root_node_idxs.len() == 0 { ancestor_root_node_idxs.push( start_node_idx ); }

        for i in 0..ancestor_root_node_idxs.len() {
            self._edges.push(edge.clone());
            let edge_idx = self._edges.len() - 1;
            self._edge_infos.push(PlanningGraphEdgeInfo::new(edge_idx, start_node_idx, end_node_idx, ancestor_root_node_idxs[i]));
            self._node_infos[start_node_idx].add_outflow_edge_idx(edge_idx);
            self._node_infos[end_node_idx].add_inflow_edge_idx(edge_idx);
        }

        return Ok(( ));
    }

    pub fn add_auto_linear_edge(&mut self, start_node_idx: usize, end_node_idx: usize, lambda: f64) -> Result<(), String> {
        let linear_interpolation = get_linear_interpolation_with_stepsize( self.get_node_ref(start_node_idx)?, self.get_node_ref(end_node_idx)?, lambda );
        let linear_spline_path = LinearSplinePath::new(linear_interpolation);
        return self.add_edge(&linear_spline_path, start_node_idx, end_node_idx);
    }

    pub fn add_auto_two_waypoint_edge(&mut self, start_node_idx: usize, end_node_idx: usize) -> Result<(), String> {
        let mut linear_spline_path = LinearSplinePath::new_empty();
        linear_spline_path.add_waypoint( &self._nodes[start_node_idx] );
        linear_spline_path.add_waypoint( &self._nodes[end_node_idx] );
        return self.add_edge(&linear_spline_path, start_node_idx, end_node_idx);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _add_node(&mut self, node: &DVector<f64>, is_root_node: bool) -> usize {
        self._nodes.push(node.clone());
        let node_idx = self._nodes.len() -1;
        self._node_infos.push( PlanningGraphNodeInfo::new(node_idx, Vec::new(), Vec::new(), is_root_node) );

        return node_idx;
    }

    fn _add_edge_with_just_two_endpoint_nodes(&mut self, start_node_idx: usize, end_node_idx: usize) -> Result<(), String> {
        let mut new_edge = LinearSplinePath::new_empty();
        new_edge.add_waypoint( &self.get_node(start_node_idx)? );
        new_edge.add_waypoint( &self.get_node(end_node_idx)? );

        return self.add_edge(&new_edge, start_node_idx, end_node_idx);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_tree(&self) {
        println!("{}{}planning tree nodes ---> {}", color::Fg(color::Green), style::Bold, style::Reset);

        let l = self._nodes.len();
        for i in 0..l {
            println!("     {:?} -- inflow edge idxs: {:?}, outflow edge idxs: {:?}, is leaf: {:?}, is root: {:?}, node: {:?}", i, self._node_infos[i].inflow_edge_idxs, self._node_infos[i].outflow_edge_idxs,
                     self._node_infos[i].is_leaf_node, self._node_infos[i].is_root_node, self._nodes[i].data.as_vec());
        }

        println!("\n{}{}planning tree edges ---> {}", color::Fg(color::Green), style::Bold, style::Reset);
        let l = self._edges.len();
        for i in 0..l {
            println!("     {:?} -- start node idx: {:?}, end node idx: {:?}, ancestor root node idx: {:?}, start node: {:?}, end node: {:?}, path: {:?}", i, self._edge_infos[i].start_node_idx, self._edge_infos[i].end_node_idx, self._edge_infos[i].ancestor_root_node_idx,
                     self._nodes[self._edge_infos[i].start_node_idx].data.as_vec(), self._nodes[self._edge_infos[i].end_node_idx].data.as_vec(), self._edges[i]);
        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_path_from_tree_root_to_node(&self, node_idx: usize) -> Result<LinearSplinePath, String> {
        let mut out_path = LinearSplinePath::new_empty();
        if self._node_infos[node_idx].is_root_node {
            out_path.add_waypoint( &self._nodes[node_idx] );
            return Ok(out_path);
        }

        let mut curr_node_idx = node_idx;
        loop {
            if self._node_infos[curr_node_idx].is_root_node { break; }
            let inflow_edge_idx = self._node_infos[curr_node_idx].inflow_edge_idxs[0];
            let combine_path_res = self._edges[inflow_edge_idx].combine_ordered(&out_path);
            if combine_path_res.is_err() {
                return Err("error on combine".to_string());
            }
            else { out_path = combine_path_res.ok().unwrap(); }
            curr_node_idx = self._edge_infos[inflow_edge_idx].start_node_idx;
        }

        if out_path.get_num_waypoints() == 0 { return Err("empty path after all combines".to_string()); }
        else { return Ok(out_path); }
    }

    pub fn get_path_from_particular_tree_root_to_node(&self, tree_root_idx: usize, node_idx: usize) -> Result<LinearSplinePath, String> {
        if !self._node_infos[tree_root_idx].is_root_node { return Err("given tree_root_idx is not even a root node".to_string()); }

        let mut out_path = LinearSplinePath::new_empty();
        if node_idx == tree_root_idx {
            out_path.add_waypoint( &self._nodes[node_idx] );
            return Ok(out_path);
        }

        let mut inflow_can_connect_to_tree_root_idx = false;
        self._node_infos[node_idx].inflow_edge_idxs.iter().for_each( |x| if self._edge_infos[*x].ancestor_root_node_idx == tree_root_idx { inflow_can_connect_to_tree_root_idx = true } );

        if !inflow_can_connect_to_tree_root_idx { return Err("inflow edge is not connected to the desired root node".to_string()); }

        let mut curr_node_idx = node_idx;
        loop {
            if curr_node_idx == tree_root_idx { break; }
            let mut inflow_edge_idx = usize::max_value();
            self._node_infos[curr_node_idx].inflow_edge_idxs.iter().for_each(|x| {
                if self._edge_infos[*x].ancestor_root_node_idx == tree_root_idx { inflow_edge_idx = self._edge_infos[*x].edge_idx }
            });
            let combine_path_res = self._edges[inflow_edge_idx].combine_ordered(&out_path);
            if combine_path_res.is_err() {
                return Err("error on combine".to_string());
            }
            else { out_path = combine_path_res.ok().unwrap(); }
            curr_node_idx = self._edge_infos[inflow_edge_idx].start_node_idx;
        }

        if out_path.get_num_waypoints() == 0 { return Err("empty path after all combines".to_string()); }
        else { return Ok(out_path); }
    }

    pub fn get_path_that_connects_two_roots(&self, root_idx1: usize, root_idx2: usize, connecting_node_idx: usize) -> Result<LinearSplinePath, String> {
        let path1_res = self.get_path_from_particular_tree_root_to_node(root_idx1, connecting_node_idx);
        if path1_res.is_err() {
            return Err("error on connection1.  The error was the following: ".to_string() + path1_res.err().unwrap().as_str() );
        }

        let path2_res = self.get_path_from_particular_tree_root_to_node(root_idx2, connecting_node_idx);
        if path2_res.is_err() {
            return Err("error on connection2.  The error was the following: ".to_string() + path2_res.err().unwrap().as_str());
        }

        let mut path1 = path1_res.ok().unwrap();
        let mut path2 = path2_res.ok().unwrap();
        path2.reverse();

        let mut out_path_res = path1.combine_ordered(&path2);
        if out_path_res.is_err() { return Err("error on final connection of get path that connects two roots".to_string()); }

        return Ok(out_path_res.ok().unwrap());
    }

    pub fn get_previous_n_states_that_lead_into_node(&self, node_idx: usize, n: usize) -> Result<Vec<DVector<f64>>, String> {
        if node_idx >= self._nodes.len() {
            return Err(format!("node_idx {:?} is too high for number of nodes in planning tree ({:?})", node_idx, self._nodes.len()));
        }

        let mut out_vec = Vec::new();

        let mut curr_idx = node_idx;
        loop {
            let mut curr_inflow_edge_vec = &self._node_infos[curr_idx].inflow_edge_idxs;
            if curr_inflow_edge_vec.len() == 0 {
                if out_vec.len() == 0 {
                    for i in 0..n {
                        out_vec.push( self._nodes[node_idx].clone() );
                    }
                    out_vec.reverse();
                    return Ok(out_vec);
                } else {
                    let remainder = n - out_vec.len();
                    let dvec_to_add = self._nodes[curr_idx].clone();
                    for i in 0..remainder {
                        out_vec.push( dvec_to_add.clone() );
                    }
                    out_vec.reverse();
                    return Ok(out_vec);
                }
            } else {
                let inflow_edge_idx = self._node_infos[curr_idx].inflow_edge_idxs[0];
                let curr_edge = &self._edges[inflow_edge_idx];
                let waypoints = &curr_edge.waypoints;
                let l = waypoints.len();
                for i in 0..l-1 {
                    out_vec.push( waypoints[l - 1 - i].clone() );
                    if out_vec.len() == n {
                        out_vec.reverse();
                        return Ok(out_vec);
                    }
                }

                curr_idx = self._edge_infos[inflow_edge_idx].start_node_idx;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_node_ref(&self, idx: usize) -> Result<&DVector<f64>, String> {
        if idx >= self._nodes.len() {
            return Err( format!( "tried to get node ref in planning tree where idx ({:?}) was >= than number of nodes ({:?}).",  idx, self._nodes.len()) );
        }
        return Ok(&self._nodes[idx]);
    }

    pub fn get_edge_ref(&self, idx: usize) -> Result<&LinearSplinePath, String> {
        if idx >= self._edges.len() {
            return Err( format!( "tried to get edge ref in planning tree where idx ({:?}) was >= than number of edges ({:?}).",  idx, self._edges.len()) );
        }
        return Ok(&self._edges[idx]);
    }

    pub fn get_node(&self, idx: usize) -> Result<DVector<f64>, String> {
        if idx >= self._nodes.len() {
            return Err( format!( "tried to get node in planning tree where idx ({:?}) was >= than number of nodes ({:?}).",  idx, self._nodes.len()) );
        }
        return Ok(self._nodes[idx].clone());
    }

    pub fn get_edge(&self, idx: usize) -> Result<LinearSplinePath, String> {
        if idx >= self._edges.len() {
            return Err( format!( "tried to get edge ref in planning tree where idx ({:?}) was >= than number of edges ({:?}).",  idx, self._edges.len()) );
        }

        return Ok(self._edges[idx].clone());
    }

    pub fn get_num_roots(&self) -> usize {
        return self._num_roots;
    }
}