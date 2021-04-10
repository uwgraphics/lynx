use crate::utils::utils_sampling::prelude::*;
use nalgebra::DVector;
use termion::{style, color};
use crate::utils::utils_sampling::float_vec_sampler_traits::FloatVecSampler;

#[derive(Debug, Clone)]
struct KDTreeNode {
    pub point_id: usize,
    pub node_layer: usize,
    pub parent_node_id: Option<usize>,
    pub child1_node_id: Option<usize>,
    pub child2_node_id: Option<usize>,
    pub max_along_parent_split_dimension: Option<f64>,
    pub min_along_parent_split_dimension: Option<f64>,
}

impl KDTreeNode {
    pub fn new(point_id: usize, node_layer: usize, parent_node_id: Option<usize>, child1_node_id: Option<usize>, child2_node_id: Option<usize>, max_along_parent_split_dimension: Option<f64>, min_along_parent_split_dimension: Option<f64>) -> Self {
        Self { point_id, node_layer, parent_node_id, child1_node_id, child2_node_id, max_along_parent_split_dimension, min_along_parent_split_dimension }
    }

    pub fn is_leaf(&self) -> bool {
        if self.child1_node_id.is_none() && self.child2_node_id.is_none() { return true; }
        else { return false; }
    }

    pub fn is_root(&self) -> bool {
        if self.parent_node_id.is_none() { return true; }
        else { return false; }
    }
}

#[derive(Debug, Clone)]
pub struct KDTree {
    _dim: usize,
    _depth: usize,
    _points: Vec<DVector<f64>>,
    _kdtree_nodes: Vec<KDTreeNode>,
    _num_adds_count: usize
}

impl KDTree {
    pub fn new_empty(dim: usize, ) -> Self {
        let depth = 0;
        let points = Vec::new();
        let kdtree_nodes = Vec::new();
        return Self { _dim: dim, _depth: depth, _points: points, _kdtree_nodes: kdtree_nodes, _num_adds_count: 0 };
    }

    pub fn new_random(dim: usize, num_points: usize) -> Result<Self, String> {
        let mut out_self = Self::new_empty(dim);
        let mut sampler = RangeFloatVecSampler::new(-1.0, 1.0, dim);

        for _ in 0..num_points {
            let s = sampler.float_vec_sampler_sample()?;
            out_self.add_point(&s);
        }

        return Ok(out_self);
    }

    pub fn new_balanced(points: &Vec<DVector<f64>>) -> Result<Self, String> {
        if points.len() == 0 {
            return Err("points is empty".to_string());
        }

        let dim = points[0].len();

        let mut out_self = Self::new_empty(dim);

        let mut all_sorted_points = Vec::new(  );
        for i in 0..dim {
            let mut sorted_points = points.clone();
            sorted_points.sort_by(|x, y| x[i].partial_cmp(&y[i]).unwrap());
            all_sorted_points.push(sorted_points);
        }

        let mut dim_count = 0;
        while all_sorted_points[0].len() > 0 {
            let split_point = all_sorted_points[dim_count][ (all_sorted_points[dim_count].len()/2) ].clone();
            out_self._add_initial_point(&split_point);

            for i in 0..dim {
                let binary_search_res = all_sorted_points[i].binary_search_by(|x| x[i].partial_cmp(&split_point[i]).unwrap());
                match binary_search_res {
                    Ok(j) => {
                        all_sorted_points[i].remove(j);
                    },
                    Err(j) => {
                        println!("ERROR: point wasn't found in new_balanced {:?} {:?}", dim_count, split_point);
                    }
                }
            }

            dim_count += 1;
            dim_count = dim_count % dim;
        }

        // println!("{:?}", out_self._kdtree_nodes.len());
        // println!("{:?}", out_self._points.len());

        Ok(out_self)
    }

    pub fn new_random_balanced(dim: usize, num_points: usize) -> Result<Self, String> {
        let mut points = Vec::new();
        let mut sampler = RangeFloatVecSampler::new(-1.0, 1.0, dim);

        for _ in 0..num_points {
            points.push( sampler.float_vec_sampler_sample()? );
        }

        return Ok(Self::new_balanced(&points).unwrap());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn rebalance(&mut self, num_adds_check: bool) -> Result<(), String> {
        if num_adds_check { if !(self._num_adds_count % 5 == 0) { return Ok(()) } }

        let mut balanced = Self::new_balanced(&self._points)?;

        self._kdtree_nodes = balanced._kdtree_nodes.clone();
        self._depth = balanced._depth;

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_point(&mut self, point: &DVector<f64>) -> Result<(), String> {
        if point.len() != self._dim {
            return Err(format!("point dim ({:?}) does not match KDTree dim ({:?})", point.len(), self._dim));
        }

        self._add_point_r(point, 0);
        // self.rebalance(false);
        self._num_adds_count += 1;

        Ok(())
    }

    fn _add_initial_point(&mut self, point: &DVector<f64>) -> Result<(), String> {
        if point.len() != self._dim {
            return Err(format!("point dim ({:?}) does not match KDTree dim ({:?})", point.len(), self._dim));
        }

        self._add_point_r(point, 0);
        self._num_adds_count += 1;

        Ok(())
    }

    fn _add_point_r(&mut self, point: &DVector<f64>, node_idx: usize) {
        if self._kdtree_nodes.len() == 0 {
            self._points.push(point.clone());
            self._kdtree_nodes.push( KDTreeNode::new(0, 0, None, None, None, None, None) );
            self._depth = 1;
            return;
        }

        let layer_depth = self._kdtree_nodes[node_idx].node_layer;
        let split_idx = self._get_split_idx(layer_depth);
        let point_ref = &self._points[ self._kdtree_nodes[node_idx].point_id ];
        let mut parent_split_idx = None;
        if self._kdtree_nodes[node_idx].parent_node_id.is_some() {
            parent_split_idx = Some(self._get_split_idx(self._kdtree_nodes[self._kdtree_nodes[node_idx].parent_node_id.unwrap()].node_layer));
        }

        if parent_split_idx.is_some() {
            if self._kdtree_nodes[node_idx].max_along_parent_split_dimension.is_none() || self._kdtree_nodes[node_idx].max_along_parent_split_dimension.unwrap() < point[parent_split_idx.unwrap()] {
                self._kdtree_nodes[node_idx].max_along_parent_split_dimension = Some(point[parent_split_idx.unwrap()]);
            }

            if self._kdtree_nodes[node_idx].min_along_parent_split_dimension.is_none() || self._kdtree_nodes[node_idx].min_along_parent_split_dimension.unwrap() > point[parent_split_idx.unwrap()] {
                self._kdtree_nodes[node_idx].min_along_parent_split_dimension = Some(point[parent_split_idx.unwrap()]);
            }
        }

        if point[split_idx] >= point_ref[split_idx] {
            if self._kdtree_nodes[node_idx].child2_node_id.is_none() {
                self._points.push(point.clone());
                let mut parent_split_idx = self._get_split_idx(self._kdtree_nodes[node_idx].node_layer);
                let extreme_along_parent_split_dimension = Some(point[parent_split_idx]);
                self._kdtree_nodes.push( KDTreeNode::new( self._points.len() - 1, self._kdtree_nodes[node_idx].node_layer + 1, Some(node_idx), None, None, extreme_along_parent_split_dimension, extreme_along_parent_split_dimension ) );
                self._kdtree_nodes[node_idx].child2_node_id = Some(self._kdtree_nodes.len() - 1);
                if self._depth <= self._kdtree_nodes[node_idx].node_layer + 1 {
                    self._depth += 1;
                }
                return;
            } else {
                return self._add_point_r(point, self._kdtree_nodes[node_idx].child2_node_id.unwrap());
            }
        } else {
            if self._kdtree_nodes[node_idx].child1_node_id.is_none() {
                self._points.push(point.clone());
                let mut parent_split_idx = self._get_split_idx(self._kdtree_nodes[node_idx].node_layer);
                let extreme_along_parent_split_dimension = Some(point[parent_split_idx]);
                self._kdtree_nodes.push( KDTreeNode::new( self._points.len() - 1, self._kdtree_nodes[node_idx].node_layer + 1, Some(node_idx), None, None, extreme_along_parent_split_dimension, extreme_along_parent_split_dimension ) );
                self._kdtree_nodes[node_idx].child1_node_id = Some(self._kdtree_nodes.len() - 1);
                if self._depth <= self._kdtree_nodes[node_idx].node_layer + 1 {
                    self._depth += 1;
                }
                return;
            } else {
                return self._add_point_r(point, self._kdtree_nodes[node_idx].child1_node_id.unwrap());
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_closest(&self, target_point: &DVector<f64>) -> Result<(usize, f64), String> {
        if self._points.len() == 0 {
            return Err("there are no points in the KDtree".to_string());
        }

        return Ok( self._get_closest_recursion(target_point, 0, None ));
    }

    fn _get_closest_recursion(&self, target_point: &DVector<f64>, node_idx: usize, best_so_far: Option<(usize, f64)>) -> (usize, f64) {
        let mut _best_so_far = (usize::max_value(), std::f64::INFINITY);

        let query_point = &self._points[self._kdtree_nodes[node_idx].point_id];
        if best_so_far.is_some() {
            _best_so_far = best_so_far.unwrap();
        } else {
            _best_so_far = ( node_idx, (target_point - query_point).norm() );
        }

        let check_dis = (target_point - query_point).norm();
        if check_dis < _best_so_far.1 {
            _best_so_far = ( node_idx, (check_dis) );
        }

        let mut next_idx = None;
        let mut other_idx = None;
        let mut split_left = false;

        let split_idx = self._get_split_idx(self._kdtree_nodes[node_idx].node_layer);
        if target_point[split_idx] >= query_point[split_idx] {
            next_idx = self._kdtree_nodes[node_idx].child2_node_id;
            other_idx = self._kdtree_nodes[node_idx].child1_node_id;
        } else {
            split_left = true;
            next_idx = self._kdtree_nodes[node_idx].child1_node_id;
            other_idx = self._kdtree_nodes[node_idx].child2_node_id;
        }

        if next_idx.is_some() {
            let next_idx_unwrap = next_idx.unwrap();
            let tmp = self._get_closest_recursion(target_point, next_idx_unwrap, Some(_best_so_far));
            _best_so_far = tmp;
        }

        if other_idx.is_some() {
            let other_idx_unwrap = other_idx.unwrap();
            let mut extreme_along_parent_split_dimension_option = self._kdtree_nodes[other_idx_unwrap].min_along_parent_split_dimension;
            if !split_left {
                extreme_along_parent_split_dimension_option = self._kdtree_nodes[other_idx_unwrap].max_along_parent_split_dimension;
            }
            if extreme_along_parent_split_dimension_option.is_some() {
                let extreme_along_parent_split_dimension = extreme_along_parent_split_dimension_option.unwrap();
                let distance_check_lower_bound = (extreme_along_parent_split_dimension - target_point[split_idx]).powi(2).sqrt();
                if distance_check_lower_bound <= _best_so_far.1 {
                    let tmp = self._get_closest_recursion(target_point, other_idx_unwrap, Some(_best_so_far));
                    _best_so_far = tmp;
                }
            }
        }

        return _best_so_far;
    }

    pub fn get_closest_brute_force(&self, target_point: &DVector<f64>) -> Result<(usize, f64), String> {
        if self._points.len() == 0 {
            return Err("there are no points in the KDtree".to_string());
        }

        let mut best_so_far = (usize::max_value(), std::f64::INFINITY);
        let l = self._kdtree_nodes.len();
        for i in 0..l {
            let query_point = &self._points[self._kdtree_nodes[i].point_id];
            let dis = (query_point - target_point).norm();
            if dis < best_so_far.1 {
                best_so_far = (i, dis);
            }
        }
        return Ok(best_so_far);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_closest_k(&self, target_point: &DVector<f64>, k: usize) -> Vec<(usize, f64)> {
        let mut out_vec = vec![ (usize::max_value(), std::f64::INFINITY); k ];

        self._get_closest_k_recursion(target_point, 0, &mut out_vec);

        let l = out_vec.len();
        for i in 0..l {
            if out_vec[ l - i - 1 ].0 == usize::max_value() {
                out_vec.remove(l - i - 1);
            } else {
                return out_vec;
            }
        }

        out_vec
    }

    fn _get_closest_k_recursion(&self, target_point: &DVector<f64>, node_idx: usize, best_so_far: &mut Vec<(usize, f64)>) {
        let mut cutoff = best_so_far[best_so_far.len() - 1];

        let query_point = &self._points[self._kdtree_nodes[node_idx].point_id];
        let check_dis = (target_point - query_point).norm();
        if check_dis < cutoff.1 {
            let binary_search_res = best_so_far.binary_search_by(|x| x.1.partial_cmp(&check_dis).unwrap());
            match binary_search_res {
                Ok(i) => {
                    if !(node_idx == best_so_far[i + 1].0) {
                        best_so_far.insert(i, (node_idx, check_dis));
                        best_so_far.remove(best_so_far.len() - 1);
                        cutoff = best_so_far[best_so_far.len() - 1];
                    }
                },
                Err(i) => {
                    best_so_far.insert(i, (node_idx, check_dis));
                    best_so_far.remove(best_so_far.len() - 1);
                    cutoff = best_so_far[best_so_far.len() - 1];
                }
            }
        }

        let mut next_idx = None;
        let mut other_idx = None;
        let mut left = false;

        let split_idx = self._get_split_idx(self._kdtree_nodes[node_idx].node_layer);
        if target_point[split_idx] >= query_point[split_idx] {
            next_idx = self._kdtree_nodes[node_idx].child2_node_id;
            other_idx = self._kdtree_nodes[node_idx].child1_node_id;
        } else {
            left = true;
            next_idx = self._kdtree_nodes[node_idx].child1_node_id;
            other_idx = self._kdtree_nodes[node_idx].child2_node_id;
        }

        if next_idx.is_some() {
            let next_idx_unwrap = next_idx.unwrap();
            self._get_closest_k_recursion(target_point, next_idx_unwrap, best_so_far);
            cutoff = best_so_far[best_so_far.len() - 1];
        }

        if other_idx.is_some() {
            let other_idx_unwrap = other_idx.unwrap();
            let mut extreme_along_parent_split_dimension_option = self._kdtree_nodes[other_idx_unwrap].min_along_parent_split_dimension;
            if !left {
                extreme_along_parent_split_dimension_option = self._kdtree_nodes[other_idx_unwrap].max_along_parent_split_dimension;
            }
            if extreme_along_parent_split_dimension_option.is_some() {
                let extreme_along_parent_split_dimension = extreme_along_parent_split_dimension_option.unwrap();
                let distance_check_lower_bound = (extreme_along_parent_split_dimension - target_point[split_idx]).powi(2).sqrt();
                if distance_check_lower_bound <= cutoff.1 {
                    self._get_closest_k_recursion(target_point, other_idx_unwrap, best_so_far);
                    cutoff = best_so_far[best_so_far.len() - 1];
                }
            }
        }
    }

    pub fn get_closest_k_brute_force(&self, target_point: &DVector<f64>, k: usize) -> Vec<(usize, f64)> {
        let mut out_vec = vec![ (usize::max_value(), std::f64::INFINITY); k ];

        let l = self._kdtree_nodes.len();
        for i in 0..l {
            let query_point = &self._points[self._kdtree_nodes[i].point_id];
            let dis = (query_point - target_point).norm();
            if dis < out_vec[out_vec.len() - 1].1 {
                let binary_search_res = out_vec.binary_search_by(|x| x.1.partial_cmp(&dis).unwrap());
                match binary_search_res {
                    Ok(j) => {
                        out_vec.insert(j, (i, dis));
                        out_vec.remove(out_vec.len() - 1);
                    },
                    Err(j) => {
                        out_vec.insert(j, (i, dis));
                        out_vec.remove(out_vec.len() - 1);
                    }
                }
            }
        }

        let l = out_vec.len();
        for i in 0..l {
            if out_vec[ l - i - 1 ].0 == usize::max_value() {
                out_vec.remove(l - i - 1);
            } else {
                return out_vec;
            }
        }

        return out_vec;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_all_within_distance_r(&self, target_point: &DVector<f64>, r: f64) -> Vec<(usize, f64)> {
        let mut out_vec = Vec::new();

        self._get_all_within_distance_r_recursion(target_point, r, 0, &mut out_vec);

        return out_vec;
    }

    fn _get_all_within_distance_r_recursion(&self, target_point: &DVector<f64>, r: f64, node_idx: usize, out_vec: &mut Vec<(usize, f64)>) {

        let query_point = &self._points[self._kdtree_nodes[node_idx].point_id];
        let check_dis = (target_point - query_point).norm();
        if check_dis <= r {
            let binary_search_res = out_vec.binary_search_by(|x| x.1.partial_cmp(&check_dis).unwrap());
            match binary_search_res {
                Ok(i) => {
                    if !(node_idx == out_vec[i + 1].0) {
                        out_vec.insert(i, (node_idx, check_dis));
                    }
                },
                Err(i) => {
                    out_vec.insert(i, (node_idx, check_dis));
                }
            }
        }

        let mut next_idx = None;
        let mut other_idx = None;
        let mut left = false;

        let split_idx = self._get_split_idx(self._kdtree_nodes[node_idx].node_layer);
        if target_point[split_idx] >= query_point[split_idx] {
            next_idx = self._kdtree_nodes[node_idx].child2_node_id;
            other_idx = self._kdtree_nodes[node_idx].child1_node_id;
        } else {
            left = true;
            next_idx = self._kdtree_nodes[node_idx].child1_node_id;
            other_idx = self._kdtree_nodes[node_idx].child2_node_id;
        }


        if next_idx.is_some() {
            let next_idx_unwrap = next_idx.unwrap();
            self._get_all_within_distance_r_recursion(target_point, r, next_idx_unwrap, out_vec);
        }

        if other_idx.is_some() {
            let other_idx_unwrap = other_idx.unwrap();
            let mut extreme_along_parent_split_dimension_option = self._kdtree_nodes[other_idx_unwrap].min_along_parent_split_dimension;
            if !left {
                extreme_along_parent_split_dimension_option = self._kdtree_nodes[other_idx_unwrap].max_along_parent_split_dimension;
            }
            if extreme_along_parent_split_dimension_option.is_some() {
                let extreme_along_parent_split_dimension = extreme_along_parent_split_dimension_option.unwrap();
                let distance_check_lower_bound = (extreme_along_parent_split_dimension - target_point[split_idx]).powi(2).sqrt();
                if distance_check_lower_bound <= r {
                    self._get_all_within_distance_r_recursion(target_point, r, other_idx_unwrap, out_vec);
                }
            }
        }

    }

    pub fn get_all_within_distance_r_brute_force(&self, target_point: &DVector<f64>, r: f64) -> Vec<(usize, f64)> {
        let mut out_vec: Vec<(usize, f64)> = Vec::new();

        let l = self._kdtree_nodes.len();
        for i in 0..l {
            let query_point = &self._points[self._kdtree_nodes[i].point_id];
            let dis = (query_point - target_point).norm();
            if dis < r {
                let binary_search_res = out_vec.binary_search_by(|x| x.1.partial_cmp(&dis).unwrap());
                match binary_search_res {
                    Ok(j) => {
                        out_vec.insert(j, (i, dis));
                    },
                    Err(j) => {
                        out_vec.insert(j, (i, dis));
                    }
                }
            }
        }

        return out_vec;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_point_ref(&self, idx: usize) -> Result<&DVector<f64>, String> {
        if idx >= self._points.len() {
            return Err(format!("idx {:?} is too big for number of points ({:?})", idx, self._points.len()));
        }

        return Ok(&self._points[idx]);
    }

    pub fn get_point_ref_from_tuple(&self, tuple: (usize, f64)) -> Result<&DVector<f64>, String> {
        return self.get_point_ref(tuple.0);
    }

    pub fn get_num_adds_count(&self) -> usize { return self._num_adds_count; }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print(&self) {
        println!("{:?}", self);
    }

    pub fn print_summary(&self) {
        let l = self._kdtree_nodes.len();
        for i in 0..l {
            println!("{}{}   node {:?} --->  {}", style::Bold, color::Fg(color::Blue), i, style::Reset);
            println!("{}      point: {:?} {}", color::Fg(color::LightWhite), self._points[self._kdtree_nodes[i].point_id].data.as_vec(), style::Reset);
            println!("{}      parent id: {:?} {}", color::Fg(color::LightWhite), self._kdtree_nodes[i].parent_node_id, style::Reset);
            println!("{}      child1 id: {:?} {}", color::Fg(color::LightWhite), self._kdtree_nodes[i].child1_node_id, style::Reset);
            println!("{}      child2 id: {:?} {}", color::Fg(color::LightWhite), self._kdtree_nodes[i].child2_node_id, style::Reset);
            println!("{}      layer: {:?} {}", color::Fg(color::LightWhite), self._kdtree_nodes[i].node_layer, style::Reset);
            println!("{}      max along parent split dimension: {:?} {}", color::Fg(color::LightWhite), self._kdtree_nodes[i].max_along_parent_split_dimension, style::Reset);
            println!("{}      min along parent split dimension: {:?} {}", color::Fg(color::LightWhite), self._kdtree_nodes[i].min_along_parent_split_dimension, style::Reset);
            println!("{}      split dimension: {:?} {}", color::Fg(color::LightWhite), self._get_split_idx(self._kdtree_nodes[i].node_layer), style::Reset);
        }
        println!("{}{}KDTree has {:?} nodes and has depth {:?} {}", style::Bold, color::Fg(color::LightCyan), self._kdtree_nodes.len(), self._depth, style::Reset);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _get_split_idx(&self, layer_depth: usize) -> usize {
        return layer_depth % self._dim;
    }
}