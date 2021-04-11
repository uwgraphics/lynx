extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::nearest_neighbor_utils::kd_tree_utils::KDTree;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::optimal_motion_planning::prelude::*;

fn main() {
    let mut kdtree = KDTree::new(2);

    // let res = kdtree.get_point_idxs_within_distance_r(&vec_to_dvec(&vec![0.,0.]), 2.0);
    // println!("{:?}", res.len());

    let res = kdtree.add(vec![0.,0.]);
    println!("{:?}", res);
}