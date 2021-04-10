use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::utils::utils_math::interpolation_utils::*;
use nalgebra::DVector;

pub fn resample_path(original_path: &Vec<DVector<f64>>, target_number_of_waypoints: usize) -> Vec<DVector<f64>> {
    let num_waypoints_in_original_path = original_path.len();
    let mut out_path = Vec::new();

    let mut distances_along_path = vec![0.0];
    for i in 1..num_waypoints_in_original_path {
        let curr_dis = (&original_path[i] - &original_path[i-1]).norm();
        distances_along_path.push(curr_dis + distances_along_path[i-1]);
    }
    let mut total_distance = distances_along_path[distances_along_path.len() - 1];

    let mut arc_length_markers = get_linear_interpolation_with_stepsize(&DVector::from_element(1, 0.0), &DVector::from_element(1, total_distance), total_distance / (target_number_of_waypoints - 1) as f64);
    if arc_length_markers.len() > target_number_of_waypoints && arc_length_markers.len() >= 2 {
        arc_length_markers.remove(arc_length_markers.len() - 2);
    }

    let l = arc_length_markers.len();
    for i in 0..l {
        let binary_search_res = distances_along_path.binary_search_by(|x| x.partial_cmp(&arc_length_markers[i][0]).unwrap());
        match binary_search_res {
            Ok(j) => {
                out_path.push( original_path[j].clone() );
            },
            Err(j) => {
                let upper_bound_idx = j;
                let lower_bound_idx = j-1;

                let ratio = (arc_length_markers[i][0] - distances_along_path[lower_bound_idx]) / (distances_along_path[upper_bound_idx] - distances_along_path[lower_bound_idx]);

                out_path.push( &original_path[lower_bound_idx] + ratio * ( &original_path[upper_bound_idx] - &original_path[lower_bound_idx]  ) );
            }
        }
    }

    return out_path;
}

pub fn resample_linear_spline_path(original_path: &LinearSplinePath, target_number_of_waypoints: usize) -> LinearSplinePath {
    let out_dvecs = resample_path(&original_path.waypoints, target_number_of_waypoints);
    return LinearSplinePath::new(out_dvecs);
}


