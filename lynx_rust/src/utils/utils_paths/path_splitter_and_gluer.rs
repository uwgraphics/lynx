use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use nalgebra::DVector;
use crate::utils::utils_math::nalgebra_utils::vec_to_dvec;

pub fn split_concatenated_optimization_vector_into_linear_spline_path(vector: &DVector<f64>, point_dimension: usize) -> Result<LinearSplinePath, String> {
    let l = vector.len();
    let m = l % point_dimension;
    if m != 0 {
        return Err(format!("concanetated optimization vector of length {:?} could not be split into linear spline with point dimension of {:?}", l, m));
    }

    let mut out_linear_spline_path = LinearSplinePath::new_empty();

    let mut curr_waypoint = Vec::new();
    for i in 0..l {

        curr_waypoint.push( vector[i] );
        if curr_waypoint.len() == point_dimension {
            out_linear_spline_path.add_waypoint( &vec_to_dvec(&curr_waypoint) );
            curr_waypoint = Vec::new();
        }

    }

    return Ok(out_linear_spline_path);
}

pub fn glue_linear_spline_path_into_concatenated_optimization_vector( linear_spline_path: &LinearSplinePath ) -> Result<DVector<f64>, String> {
    let mut out_vec = Vec::new();

    let l = linear_spline_path.waypoints.len();
    for i in 0..l {
        let point_dim = linear_spline_path.waypoints[i].data.len();
        for j in 0..point_dim {
            out_vec.push(linear_spline_path.waypoints[i][j]);
        }
    }

    return Ok(vec_to_dvec(&out_vec));
}