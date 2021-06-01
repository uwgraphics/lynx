use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::utils::utils_math::interpolation_utils::*;
use nalgebra::DVector;

#[derive(Clone, Debug)]
pub struct ArclengthParameterizationUtil {
    _distances_to_each_waypoint: Vec<f64>,
    _length_ratio_to_each_waypoint: Vec<f64>,
    _total_dis: f64
}

impl ArclengthParameterizationUtil {
    pub fn new(linear_spline_path: &LinearSplinePath) -> Self {
        let mut _distances_to_each_waypoint = Vec::new();
        let mut _length_ratio_to_each_waypoint = Vec::new();
        let mut _total_dis = 0.0;

        let waypoints_ref = &linear_spline_path.waypoints;
        let l = waypoints_ref.len();

        if l > 0 {
            _distances_to_each_waypoint.push(0.0);
        }

        if l > 1 {
            for i in 1..l {
                let dis = (&waypoints_ref[i] - &waypoints_ref[i-1]).norm();
                _distances_to_each_waypoint.push(dis + _distances_to_each_waypoint[i-1]);
            }
        }
        _total_dis = _distances_to_each_waypoint[l-1];

        for i in 0..l {
            _length_ratio_to_each_waypoint.push( _distances_to_each_waypoint[i] / _total_dis );
        }

        Self {
            _distances_to_each_waypoint,
            _length_ratio_to_each_waypoint,
            _total_dis
        }
    }

    pub fn get_arclength_interpolated_point(&self, linear_spline_path: &LinearSplinePath, u: f64) -> DVector<f64> {
        let mut u_ = u.max(0.0).min(1.0);

        let binary_search_res = self._length_ratio_to_each_waypoint.binary_search_by(|x| x.partial_cmp(&u_).unwrap());
        return match binary_search_res {
            Ok(i) => {
                linear_spline_path.waypoints[i].clone()
            }
            Err(i) => {
                let upper_boundary_length_ratio = self._length_ratio_to_each_waypoint[i];
                let lower_boundary_length_ratio = self._length_ratio_to_each_waypoint[i - 1];

                let ratio_between_boundaries = (u - lower_boundary_length_ratio) / (upper_boundary_length_ratio - lower_boundary_length_ratio);

                (1.0 - ratio_between_boundaries) * &linear_spline_path.waypoints[i - 1] + (ratio_between_boundaries) * &linear_spline_path.waypoints[i]
            }
        }
    }
}
