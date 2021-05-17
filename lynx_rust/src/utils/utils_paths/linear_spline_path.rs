use nalgebra::{DVector, min};
use crate::utils::utils_math::nalgebra_utils::vec_to_dvec;
use crate::utils::utils_math::interpolation_utils::{get_linear_interpolation_with_stepsize};
use crate::utils::utils_paths::arclength_parameterization_util::ArclengthParameterizationUtil;

/*
#[derive(Debug, Clone)]
pub struct LinearSplinePath {
    pub waypoints: Vec<DVector<f64>>,
}

impl LinearSplinePath {
    pub fn new(waypoints: Vec<DVector<f64>>) -> Self {
        return Self {waypoints}
    }

    pub fn new_from_vecs(waypoints: Vec<Vec<f64>>) -> Self {
        let mut waypoints_ = waypoints.iter().map(|x| vec_to_dvec(x)).collect();
        return Self::new(waypoints_);
    }

    pub fn new_empty() -> Self {
        let waypoints = Vec::new();
        return Self::new(waypoints);
    }

    pub fn new_linear_interpolation(start_state: &DVector<f64>, end_state: &DVector<f64>, step_size: f64) -> Self {
        let waypoints =get_linear_interpolation_with_stepsize(start_state, end_state, step_size);
        return Self::new(waypoints);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_waypoint(&mut self, waypoint: &DVector<f64>) {
        self.waypoints.push( waypoint.clone() );
    }

    pub fn get_num_waypoints(&self) -> usize {
        return self.waypoints.len();
    }

    pub fn combine_ordered(&self, successor_linear_spline_path: &LinearSplinePath) -> Result<LinearSplinePath, f64> {
        /*  returns the compined spline or the closest distance that the path end-points get from each other as an error  */

        let res = self._easy_base_case_combines(successor_linear_spline_path);
        if res.is_some() { return res.unwrap(); }

        return self._combine_with_connection_point_idxs( successor_linear_spline_path, self.get_num_waypoints()-1, 0 );
    }

    pub fn combine_with_calculated_order(&self, other: &LinearSplinePath) -> Result<LinearSplinePath, f64> {
        /*  returns the compined spline or the closest distance that the path end-points get from each other as an error  */

        let res = self._easy_base_case_combines(other);
        if res.is_some() { return res.unwrap(); }

        let mut min_dis = std::f64::INFINITY;

        let d = (&self.waypoints[self.get_num_waypoints()-1] - &other.waypoints[0]).norm();
        if d < 0.000001 { return self._combine_with_connection_point_idxs(other, self.get_num_waypoints()-1, 0); }
        if d < min_dis { min_dis = d };

        let d = (&self.waypoints[0] - &other.waypoints[other.get_num_waypoints()-1]).norm();
        if d < 0.000001 { return self._combine_with_connection_point_idxs(other, 0, other.get_num_waypoints()-1); }
        if d < min_dis { min_dis = d };

        let d = (&self.waypoints[self.get_num_waypoints()-1] - &other.waypoints[other.get_num_waypoints()-1]).norm();
        if d < 0.000001 { return self._combine_with_connection_point_idxs(other, self.get_num_waypoints()-1, other.get_num_waypoints()-1); }
        if d < min_dis { min_dis = d };

        let d = (&self.waypoints[0] - &other.waypoints[0]).norm();
        if d < 0.000001 { return self._combine_with_connection_point_idxs(other, 0, 0); }
        if d < min_dis { min_dis = d };

        return Err(min_dis);
    }

    pub fn reverse(&mut self) {
        self.waypoints.reverse();
    }

    fn _combine_with_connection_point_idxs(&self, other: &LinearSplinePath, self_connection_idx: usize, other_connection_idx: usize) -> Result<LinearSplinePath, f64> {
        let dis = (&self.waypoints[self_connection_idx] - &other.waypoints[other_connection_idx]).norm();
        if dis < 0.000001 {
            let mut waypoints = Vec::new();

            if self_connection_idx > other_connection_idx && other_connection_idx == 0 {

                if self.get_num_waypoints() == 1 && self_connection_idx == 0 {
                    waypoints.push( self.waypoints[0].clone() );
                } else {
                    for i in 0..self_connection_idx {
                        waypoints.push(self.waypoints[i].clone());
                    }
                }

                if other.get_num_waypoints() >= other_connection_idx+1 {
                    for i in other_connection_idx+1..other.get_num_waypoints() {
                        waypoints.push(other.waypoints[i].clone());
                    }
                }

            } else if other_connection_idx > self_connection_idx && self_connection_idx == 0 {

                if other.get_num_waypoints() == 1 && other_connection_idx == 0 {
                    waypoints.push( other.waypoints[0].clone() );
                } else {
                    for i in 0..other_connection_idx {
                        waypoints.push(other.waypoints[i].clone());
                    }
                }

                if self.get_num_waypoints() >= self_connection_idx+1 {
                    for i in self_connection_idx+1..self.get_num_waypoints() {
                        waypoints.push(self.waypoints[i].clone());
                    }
                }
            } else if self_connection_idx == 0 && other_connection_idx == 0 {

                if self.get_num_waypoints() == 1 {
                    waypoints.push(self.waypoints[0].clone());
                } else {
                    for i in 0..self.get_num_waypoints() {
                        waypoints.push( self.waypoints[ i ].clone() );
                    }
                }

                waypoints.reverse();

                if other.get_num_waypoints() >= other_connection_idx+1 {
                    for i in other_connection_idx+1..other.get_num_waypoints() {
                        waypoints.push(other.waypoints[i].clone());
                    }
                }

            } else if self_connection_idx == self.get_num_waypoints()-1 && other_connection_idx == other.get_num_waypoints()-1 {

                if self.get_num_waypoints() == 1 {
                    waypoints.push(self.waypoints[0].clone());
                } else {
                    for i in 0..self.get_num_waypoints() {
                        waypoints.push( self.waypoints[ i ].clone() );
                    }
                }

                if other.get_num_waypoints() >= 2 {
                    for i in 1..other.get_num_waypoints() {
                        waypoints.push(other.waypoints[other.get_num_waypoints()-1-i].clone());
                    }
                }

            }

            return Ok( Self::new(waypoints) );
        } else {
            return Err(dis);
        }
    }

    fn _easy_base_case_combines(&self, other: &LinearSplinePath) -> Option<Result<LinearSplinePath, f64>> {
        if self.get_num_waypoints() == 0 && other.get_num_waypoints() == 0 { return Some(Err(std::f64::INFINITY)); }
        if self.get_num_waypoints() == 0 { return Some(Ok( other.clone() )); }
        if other.get_num_waypoints() == 0 { return Some(Ok(self.clone())); }

        return None;
    }
}
*/


#[derive(Debug, Clone)]
pub struct LinearSplinePath {
    pub waypoints: Vec<DVector<f64>>,
    _arclength_parameterization_util: Option<ArclengthParameterizationUtil>
}

impl LinearSplinePath {
    pub fn new(waypoints: Vec<DVector<f64>>) -> Self {
        let _arclength_parameterization_util = None;
        return Self {
            waypoints,
            _arclength_parameterization_util
        }
    }

    pub fn new_from_vecs(waypoints: Vec<Vec<f64>>) -> Self {
        let mut waypoints_ = waypoints.iter().map(|x| vec_to_dvec(x)).collect();
        return Self::new(waypoints_);
    }

    pub fn new_empty() -> Self {
        let waypoints = Vec::new();
        return Self::new(waypoints);
    }

    pub fn new_linear_interpolation(start_state: &DVector<f64>, end_state: &DVector<f64>, step_size: f64) -> Self {
        let waypoints =get_linear_interpolation_with_stepsize(start_state, end_state, step_size);
        return Self::new(waypoints);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_waypoint(&mut self, waypoint: &DVector<f64>) {
        self.waypoints.push( waypoint.clone() );
        self._arclength_parameterization_util = None;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_arclength_interpolated_point(&mut self, u: f64) -> DVector<f64> {
        if self._arclength_parameterization_util.is_none() {
            self._arclength_parameterization_util = Some(ArclengthParameterizationUtil::new(&self));
        }

        return self._arclength_parameterization_util.as_ref().unwrap().get_arclength_interpolated_point(&self, u);
    }

    pub fn get_num_waypoints(&self) -> usize {
        return self.waypoints.len();
    }

    pub fn combine_ordered(&self, successor_linear_spline_path: &LinearSplinePath) -> Result<LinearSplinePath, String> {
        /*  returns the compined spline or the closest distance that the path end-points get from each other as an error  */

        let res = self._easy_base_case_combines(successor_linear_spline_path);
        if res.is_some() { return res.unwrap(); }

        return self._combine_with_connection_point_idxs( successor_linear_spline_path, self.get_num_waypoints()-1, 0 );
    }

    pub fn combine_with_calculated_order(&self, other: &LinearSplinePath) -> Result<LinearSplinePath, String> {
        /*  returns the compined spline or the closest distance that the path end-points get from each other as an error  */

        let res = self._easy_base_case_combines(other);
        if res.is_some() { return res.unwrap(); }

        let mut min_dis = std::f64::INFINITY;

        let d = (&self.waypoints[self.get_num_waypoints()-1] - &other.waypoints[0]).norm();
        if d < 0.000001 { return self._combine_with_connection_point_idxs(other, self.get_num_waypoints()-1, 0); }
        if d < min_dis { min_dis = d };

        let d = (&self.waypoints[0] - &other.waypoints[other.get_num_waypoints()-1]).norm();
        if d < 0.000001 { return self._combine_with_connection_point_idxs(other, 0, other.get_num_waypoints()-1); }
        if d < min_dis { min_dis = d };

        let d = (&self.waypoints[self.get_num_waypoints()-1] - &other.waypoints[other.get_num_waypoints()-1]).norm();
        if d < 0.000001 { return self._combine_with_connection_point_idxs(other, self.get_num_waypoints()-1, other.get_num_waypoints()-1); }
        if d < min_dis { min_dis = d };

        let d = (&self.waypoints[0] - &other.waypoints[0]).norm();
        if d < 0.000001 { return self._combine_with_connection_point_idxs(other, 0, 0); }
        if d < min_dis { min_dis = d };

        return Err(format!("paths cannot be combined.  Minimum distance between paths ({:?}) is too large.", min_dis));
    }

    pub fn reverse(&mut self) {
        self.waypoints.reverse();
    }

    fn _combine_with_connection_point_idxs(&self, other: &LinearSplinePath, self_connection_idx: usize, other_connection_idx: usize) -> Result<LinearSplinePath, String> {
        let dis = (&self.waypoints[self_connection_idx] - &other.waypoints[other_connection_idx]).norm();
        if dis < 0.000001 {
            let mut waypoints = Vec::new();

            if self_connection_idx > other_connection_idx && other_connection_idx == 0 {

                if self.get_num_waypoints() == 1 && self_connection_idx == 0 {
                    waypoints.push( self.waypoints[0].clone() );
                } else {
                    for i in 0..self_connection_idx+1 {
                        waypoints.push(self.waypoints[i].clone());
                    }
                }

                if other.get_num_waypoints() >= other_connection_idx+1 {
                    for i in other_connection_idx+1..other.get_num_waypoints() {
                        waypoints.push(other.waypoints[i].clone());
                    }
                }

            } else if other_connection_idx > self_connection_idx && self_connection_idx == 0 {

                if other.get_num_waypoints() == 1 && other_connection_idx == 0 {
                    waypoints.push( other.waypoints[0].clone() );
                } else {
                    for i in 0..other_connection_idx+1 {
                        waypoints.push(other.waypoints[i].clone());
                    }
                }

                if self.get_num_waypoints() >= self_connection_idx+1 {
                    for i in self_connection_idx+1..self.get_num_waypoints() {
                        waypoints.push(self.waypoints[i].clone());
                    }
                }
            } else if self_connection_idx == 0 && other_connection_idx == 0{

                if self.get_num_waypoints() == 1 {
                    waypoints.push(self.waypoints[0].clone());
                } else {
                    for i in 0..self.get_num_waypoints() {
                        waypoints.push( self.waypoints[ i ].clone() );
                    }
                }

                waypoints.reverse();

                if other.get_num_waypoints() >= other_connection_idx+1 {
                    for i in other_connection_idx+1..other.get_num_waypoints() {
                        waypoints.push(other.waypoints[i].clone());
                    }
                }

            } else if self_connection_idx == self.get_num_waypoints()-1 && other_connection_idx == other.get_num_waypoints()-1 {

                if self.get_num_waypoints() == 1 {
                    waypoints.push(self.waypoints[0].clone());
                } else {
                    for i in 0..self.get_num_waypoints() {
                        waypoints.push( self.waypoints[ i ].clone() );
                    }
                }

                if other.get_num_waypoints() >= 2 {
                    for i in 1..other.get_num_waypoints() {
                        waypoints.push(other.waypoints[other.get_num_waypoints()-1-i].clone());
                    }
                }

            }

            return Ok( Self::new(waypoints) );
        } else {
            return Err(format!("paths cannot be combined.  Distance between paths at given connection point ({:?}) is too large.", dis));
        }
    }

    fn _easy_base_case_combines(&self, other: &LinearSplinePath) -> Option<Result<LinearSplinePath, String>> {
        if self.get_num_waypoints() == 0 && other.get_num_waypoints() == 0 { return Some(Err("paths cannot be combined.  Both paths have length of zero.".to_string())); }
        if self.get_num_waypoints() == 0 { return Some(Ok( other.clone() )); }
        if other.get_num_waypoints() == 0 { return Some(Ok(self.clone())); }

        return None;
    }
}

