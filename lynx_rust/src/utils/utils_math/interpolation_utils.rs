use nalgebra::{DVector, Vector3};
use crate::utils::utils_math::nalgebra_utils::vec_to_dvec;

pub fn get_linear_interpolation_with_stepsize(start_pt: &DVector<f64>, goal_pt: &DVector<f64>, step_size: f64) -> Vec<DVector<f64>> {
    let mut out_vec: Vec<DVector<f64>> = Vec::new();
    let mut curr_pt = start_pt.clone();

    out_vec.push(start_pt.clone());
    while !((&curr_pt - goal_pt).norm() < step_size) {
        let n = (goal_pt - &curr_pt).norm();
        curr_pt = &curr_pt + step_size* ((goal_pt - &curr_pt)/n);
        out_vec.push(curr_pt.clone());
    }
    out_vec.push(goal_pt.clone());

    return out_vec;
}

pub fn get_linear_interpolation_with_stepsize_vec3(start_pt: &Vector3<f64>, goal_pt: &Vector3<f64>, step_size: f64) -> Vec<Vector3<f64>> {
    let mut out_vec: Vec<Vector3<f64>> = Vec::new();
    let mut curr_pt = start_pt.clone();

    out_vec.push(start_pt.clone());
    while !((&curr_pt - goal_pt).norm() < step_size) {
        let n = (goal_pt - &curr_pt).norm();
        curr_pt = &curr_pt + step_size* ((goal_pt - &curr_pt)/n);
        out_vec.push(curr_pt.clone());
    }
    out_vec.push(goal_pt.clone());

    return out_vec;
}

pub fn get_linear_interpolation_with_stepsize_vecs(start_pt: &Vec<f64>, goal_pt: &Vec<f64>, step_size: f64) -> Vec<DVector<f64>> {
    get_linear_interpolation_with_stepsize(&vec_to_dvec(start_pt), &vec_to_dvec(goal_pt), step_size)
}

pub fn get_linear_interpolation(start: f64, end: f64, u: f64) -> f64 {
    /*
    u is a value between 0.0 and 1.0 that will be used to interpolate between start and end
    */

    return (1.0 - u) * start + (u) * end;
}

pub fn get_linear_ramp_value_within_range(ramp_start: f64, ramp_end: f64, range_start: f64, range_end: f64, range_value: f64) -> f64 {
    /*
    ramp start and stop represent the "y axis" values that the output will be between
    range start and stop values represent the "x axis" values that the input will be between.
    ramp value is the particular value between range start and range stop used to output a linear ramp value
    */

    let u = (range_value - range_start) / (range_end - range_start);
    return get_linear_interpolation(ramp_start, ramp_end, u);
}

pub fn get_range(range_start: f64, range_stop: f64, step_size: f64) -> Vec<f64> {
    let mut out_range = Vec::new();
    out_range.push(range_start);
    let mut last_added_val = range_start;

    while !( (range_stop - last_added_val).abs() < step_size ) {
        if range_stop > range_start {
            last_added_val = last_added_val + step_size;
        } else {
            last_added_val = last_added_val - step_size;
        }
        out_range.push(last_added_val);
    }

    out_range.push(range_stop);

    out_range
}
