use crate::utils::utils_math::vector_utils::*;
use nalgebra::{DVector};
use crate::utils::utils_math::interpolation_utils::*;

pub fn get_points_around_circle(center_point: &DVector<f64>, rotation_axis: &DVector<f64>, circle_radius: f64, num_samples: usize) -> Result<Vec<DVector<f64>>, String> {
    let dim = center_point.len();

    let basis = get_orthonormal_basis(rotation_axis, 3)?;

    let mut local_x = basis[1].clone();
    let mut local_y = basis[2].clone();

    let step_size = (2.0*std::f64::consts::PI / (num_samples as f64));
    let range = get_range(0.0, 2.0*std::f64::consts::PI, step_size);

    let mut out_points: Vec<DVector<f64>> = Vec::new();

    let l = range.len();
    for i in 0..l {
        let point = circle_radius * range[i].cos() * &local_x + circle_radius * range[i].sin() * &local_y;
        out_points.push(&point + center_point);
    }

    return Ok(out_points);
}