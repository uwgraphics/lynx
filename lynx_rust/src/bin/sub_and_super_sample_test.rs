extern crate lynx_lib;
use lynx_lib::utils::utils_math::interpolation_utils::*;
use lynx_lib::utils::utils_path_planning::utils_paths::linear_spline_path::LinearSplinePath;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_path_optimization::path_resample::*;

fn main() {
    let interp = get_linear_interpolation_with_stepsize(&vec_to_dvec(&vec![0.,0.]), &vec_to_dvec(&vec![1.,1.]), 0.01);
    println!("{:?}", interp.len());

    let new_path = resample_path(&interp, 8000);

    println!("{:?}", new_path);
}