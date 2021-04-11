extern crate lynx_lib;
use lynx_lib::utils::utils_recorders::recorder::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;

fn main() {
    let mut r = Recorder::new();
    r.action_add_collision_info("block".to_string(), None);
    r.action_add_edge( &vec_to_dvec(&vec![0.9, 0.9]), &vec_to_dvec(&vec![0.5, 0.5]), None );

    r.output_to_file("sprint".to_string(), "point2D".to_string(), "test".to_string());
}