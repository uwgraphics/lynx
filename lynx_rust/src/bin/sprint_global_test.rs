extern crate lynx_lib;
use lynx_lib::path_planning::sprint::sprint_local_search::SprintLocalSearch;
use lynx_lib::path_planning::sprint::sprint::Sprint;
use nalgebra::DVector;
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;
use lynx_lib::utils::utils_path_planning::utils_path_planning_sampling::{cspace_sampler::*};
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;

fn main() -> Result<(), String> {
    let mut collision_checker = ImageEnvironmentCollisionChecker::new("bugtrap5".to_string());
    let mut cspace_sampler = ImageEnvironmentCSpaceSampler::new_from_image_name("bugtrap5".to_string());

    let q_init = vec_to_dvec(&vec![0.5, 0.45]);
    let q_goal = vec_to_dvec(&vec![0.5, 1.327]);

    let res = Sprint::solve(&q_init, &q_goal, &mut collision_checker, &cspace_sampler, 0.008, false, true, None)?;

    println!("{:?}", res);

    res.output_recorder_to_file("sprint".to_string(), "point2D".to_string(), "bugtrap5".to_string());


    Ok(())
}