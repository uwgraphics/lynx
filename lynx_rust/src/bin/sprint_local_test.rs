extern crate lynx_lib;
use lynx_lib::path_planning::sprint::sprint_local_search::SprintLocalSearch;
use nalgebra::DVector;
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;

fn main() -> Result<(), String> {
    let mut collision_checker = SphereCollisionChecker::new(0.4, &vec![0.5, 0.5]);

    let mut collision_checker = ImageEnvironmentCollisionChecker::new("convexSquare".to_string());

    let q_init = vec_to_dvec(&vec![0.5,0.01]);
    let q_goal = vec_to_dvec(&vec![0.5, 0.9]);

    let res = SprintLocalSearch::solve(&q_init, &q_goal, &mut collision_checker, 0.01, false, true, None)?;

    println!("{:?}", res);

    res.output_recorder_to_file("sprint".to_string(), "point2D".to_string(), "convexSquare".to_string());

    Ok(())
}