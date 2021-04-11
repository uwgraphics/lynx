extern crate lynx_lib;
use lynx_lib::path_planning::sprint::sprint_local::*;
use lynx_lib::utils::utils_path_planning::local_search::*;
use lynx_lib::utils::utils_collisions::collision_checker::*;
use lynx_lib::utils::utils_vars::prelude::*;
use lynx_lib::utils::utils_recorders::prelude::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use lynx_lib::utils::utils_path_planning::path_planner_result::PathPlannerResult;

fn main() -> Result<(), String> {
    let mut lynx_vars = LynxVarsGeneric::new_empty_parallel(None);
    let recorder = RecorderArcMutexOption::new_none();
    let mut terminate = TerminationUtilOption::new_none();

    let c = CollisionCheckerBox::new( &SphereCollisionChecker::new(0.6, &vec![0.5, 0.5]) );

    let start = vec_to_dvec(&vec![0.,0.]);
    let goal = vec_to_dvec(&vec![1.,1.]);

    for i in 0..1 {
        let s = Instant::now();
        let mut sprint_local = SprintLocal::new(c.clone(), 0.02, false);
        let res = sprint_local.solve_local(&start, &goal, &mut lynx_vars, &recorder, &mut terminate)?;
        println!(">>{:?}", s.elapsed());
        println!("{:?}", res);
    }
    // println!("{:?}", res);
    Ok(())
}