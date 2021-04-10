extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::local_search::*;
use lynx_lib::utils::utils_collisions::collision_checker::*;
use lynx_lib::utils::utils_vars::prelude::*;
use lynx_lib::utils::utils_recorders::prelude::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_runtime_management::termination_util::TerminationUtilOption;

fn main() -> Result<(), String> {
    let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
    let recorder = RecorderArcMutexOption::new_none();
    let mut terminate = TerminationUtilOption::new_none();

    let c = CollisionCheckerBox::new( &SphereCollisionChecker::new(0.04, &vec![0.5, 0.5]) );

    let start = vec_to_dvec(&vec![0.,0.]);
    let goal = vec_to_dvec(&vec![1.,1.]);

    let l = StraightLineLocalSearch::new(c.clone(), 0.01);

    let res = l.solve_local(&start, &goal, &mut lynx_vars, &recorder, &mut terminate)?;

    println!("{:?}", res);

    Ok(())
}