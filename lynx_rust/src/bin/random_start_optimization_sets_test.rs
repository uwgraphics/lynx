extern crate lynx_lib;
use lynx_lib::utils::utils_optimization::{nonlinear_optimization_engine::*, optimization_model_prefabs::*, optimization_model_parallel::*, optimization_model::OptimizationModel, random_start_optimization_sets::*};
use lynx_lib::utils::utils_sampling::sampler::*;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use nalgebra::Vector3;
use std::time::Instant;

fn main() -> Result<(), String> {
    let mut prefab = RelaxedIKOptimizationModelPrefab::new_base("ur5", None, None)?;
    prefab.add_link_pose_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,0.,0.,0.,0.]), 0, Some(ImplicitDualQuaternion::new_from_euler_angles(0.,0.,0., Vector3::new(-0.5,0., -0.5))));

    let mut model_parallel = OptimizationModelParallel::new(&prefab.optimization_model, None);

    let robot_module_toolbox = RobotModuleToolbox::new("ur5", None, None)?;
    let s =robot_module_toolbox.get_bounds_module_ref().as_thread_sampler();
    let nonlinear_optimization_engine = NLoptNonlinearOptimizationEngine::new_slsqp();
    let start = Instant::now();
    let r = get_random_restart_optimized_set(&mut prefab.optimization_model, &nonlinear_optimization_engine, s, 4800, Some(1000), None)?;
    // let r = get_random_restart_optimized_set_parallel(&mut model_parallel, &nonlinear_optimization_engine, s, 400, Some(1000), None)?;
    println!("{:?}", start.elapsed());

    // let r = get_random_restart_optimized_set_parallel(&mut model_parallel, &nonlinear_optimization_engine, s, 200, Some(1000), None)?;
    for i in 0..r.len() {
        // println!("{:?}, {:?}, {:?}", i, r[i].get_f_val(), r[i].get_x_star());
    }

    Ok(())
}