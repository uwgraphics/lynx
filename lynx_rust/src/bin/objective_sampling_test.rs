extern crate lynx_lib;
use lynx_lib::utils::utils_optimization::optimization_model_prefabs::*;
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;
use lynx_lib::utils::utils_optimization::{random_start_optimization_sets::*, nonlinear_optimization_engine::*};
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_sampling::sampler::*;
use lynx_lib::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
// use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use nalgebra::Vector3;
use std::time::Instant;
use lynx_lib::utils::utils_optimization::optimization_model_pair::{OptimizationModelPair, OptimizationModelPairParallel, OptimizationModelPairType};
use rayon::prelude::*;

fn main() -> Result<(), String> {
    let mut r1 = RelaxedIKOptimizationModelPrefab::new_base("rik", "sawyer", Some("test"), None)?;
    r1.add_link_pose_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, None, None);
    r1.set_collision_environment("sawyer_vertical_bars");

    let mut r2 = RelaxedIKOptimizationModelPrefab::new_base("rik2","sawyer", Some("test"), None)?;
    r2.add_link_pose_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, Some(ImplicitDualQuaternion::new_from_euler_angles(0.,0.,0., Vector3::new(-0.5,0.,0.))), None);
    r2.set_collision_environment("sawyer_vertical_bars");

    let nonlinear_optimization_engine = NLoptNonlinearOptimizationEngine::new_slsqp();
    let b = r1.optimization_model.robot_module_toolbox.as_ref().unwrap().get_bounds_module_ref().clone();
    let sampler = b.as_thread_sampler();
    let collision_checker = RobotCollisionChecker;

    // let mut optimization_model_pair = OptimizationModelPair::new("test", r1.optimization_model, r2.optimization_model);

    let res = get_random_restart_optimized_set(&mut r2.optimization_model, &nonlinear_optimization_engine, sampler, &Some(&collision_checker), 200, 1000, true, Some(500), None)?;
    res.print_summary();
    // let res = get_random_restart_optimized_boundary_sets_with_warmstart_initial_conditions(&mut optimization_model_pair, &nonlinear_optimization_engine, sampler, &Some(&collision_checker), 200, 200, 1000, true, Some(400), None)?;
    // res[0].print_summary();
    // res[1].print_summary();
    res.output_to_file("relaxed_omp", "sawyer", "objective_sampling_test");

    /*
        let mut t = &mut OptimizationModelPairType::new_parallel("t", r1.optimization_model, r2.optimization_model, None);
        match t {
        OptimizationModelPairType::Parallel(o) => {
            let iter = o.get_par_iter_mut();
            iter.enumerate().for_each(|(idx, optimization_model_pair)| {
                println!("{:?}", optimization_model_pair.name);
            });
        }
        _ => {}
    }
    */
    /*
    let res = get_random_restart_optimized_boundary_sets_with_warmstart_initial_conditions_autotype(&mut t, &nonlinear_optimization_engine, sampler, Some(&collision_checker), 1500, 1500, 2000, true, Some(5000), None)?;

    /*
    let mut r1 = RelaxedIKOptimizationModelPrefab::new_base("sawyer", Some("test"), None)?;
    r1.add_link_pose_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, None, None);

    let mut r1p = r1.optimization_model.to_optimization_model_parallel(None);

    let mut r2 = RelaxedIKOptimizationModelPrefab::new_base("sawyer", Some("test"), None)?;
    r2.add_link_position_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, None, None);

    let mut r2p = r2.optimization_model.to_optimization_model_parallel(None);

    let nonlinear_optimization_engine = NLoptNonlinearOptimizationEngine::new_slsqp();
    let b = r1.optimization_model.robot_module_toolbox.as_ref().unwrap().get_bounds_module_ref().clone();
    let sampler = b.as_thread_sampler();
    let collision_checker = RobotCollisionChecker;


    // let res = get_random_restart_optimized_boundary_sets_with_warmstart_initial_conditions(&mut r1.optimization_model, &mut r2.optimization_model, &nonlinear_optimization_engine, sampler, Some(&collision_checker), 100, 100, 200, true, Some(1000), None)?;

    let res = get_random_restart_optimized_set(&mut r2.optimization_model, &nonlinear_optimization_engine, sampler, Some(&collision_checker), 1500, 2000, false, Some(500), None)?;

    res.print_summary();
    res.output_to_file("relaxed_omp_local", "sawyer", "randoms3");

    */

    res[0].print_summary();
    res[0].output_to_file("relaxed_omp_local", "sawyer", "randoms");

    res[1].print_summary();
    res[1].output_to_file("relaxed_omp_local", "sawyer", "randoms2");
    */

    Ok(())
}