extern crate lynx_lib;
use lynx_lib::utils::utils_optimization::{optimization_model_prefabs::*, optimization_model_parallel::OptimizationModelParallel};
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_robot_objective_specification::link_kinematic_objective_specification::LinkPoseMatchingSpecification;
use lynx_lib::utils::utils_robot_objective_specification::robot_salient_link_description::RobotSalientLinkDescription;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use lynx_lib::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use nalgebra::{Vector3};
use lynx_lib::utils::utils_sampling::sampler::*;
use std::time::Instant;
use rayon::prelude::*;

fn main() -> Result<(), String> {

    let init_state = vec_to_dvec(&vec![0.1; 8]);

    let mut p = RelaxedIKOptimizationModelPrefab::new_base("sawyer", Some("test"), None)?;
    p.add_link_pose_matching_objective_relative_to_state(&init_state, 0, Some(ImplicitDualQuaternion::new_from_euler_angles(0.,0.,0., Vector3::new(-0.5,0.,0.0))));
    // p.add_joint_velocity_minimizing_objective(&init_state);
    p.add_joint_acceleration_minimizing_objective(&init_state);
    p.add_joint_jerk_minimizing_objective(&init_state);
    p.add_self_collision_avoidance_objective();
    // p.add_environment_collision_avoidance_objective("floor");
    // p.add_self_collision_avoidance_objective();
    // p.set_initial_condition(init_state.clone());
    // p.add_pull_to_cspace_goal_objective(&vec_to_dvec(&vec![5.0; 8]), &init_state, 0.5);
    // p.add_link_position_matching_objective_relative_to_state( &init_state, 0, None );

    p.optimization_model.objective_function.print_summary();

    p.optimization_model.objective_function.remove_isolated_objective_term("robot_minimize_joint_jerk");


    p.optimization_model.objective_function.print_summary();

    // p.optimization_model.print_diagnostics_with_default_nonlinear_optimization_engine_check(&Some(init_state.data.as_vec().clone()));

    // let res = p.optimization_model.call_objective(&init_state, true)?;
    // res.print_summary();

    // p.optimization_model.objective_function.update_weight_via_idx(0, 20.0);
    // p.optimization_model.objective_function.update_weight_via_idx(1, 1.0);
    // p.optimization_model.objective_function.update_weight_via_idx(2, 1.0);

    // let res = p.optimization_model.optimize_slsqp_default(&init_state, false)?;
    // res.print_summary();

    /*
    let mut op = OptimizationModelParallel::new(&p.optimization_model, None);

    op.get_par_iter_mut().for_each(|x| {
       let res= x.optimize_open_default(&vec_to_dvec(&vec![0.,0.,0.,0.,0.,0.]), false);
        res.unwrap().print_summary();
    });
    */

    // let mut r = RobotModuleToolbox::new("sawyer", Some("test"), None)?;
    // r.get_dof_module_ref().print_joint_dof_order();

    Ok(())
}