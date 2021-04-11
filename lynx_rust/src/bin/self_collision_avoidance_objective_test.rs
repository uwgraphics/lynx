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
    let init_state = vec_to_dvec(&vec![0.,0.,0.,0.,-1.7,0.,0.,0.]);

    let mut p = RelaxedIKOptimizationModelPrefab::new_base("sawyer", None, None)?;
    p.add_self_collision_avoidance_objective();
    p.set_initial_condition(init_state.clone());

    // p.optimization_model.print_diagnostics_with_default_nonlinear_optimization_engine_check(&Some(init_state.data.as_vec().clone()));

    let mut lynx_vars = p.optimization_model.lynx_vars.clone();
    let mut robot_module_toolbox = p.optimization_model.robot_module_toolbox.clone();
    let mut objective_function = p.optimization_model.objective_function.clone();

    let o = objective_function.call(&init_state, &mut lynx_vars, &mut robot_module_toolbox, true)?;

    o.print_summary();

    let g = objective_function.gradient(&init_state, &mut lynx_vars, &mut robot_module_toolbox, true)?;
    g.print_summary();

    Ok(())
}