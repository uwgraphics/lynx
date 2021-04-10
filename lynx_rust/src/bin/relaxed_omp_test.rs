extern crate lynx_lib;
use lynx_lib::optimal_motion_planning::prelude::*;


fn main() -> Result<(), String> {
    let mut start_boundary = RelaxedIKOptimizationModelPrefab::new_base("r1", "sawyer", None, None)?;
    start_boundary.add_link_pose_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, None, None);

    let mut end_boundary = RelaxedIKOptimizationModelPrefab::new_base("r1", "sawyer", None, None)?;
    end_boundary.add_link_pose_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, Some(ImplicitDualQuaternion::new_from_euler_angles(0.,0.,0., Vector3::new(-0.9,-0.2,0.5))), None);

    let mut pathwise = RelaxedIKOptimizationModelPrefab::new_base("r1", "sawyer", None, None)?;
    pathwise.add_joint_acceleration_minimizing_objective( &None, Some(0.6) );
    pathwise.add_joint_jerk_minimizing_objective( &None, Some(1.7) );

    let mut pointwise = RelaxedIKOptimizationModelPrefab::new_base("r1", "sawyer", None, None)?;
    pointwise.add_link_orientation_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, None, None);

    let mut path_optimization_prefab = PathOptimizationPrefab::new(start_boundary.optimization_model, end_boundary.optimization_model, pointwise.optimization_model, pathwise.optimization_model, 8, Some(12), Some(12))?;

    let nonlinear_optimization_engine = NLoptNonlinearOptimizationEngine::new_slsqp();
    let robot_module_toolbox = RobotModuleToolbox::new("sawyer", None, None)?;
    let bounds_module = robot_module_toolbox.get_bounds_module_ref().clone();
    let sampler = bounds_module.as_thread_sampler();
    let collision_checker = RobotCollisionChecker;

    let res = RelaxedOMP::solve( &mut path_optimization_prefab, &nonlinear_optimization_engine, sampler, &Some(&collision_checker), 500, 500, Some(400), None, &None, &None, &None, Some(2.0), 0.02, true, false)?;

    // res.output_multiple_paths_to_file(vec![0,50,13,25,res.get_solutions_ref().len()-1], "relaxed_omp", "sawyer", "test");
    // res.output_path_optimization_solution_cost_timeline_to_file("relaxed_omp", "sawyer", "test");

    Ok(())
}