extern crate lynx_lib;
use lynx_lib::optimal_motion_planning::prelude::*;


fn main() -> Result<(), String> {
    let mut start_boundary = RelaxedIKOptimizationModelPrefab::new_base("r1", "sawyer", Some("test"), None)?;
    start_boundary.add_link_pose_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, None, None);

    let mut end_boundary = RelaxedIKOptimizationModelPrefab::new_base("r1", "sawyer", Some("test"), None)?;
    end_boundary.add_link_pose_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, Some(ImplicitDualQuaternion::new_from_euler_angles(0.,0.,0., Vector3::new(-0.9,-0.2,0.5))), None);

    let mut pathwise = RelaxedIKOptimizationModelPrefab::new_base("r1", "sawyer", Some("test"), None)?;
    pathwise.add_joint_acceleration_minimizing_objective( &None, Some(0.4) );
    pathwise.add_joint_jerk_minimizing_objective( &None, Some(0.5) );

    let mut pointwise = RelaxedIKOptimizationModelPrefab::new_base("r1", "sawyer", Some("test"), None)?;
    pointwise.add_link_orientation_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, None, None);

    let mut path_optimization_prefab = PathOptimizationPrefab::new(start_boundary.optimization_model, end_boundary.optimization_model, pointwise.optimization_model, pathwise.optimization_model, 8, Some(12), Some(12))?;
    path_optimization_prefab.set_collision_environment("sawyer_vertical_bars");

    let mut experiment = RobotOMPExperiment::new(path_optimization_prefab, 0.02, "sawyer", "tester")?;

    experiment.run_experiment(OMPSolver::RelaxedOMP(Some(5.0), (500,500)), PathFileOutputMode::OutputRandomNPaths(20), true)?;

    Ok(())
}