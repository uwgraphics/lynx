extern crate lynx_lib;
use lynx_lib::utils::utils_optimization::{optimization_model_prefabs::*, optimization_model::*};
use lynx_lib::relaxed_omp::relaxed_omp_local::RelaxedOMPLocal;
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use std::time::Instant;

fn main() -> Result<(), String> {

    let q_init = vec_to_dvec(&vec![0.,0.,0.,0.,0.,0.,0.,0.]);
    let q_goal = vec_to_dvec(&vec![0.,0.,-1.57,0.,0.,0.,1.57,0.]);

    let mut pathwise_objective_prefab = RelaxedIKOptimizationModelPrefab::new_base("test" ,"sawyer", Some("test"), None)?;
    pathwise_objective_prefab.add_joint_acceleration_minimizing_objective(&q_init, Some(0.1));
    pathwise_objective_prefab.add_joint_jerk_minimizing_objective(&q_init, Some(0.1));
    let coll = RobotCollisionChecker;

    let mut pointwise_objective_prefab = RelaxedIKOptimizationModelPrefab::new_base("test2", "sawyer", Some("test"), None)?;
    pointwise_objective_prefab.add_link_orientation_matching_objective_relative_to_state( &q_init, 0, None, None );

    let mut pointwise_type = OptimizationModelType::SingleThreaded(pointwise_objective_prefab.optimization_model);
    let mut pathwise_type = OptimizationModelType::SingleThreaded(pathwise_objective_prefab.optimization_model);

    let mut group = OptimizationModelTypeGroup::new(vec![ pointwise_type, pathwise_type ]);

    let s = Instant::now();
    let res = RelaxedOMPLocal::solve_single(&q_init, &q_goal, &mut group, &coll, 0.01, false, false, None )?;

    println!("{:?}", s.elapsed());
    println!("{:?}", res);

    res.output_path_to_file("relaxed_omp_local", "sawyer", "test");

    Ok(())
}