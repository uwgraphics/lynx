extern crate lynx_lib;
use lynx_lib::optimal_motion_planning::prelude::*;
use lynx_lib::utils::utils_experiments::optimal_motion_planning_experiments::optimization_model_tester::OptimizationModelTester;
use lynx_lib::utils::utils_robot_objective_specification::link_kinematic_objective_specification::LinkPositionMatchingSpecification;


fn main() -> Result<(), String> {
    // let r = RobotModuleToolbox::new("hubo", Some("c1"), None)?;
    // r.get_dof_module_ref().print_joint_dof_order();

    let mut r = RelaxedIKOptimizationModelPrefab::new_base("t", "sawyer", Some("test"), None)?;
    r.add_link_orientation_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,0.0,0.,0.0,0.,0.,0.]), 0, None, None);
    r.add_link_position_matching_objective(LinkPositionMatchingSpecification::new(0, Vector3::new(0.55,0.45,0.2)), None);
    r.set_collision_environment("sawyer_lotsofboxes");

    // r.add_link_position_matching_objective(LinkPositionMatchingSpecification::new(0, Vector3::new(0.5,0.,0.)), None);
    // println!("got here");
    // r.add_two_link_match_given_distance(0,1, 0.3, None);
    // r.add_two_link_match_orientation(0,1, None);
    // r.add_link_orientation_matching_objective_relative_to_state(&vec_to_dvec(&vec![0.,0.,-1.0,0.,2.0,0.,0.,0.]), 0, None, None);
    // r.add_link_lookat_objective(0, Some(Vector3::new(0.0, 3.0, 5.0)), None);
    // r.add_link_upright_objective(0, None);
    // r.add_link_lookat_another_link_objective(1,0, None);
    // r.add_link_upright_objective(1, None);
    // r.add_link_position_matching_objective(LinkPositionMatchingSpecification::new(0, Vector3::new(0.5,-0.5,0.2)), None);
    // r.add_link_upright_objective(0, None);
    // r.add_link_upright_objective(0, None);
    // r.add_link_position_matching_objective(LinkPositionMatchingSpecification::new(0, Vector3::new(0.52,0.25,0.45)), None);
    // r.add_link_lookat_objective(0, Some(Vector3::new(0.55,0.25,0.45)), None);
    // r.add_link_upright_objective(0, None);

    OptimizationModelTester::output_samples_of_relaxed_ik_prefab_model(&mut r, "sawyer")?;

    Ok(())
}