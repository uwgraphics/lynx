extern crate lynx_lib;
use lynx_lib::robot_modules::{robot_configuration_module::RobotConfigurationModule, robot_dof_module::RobotDOFModule, robot_fk_module::RobotFKModule, robot_bounds_module::RobotBoundsModule, robot_core_collision_module::*, robot_triangle_mesh_collision_module::RobotTriangleMeshCollisionModule};


fn main() -> Result<(), String> {
    let robot_configuration_module = RobotConfigurationModule::new("sawyer".to_string(), None);
    let robot_dof_module = RobotDOFModule::new(&robot_configuration_module);
    let robot_fk_module = RobotFKModule::new(&robot_configuration_module, &robot_dof_module);
    let robot_bounds_module = RobotBoundsModule::new(&robot_configuration_module, &robot_dof_module, None);

    robot_dof_module.print_joint_dof_order();

    let mut robot_core_collision_module = RobotCoreCollisionModule::new(&robot_configuration_module, &robot_fk_module, &robot_bounds_module)?;
    // let mut robot_triangle_mesh_collision_module = RobotTriangleMeshCollisionModule::new(&robot_configuration_module, &robot_fk_module, &robot_bounds_module)?;

    let fk_res = robot_fk_module.compute_fk_vec(&vec![0.,0.,0.,0.,2.7,-1.8,0.,0.])?;
    let res = robot_core_collision_module.self_intersect_check(&fk_res, LinkGeometryType::OBBs, false)?;

    res.print_summary();

    // robot_core_collision_module.add_not_in_collision_example_with_fk(&vec![1.,0.0,2.,0.,0.,0.], &robot_fk_module);

    // robot_core_collision_module.add_all_zeros_config_as_not_in_collision_example(&robot_fk_module);

    // let res = robot_core_collision_module.add_manual_collision_check_skip_between_links(&robot_configuration_module, "head".to_string(), "right_l3".to_string());

    // robot_core_collision_module.revert_skip_collision_check_tensors();
    // robot_core_collision_module.refine_models(&robot_fk_module, &robot_bounds_module, &mut robot_triangle_mesh_collision_module, 10);
    // robot_core_collision_module.accuracy_check(&robot_fk_module, &robot_bounds_module, &mut robot_triangle_mesh_collision_module, 30);

    robot_core_collision_module.print_collision_check_skips(LinkGeometryType::ConvexShapes);

    Ok(())
}
