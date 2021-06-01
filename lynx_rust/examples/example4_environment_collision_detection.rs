extern crate lynx_lib;
use lynx_lib::prelude::*;
use lynx_lib::utils::utils_math::prelude::vec_to_dvec;


fn main() -> Result<(), String> {
    // load robot world with environment "single_box" (included in assets/mesh_environments)
    let mut robot_world = RobotWorld::new(vec!["ur5"], vec![None], Some("single_box"))?;

    // compute forward kinematics using the fk_module
    let fk_result = robot_world.get_robot_set_ref().compute_fk(&vec_to_dvec(&vec![0.,0.,0.,0.,0.,0.]))?;

    // do environment intersection test
    let environment_intersect_result = robot_world.environment_intersect_check(&fk_result, LinkGeometryType::OBBs, true)?;

    // print summary of result, should be Intersection Not Found
    environment_intersect_result.print_summary();
    assert_eq!(environment_intersect_result.in_collision(), false);



    // compute forward kinematics using the fk_module
    let fk_result = robot_world.get_robot_set_ref().compute_fk(&vec_to_dvec(&vec![1.57,0.0,-1.57,0.,0.,0.]))?;

    // do environment intersection test
    let environment_intersect_result = robot_world.environment_intersect_check(&fk_result, LinkGeometryType::OBBs, true)?;

    // print summary of result, should be Intersection Found between "box_0" and "forearm_link"
    environment_intersect_result.print_summary();
    assert_eq!(environment_intersect_result.in_collision(), true);



    Ok(())
}