extern crate lynx_lib;
use lynx_lib::prelude::*;


fn main() -> Result<(), String> {
    // load default robot module toolbox
    let mut robot = Robot::new("ur5", None)?;

    // compute forward kinematics using the fk_module
    let fk_result = robot.get_fk_module_ref().compute_fk_vec(&vec![0., 0., 0., 0., 0., 0.])?;

    // do self intersection test
    let self_intersect_result = robot.get_core_collision_module_mut_ref().self_intersect_check(&fk_result, LinkGeometryType::OBBs, true)?;

    // print summary of result, should be Intersection Not Found
    self_intersect_result.print_summary();
    assert_eq!(self_intersect_result.is_in_collision(), false);



    // compute forward kinematics using the fk_module
    let fk_result = robot.get_fk_module_ref().compute_fk_vec(&vec![0., 0., 3., 0., 0., 0.])?;

    // do self intersection test
    let self_intersect_result = robot.get_core_collision_module_mut_ref().self_intersect_check(&fk_result, LinkGeometryType::OBBs, true)?;

    // print summary of result, should be Intersection Found between "base_link" and "wrist_2_link"
    self_intersect_result.print_summary();
    assert_eq!(self_intersect_result.is_in_collision(), true);



    Ok(())
}