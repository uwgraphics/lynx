extern crate lynx_lib;
use lynx_lib::prelude::*;


fn main() -> Result<(), String> {

    // load robot
    let mut robot = Robot::new("nao", None)?;

    // print link order
    robot.get_configuration_module_ref().robot_model_module.print_link_order();

    // print joint order
    robot.get_dof_module_ref().print_joint_dof_order();

    // print robot joint position bounds
    robot.get_bounds_module_ref().print_bounds();

    // print information about salient links.  This is loaded in from robot_salient_links/robot_salient_links.yaml file in robots directory.
    robot.get_salient_links_module_ref().print_summary();

    Ok(())
}