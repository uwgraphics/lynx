extern crate lynx_lib;
use lynx_lib::prelude::*;


fn main() -> Result<(), String> {

    // load robot module toolbox
    let robot_module_toolbox = RobotModuleToolbox::new("ur5", None, None)?;

    // print link order
    robot_module_toolbox.get_configuration_module_ref().robot_model_module.print_link_order();

    // print joint order
    robot_module_toolbox.get_dof_module_ref().print_joint_dof_order();

    // print robot joint position bounds
    robot_module_toolbox.get_bounds_module_ref().print_bounds();

    // print information about salient links.  This is loaded in from robot_salient_links/robot_salient_links.yaml file in robots directory
    robot_module_toolbox.get_salient_links_module_ref().print_summary();

    Ok(())
}