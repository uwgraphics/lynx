extern crate lynx_lib;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;


fn main() -> Result<(), String>{
    let mut r = RobotModuleToolbox::new("iiwa7", None, None)?;

    r.get_dof_module_ref().print_joint_dof_order();
    r.get_configuration_module_ref().robot_model_module.print_link_order();

    Ok(())
}