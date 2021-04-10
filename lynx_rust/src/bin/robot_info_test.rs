extern crate lynx_lib;
use lynx_lib::utils::utils_robot_objective_specification::robot_salient_link_description::RobotSalientLinkDescription;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;


fn main() -> Result<(), String> {

    let mut robot = RobotModuleToolbox::new("hubo", None, None)?;

    robot.get_configuration_module_ref().robot_model_module.print_link_order();

    let ri = RobotSalientLinkDescription::new("sawyer")?;

    ri.print_summary();

    Ok(())
}