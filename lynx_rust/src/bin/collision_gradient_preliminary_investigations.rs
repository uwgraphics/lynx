extern crate lynx_lib;
use lynx_lib::robot_modules::{robot_module_toolbox::RobotModuleToolbox, robot_configuration_module::RobotConfigurationModule};
use lynx_lib::utils::utils_collisions::collision_object_utils::*;
use lynx_lib::robot_modules::robot_core_collision_module::LinkGeometryType;

fn main() -> Result<(), String> {
    let mut robot_module_toolbox = RobotModuleToolbox::new("ur5", None, None)?;

    let fk_res = robot_module_toolbox.get_fk_module_ref().compute_fk_vec(&vec![0.,0.,2.7,0.,0.,0.])?;

    let contact_check_res = robot_module_toolbox.get_core_collision_module_mut_ref().self_contact_check(&fk_res, LinkGeometryType::OBBs, false, Some(0.3))?;

    contact_check_res.print_summary();

    Ok(())
}