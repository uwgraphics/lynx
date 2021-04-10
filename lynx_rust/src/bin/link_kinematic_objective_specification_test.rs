extern crate lynx_lib;
use lynx_lib::utils::utils_robot_objective_specification::{robot_salient_link_description::RobotSalientLinkDescription, link_kinematic_objective_specification::*};
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use lynx_lib::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use nalgebra::{Point3, DVector, Vector3};

fn main() -> Result<(), String> {
    let robot_salient_link_description = RobotSalientLinkDescription::new("ur5")?;

    let robot_module_toolbox = RobotModuleToolbox::new("ur5", None, None)?;

    let fk_res = robot_module_toolbox.get_fk_module_ref().compute_fk_on_all_zeros_config()?;

    let l = LinkPoseMatchingSpecification::new_relative_to_fk_res(&robot_salient_link_description, 0, &fk_res, Some(ImplicitDualQuaternion::new_from_euler_angles(0.5,0.,0., Vector3::new(1.,0.,0.))))?;

    println!("{:?}", l);

    Ok(())
}