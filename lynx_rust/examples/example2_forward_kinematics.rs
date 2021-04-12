extern crate lynx_lib;
use lynx_lib::prelude::*;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};

fn main() -> Result<(), String> {
    // load default robot module toolbox
    let robot_module_toolbox = RobotModuleToolbox::new("ur5", None, None)?;

    // compute forward kinematics using the fk_module
    let fk_result = robot_module_toolbox.get_fk_module_ref().compute_fk_vec(&vec![0.,0.,0.,0.,0.,0.])?;

    // print fk results next to the robot's link names
    robot_module_toolbox.get_fk_module_ref().print_results_next_to_link_names(&fk_result, robot_module_toolbox.get_configuration_module_ref());

    let ee_pos = fk_result.get_link_frames_ref()[7].as_ref().unwrap();

    assert_eq!(ee_pos.translation, Vector3::new(0.0, 0.19145, 1.001059));
    assert_eq!(ee_pos.quat, UnitQuaternion::from_quaternion(Quaternion::new(0.7071067818211393, 0.0, 0.0, 0.7071067805519557)));

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////


    // load robot module toolbox with mobile base
    let robot_module_toolbox = RobotModuleToolbox::new("ur5", Some("planar_base"), None)?;

    // compute forward kinematics using the fk_module, moving the mobile base 1 meter forward on the x axis
    let fk_result = robot_module_toolbox.get_fk_module_ref().compute_fk_vec(&vec![0.,0.,0.,0.,0.,0.,1.,0.,0.])?;

    // print fk results next to the robot's link names
    robot_module_toolbox.get_fk_module_ref().print_results_next_to_link_names(&fk_result, robot_module_toolbox.get_configuration_module_ref());

    let ee_pos = fk_result.get_link_frames_ref()[7].as_ref().unwrap();

    assert_eq!(ee_pos.translation, Vector3::new(1.0, 0.19145, 1.001059));
    assert_eq!(ee_pos.quat, UnitQuaternion::from_quaternion(Quaternion::new(0.7071067818211393, 0.0, 0.0, 0.7071067805519557)));

    Ok(())
}