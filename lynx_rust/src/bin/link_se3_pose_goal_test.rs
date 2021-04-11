extern crate lynx_lib;
use lynx_lib::utils::utils_se3::link_se3_pose_goal::LinkSE3PoseGoal;
use lynx_lib::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use nalgebra::Vector3;
use lynx_lib::robot_modules::robot_configuration_module::RobotConfigurationModule;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;


fn main() -> Result<(), String> {
    let mut r = RobotConfigurationModule::new("ur5", None);
    let mut g = LinkSE3PoseGoal::new(&r, 1, ImplicitDualQuaternion::new_identity())?;

    let mut idq = ImplicitDualQuaternion::new_from_euler_angles(0.05,0.05,0.0, Vector3::new(0.05,0.,0.));

    println!("{:?}", g.get_disp_l2_magnitude(&idq));


    let robot_toolbox = RobotModuleToolbox::new("ur5", None, None)?;

    // let fk_res = robot_toolbox.get_fk_module_ref().compute_fk_vec(&vec![0.1,0.,0.,0.,0.,0.1])?;

    let res = robot_toolbox.get_fk_module_ref().compute_fk_gradient_perturbations_on_all_zeros_config()?;

    println!("{:?}", res);

    Ok(())
}