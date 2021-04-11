extern crate lynx_lib;
use lynx_lib::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use lynx_lib::utils::utils_se3::transformation_utils::*;
use nalgebra::Vector3;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;


fn main() -> Result<(), String> {
    /*
    let mut i1 = ImplicitDualQuaternion::new_from_euler_angles(0.,0.,0., Vector3::new(0.,0.,1.));
    let mut i2 = ImplicitDualQuaternion::new_from_euler_angles(0.2,0.,0., Vector3::new(0.2,0.,1.1));

    let disp = implicit_dual_quaternion_disp_idq(&i1, &i2);

    println!("{:?}", i2);
    println!("{:?}", disp);

    let mut p = Vector3::new(0.,1.,1.);
    let mut pl = &p - &i1.translation;

    let p2 = disp.multiply_by_vector3_shortcircuit(&pl) + &i1.translation;

    println!("{:?}", p2);

    let r = implicit_dual_quaternion_vector3_displacement_transform(&i1, &i2, &p);

    println!("{:?}", r);
    */

    let robot_module_toolbox = RobotModuleToolbox::new("ur5", None, None)?;

    let fk_res = robot_module_toolbox.get_fk_module_ref().compute_fk_vec(&vec![0.,0.,2.0,0.,0.,0.])?;
    let fk_res2 = robot_module_toolbox.get_fk_module_ref().compute_fk_vec(&vec![0.,0.,2.0001,0.,0.,0.])?;

    let i1 = fk_res.get_link_frames_ref()[4].as_ref().unwrap().clone();
    let i2 = fk_res2.get_link_frames_ref()[4].as_ref().unwrap().clone();
    let p = Vector3::new(0.2884979621038592, 0.0663551896521824, 0.3393566894402877);

    println!("{:?}", i1);
    println!("{:?}", i2);

    let translation_disp = &i2.translation - &i1.translation;

    println!("{:?}", p + translation_disp);

    let mut p1 = p + translation_disp;
    let mut centered = p1 - i1.translation;

    let q_disp = quaternion_disp_q(i1.quat.clone(), i2.quat.clone());
    centered = q_disp * centered;

    let answer = centered + i1.translation;
    println!("{:?}", answer);

    let a = implicit_dual_quaternion_vector3_displacement_transform(&i1, &i2, &p);
    println!("{:?}", a);

    Ok(())
}