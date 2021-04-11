use nalgebra::{UnitQuaternion, Quaternion, Vector3};
extern crate lynx_lib;
use lynx_lib::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use std::ops::Neg;

fn main() {
    let idq = ImplicitDualQuaternion::new_from_euler_angles(1.,2.,3., Vector3::new(0.,0.,0.));

    let q = idq.quat.clone();

    println!("{:?}", q);
    println!("{:?}", q.neg());

}