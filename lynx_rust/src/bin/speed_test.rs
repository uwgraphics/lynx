use std::time::{Instant, Duration};
extern crate lynx_lib;
use lynx_lib::utils::utils_math::implicit_dual_quaternion::ImplicitDualQuaternion;
use nalgebra::{Vector3};


fn main() {
    let a: Vec<Option<ImplicitDualQuaternion>> = vec![None ; 50 ];

    let v = Vector3::new(0.,0.,0.);

    let idq1 = ImplicitDualQuaternion::new_from_euler_angles(0.0,0.,0.1, Vector3::new(0.0,0.0,0.));
    let idq2 = ImplicitDualQuaternion::new_from_euler_angles(0.,0.,0.0, Vector3::new(0.1,0.,0.));

    println!("{:?}", idq1.multiply(&idq2));
    println!("{:?}", idq1.multiply_shortcircuit(&idq2));

    // println!("{:?}", idq1.translation_is_zeros);

    let start = Instant::now();
    for i in 0..10000 {
        let r = idq1.multiply_shortcircuit(&idq2);
    }
    println!("{:?}", start.elapsed());

}