use nalgebra::Vector3;
extern crate lynx_lib;
use lynx_lib::utils::utils_math::sphere_utils::*;


fn main() {
    let o1 = Vector3::new(0.,0.,0.);
    let o2 = Vector3::new(-0.159,0.,0.);

    let r1 = 1.0;
    let r2 = 0.9;

    let v = volume_of_overlap_of_two_spheres_by_coordinates(&o1, &o2, r1, r2);
    println!("{:?}", v);
}