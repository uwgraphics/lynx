extern crate lynx_lib;
use lynx_lib::utils::utils_collisions::{collision_object::CollisionObject, collision_object_utils::*};
use std::time::Instant;


fn main() {

    let mut c1 = CollisionObject::new_cuboid(1.,1.,1.,None, None);
    let mut c2 = CollisionObject::new_cuboid(1.,1.,1.,None, None);
    let mut c3 = CollisionObject::new_cuboid(1.,1.,1.,None, None);
    let mut c4 = CollisionObject::new_cuboid(1.,1.,1.,None, None);

    c2.set_curr_translation(8.,2.,2.);
    c2.set_curr_orientation_from_euler_angles(1.,3.14, 0.358723);
    c3.set_curr_translation(-2.,-2.,-2.);
    c4.set_curr_translation(2.,5.,0.);

    let mut res = c1.contact_check(&c2, Some(1.2));
    let start = Instant::now();
    for i in 0..1000 {
        res = c1.contact_check(&c2, Some(10000.0));
    }
    println!("{:?}", start.elapsed());
    println!("{:?}", res);

}