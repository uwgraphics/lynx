extern crate lynx_lib;
use lynx_lib::utils::utils_collisions::oriented_bounding_box::OBB;

fn main() {
    let mut obb1 = OBB::new_from_path("dome.obj".to_string());
    let mut obb2 = OBB::new_from_path("dome.obj".to_string());

    println!("{:?}", obb1.half_extents);
    println!("{:?}", obb1.collision_object.curr_pose);

    obb2.set_curr_translation(5.0, 0., 0.);

    println!("{:?}", obb2.half_extents);
    println!("{:?}", obb2.collision_object.curr_pose);

    obb1.stretch_or_shrink_dimensions(3.9, 1., 1.);

    println!("{:?}", obb1.intersect_check(&obb2));
}