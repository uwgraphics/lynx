extern crate lynx_lib;
use lynx_lib::utils::utils_collisions::collision_object::CollisionObject;
use lynx_lib::utils::utils_collisions::collision_object_utils::*;
use lynx_lib::utils::utils_collisions::collision_check_tensor::*;
use nalgebra::Point3;

fn main() {
    let mut c1 = CollisionObject::new_cuboid(0.5, 0.5, 0.5, None, Some("base".to_string()));
    let mut c2 = CollisionObject::new_cuboid(0.5, 0.5, 0.5, None, Some("top".to_string()));
    let mut c3 = CollisionObject::new_cuboid(0.5, 0.5, 0.5, None, Some("left".to_string()));
    let mut c4 = CollisionObject::new_cuboid(0.5, 0.5, 0.5, None, Some("right".to_string()));

    c2.set_curr_translation(0.4,0.4,1.2); c2.update_bounding_aabb();
    c3.set_curr_translation(-0.9,0.4,-0.5); c3.update_bounding_aabb();
    c4.set_curr_translation(0.9,0.4,1.9); c4.update_bounding_aabb();

    let g1 = vec![ vec![ c1, c2 ] ];
    let g2 = vec![ vec![ c3, c4 ] ];

    let s = BoolCollisionCheckTensor::new(&g2, &g2, SkipCheckForSelfCollisionMode::SameObjectOnly);

    let res = contact_check_between_multiple_collision_objects( &g2, &g2, false, Some(10000.0), Some(s) );

    println!("{:?}", res);

    let point = Point3::new(0.,0.,0.);
    let res = contains_point_check_between_multiple_collision_objects(&g1, &point, true);

    println!("{:?}", res);
}