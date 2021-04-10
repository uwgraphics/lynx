extern crate lynx_lib;
use lynx_lib::utils::utils_collisions::collision_environment::CollisionEnvironment;


fn main() {
    let mut ce = CollisionEnvironment::new("hubo_arms_around_table".to_string());

    println!("{:?}", ce.environment_obbs[0].len());
    println!("{:?}", ce.environment_obbs[0][0].name);

    println!("{:?}", ce.object_names);

    // println!("{:?}", ce.environment_obbs[0][0].half_extents);
    // println!("{:?}", ce.environment_obbs[0][0].collision_object.curr_pose);
}