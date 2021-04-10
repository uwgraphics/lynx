extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_collisions::collision_object::CollisionObject;
use std::time::Instant;

fn main() {

    let mut p = RobotCollisionChecker::new("ur5".to_string(), None, None, Some("sawyer_vertical".to_string()));

    let res = p.in_collision( &vec_to_dvec(&vec![0.,0.,0.,0.,0.,0.]) );
    println!("{:?}", res);


    let start = Instant::now();
    for i in 0..120 {
        p.clone();
    }
    println!("{:?}", start.elapsed());

    let mut p = ImageEnvironmentCollisionChecker::new("block".to_string());

    println!("{:?}", p.in_collision(&vec_to_dvec(&vec![0.5,1.0])));
    println!("{:?}", p.get_collision_environment_name());
}