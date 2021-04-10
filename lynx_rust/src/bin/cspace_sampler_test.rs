extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::utils_path_planning_sampling::{cspace_sampler::*, freespace_sampling::*};
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;


fn main() {

    // let r = RobotCSpaceSampler::new("ur5".to_string(), None, None);
    // let mut c = RobotCollisionChecker::new("ur5".to_string(), None, None, None);

    let mut c = ImageEnvironmentCollisionChecker::new("BigGameHunters".to_string());
    let s = ImageEnvironmentCSpaceSampler::new_from_image_name("BigGameHunters".to_string());

    let res = sample_batch_from_freespace(&mut c, &s, 60);
    println!("{:?}", res);
}