extern crate lynx_lib;
use lynx_lib::optimal_motion_planning::prelude::*;
use lynx_lib::utils::utils_experiments::optimal_motion_planning_experiments::hydra_21_rss_experiment_prefabs::{*};


fn main() {
    // ur5_fixed_ee_pose_experiment(None);
    // jaco7_grasp(None);
    // iiwa7_lookat(None);
    sawyer_fixed_ee_pose_experiment(None);
}