extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::prelude::*;

fn main() {
    let mut local_planner_gym = LocalPlannerGym::new_empty(0.08);

    local_planner_gym.set_robot_world("hubo", Some("c2"), None, Some("hubo_arms_around_table"));
    local_planner_gym.add_sprint_local();
    local_planner_gym.add_straight_line_local();
    local_planner_gym.add_rrt_connect(100);
    local_planner_gym.add_rrt(100);
    local_planner_gym.add_rrt_connect(500);
    local_planner_gym.add_rrt(500);
    local_planner_gym.add_rrt_connect(1000);
    local_planner_gym.add_rrt(1000);

    local_planner_gym.run(3000);
}