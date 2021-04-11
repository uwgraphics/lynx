extern crate lynx_lib;
use lynx_lib::path_planning::sprint::sprint::Sprint;
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;
use lynx_lib::utils::utils_path_planning::utils_path_planning_sampling::{cspace_sampler::*};
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_path_planning::path_planner_output::*;
use lynx_lib::robot_modules::robot_configuration_module::RobotConfigurationModule;
use lynx_lib::robot_modules::robot_dof_module::RobotDOFModule;
use std::time::Instant;
use nalgebra::DVector;
use termion::{color, style};


fn main() -> Result<(), String> {
    let mut robot_configuration_module = RobotConfigurationModule::new("ur5".to_string(), None);
    let mut robot_dof_module = RobotDOFModule::new(&robot_configuration_module);

    let mut collision_checker = RobotCollisionChecker::new("ur5".to_string(), None, None, Some("ur5_table".to_string()));
    let mut cspace_sampler = RobotCSpaceSampler::new("ur5".to_string(), None, None);

    let q_init = vec_to_dvec(&vec![ 3.14, -0.38, -1.2, -1.57, -1.57, -1.57 ]);
    let q_goal = vec_to_dvec(&vec![ 3.14, 0.0, -0.78, -0.76, -1.57, -1.57 ]);

    for i in 0..1 {
        let start = Instant::now();
        let res = Sprint::solve(&q_init, &q_goal, &mut collision_checker, &cspace_sampler, 0.04, false, false, None)?;
        let stop = start.elapsed();

        match res.result {
            PathPlannerResult::SolutionFound(s) => {
                s.waypoints.iter().for_each(|x| robot_dof_module.print_input_x_vals_next_to_joint_dof_names(x));
                println!("{}{}Solution found in {:?} {}", color::Fg(color::Green), style::Bold, stop, style::Reset);
                println!("{}{}Solution waypoints posted above. {}", color::Fg(color::Blue), style::Bold, style::Reset);
            },
            _ => { }
        }
    }

    Ok(())
}