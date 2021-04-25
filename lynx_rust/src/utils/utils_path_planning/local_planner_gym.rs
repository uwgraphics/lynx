use crate::robot_modules::robot_world::*;
use crate::path_planning::prelude::*;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use crate::utils::utils_path_planning::local_search::*;
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_collisions::prelude::*;
use crate::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use crate::utils::utils_image_environments::prelude::*;
use std::time::{Instant, Duration};
use termion::{color, style};
use crate::utils::utils_path_planning::path_planner_result::PathPlannerResult;

/*
pub struct LocalPlannerGym<'a> {
    _lynx_vars_parallel: LynxVarsGeneric<'a>,
    _lynx_vars_single_threaded: LynxVarsGeneric<'a>,
    _local_planners: Vec<LocalSearchBox>,
    _do_parallel_test: Vec<bool>,
    _sampler: Option<LynxFloatVecSamplerBox>,
    _collision_checker: Option<CollisionCheckerBox>,
    _lambda: f64
}

impl<'a> LocalPlannerGym<'a> {
    pub fn new_empty(lambda: f64) -> Self {
        let _lynx_vars_parallel = LynxVarsGeneric::new_empty_parallel(None);
        let _lynx_vars_single_threaded = LynxVarsGeneric::new_empty_single_threaded();
        let _local_planners = Vec::new();
        let _do_parallel_test = Vec::new();
        let _sampler = None;
        let _collision_checker = None;
        let _lambda = lambda;
        return Self { _lynx_vars_parallel, _lynx_vars_single_threaded, _local_planners, _do_parallel_test, _sampler, _collision_checker, _lambda };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_robot_world(&mut self, robot_name: &str, configuration_name: Option<&str>, mobile_base_bounds_filename: Option<&str>, environment_name: Option<&str>) -> Result<(), String> {
        let robot_world = RobotWorld::new(robot_name, configuration_name, mobile_base_bounds_filename, environment_name)?;

        self._sampler = Some(robot_world.get_robot_module_toolbox_ref().get_bounds_module_ref().to_lynx_float_vec_sampler_box());
        self._collision_checker = Some(RobotWorldCollisionChecker.to_collision_checker_box());

        set_or_add_lynx_var_generic!(&mut self._lynx_vars_parallel, RobotWorld, "robot_world", robot_world.clone());
        set_or_add_lynx_var_generic!(&mut self._lynx_vars_single_threaded, RobotWorld, "robot_world", robot_world.clone());

        Ok(())
    }

    pub fn set_image_environment(&mut self, image_name: &str) -> Result<(), String> {
        let image_environment = ImageEnvironment::new_collision_image(image_name);

        self._sampler = Some(image_environment.to_lynx_float_vec_sampler_box());
        self._collision_checker = Some(ImageEnvironmentCollisionChecker::new(image_name).to_collision_checker_box());

        Ok(())
    }

    pub fn set_lambda(&mut self, lambda: f64) {
        self._lambda = lambda;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_local_planner(&mut self, local_search_box: LocalSearchBox, do_parallel_test: bool) {
        self._local_planners.push(local_search_box);
        self._do_parallel_test.push(do_parallel_test);
    }

    pub fn add_straight_line_local(&mut self) -> Result<(), String> {
        if self._collision_checker.is_none() {
            return Err("collision_checker must be set before adding straight_line_local".to_string());
        }

        let planner = StraightLineLocalSearch::new(self._collision_checker.as_ref().unwrap().clone(), self._lambda).to_local_search_box();
        self.add_local_planner(planner, false);
        return Ok(());
    }

    pub fn add_sprint_local(&mut self) -> Result<(), String> {
        if self._collision_checker.is_none() {
            return Err("collision_checker must be set before adding sprint_local".to_string());
        }

        let planner = SprintLocal::new(self._collision_checker.as_ref().unwrap().clone(), self._lambda, false, false).to_local_search_box();
        self.add_local_planner(planner, true);
        return Ok(());
    }

    pub fn add_rrt(&mut self, max_num_collision_checks: usize) -> Result<(), String> {
        if self._sampler.is_none() {
            return Err("sampler must be set before adding rrt".to_string());
        }

        if self._collision_checker.is_none() {
            return Err("collision_checker must be set before adding rrt".to_string());
        }

        let planner = RRT::new(self._sampler.as_ref().unwrap().clone(), self._collision_checker.as_ref().unwrap().clone(), self._lambda, max_num_collision_checks).to_local_search_box();
        self.add_local_planner(planner, false);
        return Ok(());
    }

    pub fn add_rrt_connect(&mut self, max_num_collision_checks: usize) -> Result<(), String> {
        if self._sampler.is_none() {
            return Err("sampler must be set before adding rrt".to_string());
        }

        if self._collision_checker.is_none() {
            return Err("collision_checker must be set before adding rrt".to_string());
        }

        let planner = RRTConnect::new(self._sampler.as_ref().unwrap().clone(), self._collision_checker.as_ref().unwrap().clone(), self._lambda, max_num_collision_checks).to_local_search_box();
        self.add_local_planner(planner, false);
        return Ok(());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn run(&mut self, num_pairs: usize) -> Result<(), String> {
        if self._sampler.is_none() {
            return Err("sampler must be set before running LocalPlannerGym".to_string());
        }

        if self._collision_checker.is_none() {
            return Err("collision_checker must be set before running LocalPlannerGym".to_string());
        }

        let recorder = RecorderArcMutexOption::new_none();
        let mut terminate = TerminationUtilOption::new_none();

        let freespace_sampler = FreeSpaceSampler::new(self._sampler.as_ref().unwrap().clone(), self._collision_checker.as_ref().unwrap().clone());

        let mut solution_found_count = vec![ 0.0; self._local_planners.len() * 2 ];
        let mut average_success_solution_times = vec![Duration::new(0, 0); self._local_planners.len() * 2 ];
        let mut average_failure_solution_times = vec![Duration::new(0, 0); self._local_planners.len() * 2 ];

        for i in 0..num_pairs {
            println!("{}{}Sample Pair {:?} {}", style::Bold, color::Fg(color::LightCyan), i, style::Reset);

            let q_init = freespace_sampler.lynx_float_vec_sampler_sample(&mut self._lynx_vars_single_threaded)?;
            let q_goal = freespace_sampler.lynx_float_vec_sampler_sample(&mut self._lynx_vars_single_threaded)?;
            println!("  {}q_init {:?} {}", color::Fg(color::Blue), q_init.data.as_vec(), style::Reset);
            println!("  {}q_goal {:?} {}", color::Fg(color::Blue), q_goal.data.as_vec(), style::Reset);

            let l = self._local_planners.len();
            for j in 0..l {
                let start = Instant::now();
                let single_threaded_res = self._local_planners[j].solve_local(&q_init, &q_goal, &mut self._lynx_vars_single_threaded, &recorder, &mut terminate)?;
                let single_threaded_time = start.elapsed();

                print!("      >>> {}{}: {}", color::Fg(color::LightWhite), self._local_planners[j].name_local(), style::Reset);
                match single_threaded_res {
                    PathPlannerResult::SolutionFound(_) => {
                        print!(" {}solution found!{}", color::Fg(color::Green), style::Reset);
                        solution_found_count[j * 2] += 1.0;
                        average_success_solution_times[j * 2] += single_threaded_time.clone();
                    }
                    PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(_) => {
                        print!(" {}solution not found{}", color::Fg(color::Red), style::Reset);
                        average_failure_solution_times[j * 2] += single_threaded_time.clone();
                    }
                    PathPlannerResult::SolutionNotFound(_) => {
                        print!(" {}solution not found{}", color::Fg(color::Red), style::Reset);
                        average_failure_solution_times[j * 2] += single_threaded_time.clone();
                    }
                }
                print!(" {}time: {:?} {}\n", color::Fg(color::LightWhite), single_threaded_time, style::Reset);

                if self._do_parallel_test[j] {
                    let start = Instant::now();
                    let parallel_res = self._local_planners[j].solve_local(&q_init, &q_goal, &mut self._lynx_vars_parallel, &recorder, &mut terminate)?;
                    let parallel_time = start.elapsed();
                    print!("      >>> {}{} Parallel: {}", color::Fg(color::LightWhite), self._local_planners[j].name_local(), style::Reset);
                    match parallel_res {
                        PathPlannerResult::SolutionFound(_) => {
                            print!(" {}solution found!{}", color::Fg(color::Green), style::Reset);
                            solution_found_count[j * 2 + 1] += 1.0;
                            average_success_solution_times[j * 2 + 1] += parallel_time.clone();
                        }
                        PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(_) => {
                            print!(" {}solution not found{}", color::Fg(color::Red), style::Reset);
                            average_failure_solution_times[j * 2 + 1] += parallel_time.clone();
                        }
                        PathPlannerResult::SolutionNotFound(_) => {
                            print!(" {}solution not found{}", color::Fg(color::Red), style::Reset);
                            average_failure_solution_times[j * 2 + 1] += parallel_time.clone();
                        }
                    }
                    print!(" {}time: {:?} {}\n", color::Fg(color::LightWhite), parallel_time, style::Reset);
                }
                println!();
            }
        }


        let l = solution_found_count.len();
        let num_pairs_f64 = num_pairs as f64;
        let mut count = 0;

        println!("--------------------------------------------------------------------------------");
        println!("LocalPlannerGym Summary: ");
        for i in 0..l {
            let mut name = self._local_planners[i / 2].name_local();
            if i % 2 == 1 { name += " Parallel"; }
            if !(i % 2 == 1 && !self._do_parallel_test[i/2]) {
                print!("{:?}) {:?}\n", count, name); count += 1;
                print!(" {}        success rate {:?} / {:?} ({:?})\n{}", color::Fg(color::LightCyan), solution_found_count[i], num_pairs_f64, solution_found_count[i] / num_pairs_f64, style::Reset);
                print!(" {}        average success solution times {:?} \n{}", color::Fg(color::LightGreen), Duration::from_secs_f64(average_success_solution_times[i].as_secs_f64() / num_pairs_f64), style::Reset);
                print!(" {}        average failure solution times {:?} \n{}", color::Fg(color::LightRed), Duration::from_secs_f64(average_failure_solution_times[i].as_secs_f64() / num_pairs_f64), style::Reset);
            }
        }

        Ok(())
    }

}

 */