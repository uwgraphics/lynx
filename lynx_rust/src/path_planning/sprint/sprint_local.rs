use crate::utils::utils_path_planning::local_search::*;
use crate::utils::utils_collisions::collision_checker::*;
use crate::utils::utils_recorders::prelude::*;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use crate::utils::utils_path_planning::path_planner_result::PathPlannerResult;
use crate::utils::utils_path_planning::utils_planning_graphs::planning_dag::*;
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_collisions::collision_check_result_enum::CollisionCheckResult;
use crate::utils::utils_math::common_functions::*;
use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::utils::utils_sampling::float_vec_sampler_traits::FloatVecSampler;
use termion::{style, color};
use nalgebra::DVector;
use std::sync::RwLock;
use std::ops::Deref;

#[derive(Clone)]
pub struct SprintLocal {
    _collision_checker: CollisionCheckerBox,
    _lambda: f64,
    _force_single_threaded: bool,
    _debug: bool
}
impl SprintLocal {
    pub fn new(collision_checker: CollisionCheckerBox, lambda: f64, force_single_threaded: bool, debug: bool) -> Self {
        Self { _collision_checker: collision_checker, _lambda: lambda, _force_single_threaded: force_single_threaded, _debug: debug }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _solve_single_threaded_forward(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let mut planning_tree = PlanningDAG::new_unidirectional(q_init);
        let mut sprint_local_data_manager = SprintLocalDataManager::new(q_init, q_goal);
        let mut n_stack: Vec<usize> = Vec::new();
        let mut q_x_idx = 0 as usize;
        let mut add_checkpoint = true;

        for i in 0..1000 {
            if terminate.get_terminate() {
                return Ok(PathPlannerResult::SolutionNotFound("early terminate.".to_string()));
            }

            if self._debug { println!("{}{}>>>> local search loop {:?}.  q_x_idx is {:?} {}", color::Fg(color::Blue), style::Bold, i, q_x_idx, style::Reset); }

            if probability_heuristic_2(q_x_idx, &mut sprint_local_data_manager, None, self._debug)? || !add_checkpoint {
                if self._debug { println!("{}{}    q_x_idx {:?} was deemed to be worthwhile to extend by probability heuristic 2. {}", color::Fg(color::White), style::Bold, q_x_idx, style::Reset); }
                if add_checkpoint {
                    sprint_local_data_manager.label_q_x_as_checkpoint(q_x_idx, planning_tree.get_node_ref(q_x_idx)?)?;
                    if self._debug { println!("{}{}    q_x_idx {:?} was just labeled as a checkpoint. {}", color::Fg(color::White), style::Bold, q_x_idx, style::Reset); }
                }

                let q_c_star = probability_heuristic_3(q_x_idx, &sprint_local_data_manager, &planning_tree, self._lambda, self._debug)?;
                if self._debug { println!("{}{}    q_c_star was just calculated to be {:?} {}", color::Fg(color::White), style::Bold, q_c_star.data.as_vec(), style::Reset); }

                let in_collision = self._collision_checker.in_collision(&q_c_star, lynx_vars)?;

                match in_collision {
                    CollisionCheckResult::InCollision(s) => {
                        // if record { path_planner_output.recorder.as_mut().unwrap().action_add_collision_point(&q_c_star, None); }
                        if self._debug { println!("{}{}    q_c_star is in collision {}", color::Fg(color::LightRed), style::Bold, style::Reset); }
                        sprint_local_data_manager.add_collision_q_c_star(q_x_idx, &q_c_star)?;
                        if n_stack.is_empty() {
                            if self._debug { println!("{}{}>>>> n_stack was empty.  Exiting. {}", color::Fg(color::Red), style::Bold, style::Reset); }
                            return Ok(PathPlannerResult::SolutionNotFound("n_stack was empty".to_string()));
                        }
                        q_x_idx = n_stack.pop().unwrap();
                        add_checkpoint = true;
                    },
                    CollisionCheckResult::NotInCollision => {
                        // if record { path_planner_output.recorder.as_mut().unwrap().action_add_edge(  &q_c_star, planning_tree.get_node_ref(q_x_idx)?, None  ); }
                        let new_idx = sprint_local_data_manager.add_freespace_q_c_star(q_x_idx, &q_c_star)?;
                        let new_idx_redundancy_check = planning_tree.add_node_with_auto_two_waypoint_inflow_edge(&q_c_star, q_x_idx)?;
                        if !(new_idx == new_idx_redundancy_check) { return Err(format!("looks like there was an index misalignment in sprint local search.  data manager was on idx {:?} and planning tree was on idx {:?}", new_idx, new_idx_redundancy_check)); }
                        if self._debug { println!("{}{}    q_c_star is not in collision! added as node {:?} {}", color::Fg(color::LightGreen), style::Bold, new_idx, style::Reset); }
                        if (q_goal - &q_c_star).norm() < 2.0 * self._lambda {
                            if self._debug { println!("{}{}>>>> Solution found! {}", color::Fg(color::Green), style::Bold, style::Reset); }
                            let goal_idx = planning_tree.add_node_with_auto_linear_inflow_edge(q_goal, new_idx, self._lambda)?;
                            return Ok(PathPlannerResult::SolutionFound(planning_tree.get_path_from_tree_root_to_node(goal_idx)?));
                        }
                        n_stack.push(q_x_idx);
                        q_x_idx = new_idx;
                        add_checkpoint = false;
                    },
                    CollisionCheckResult::Error(s) => { return Err(s); }
                }

            } else {
                if self._debug { println!("{}{}    q_x_idx {:?} was deemed NOT worthwhile to extend by probability heuristic 2. {}", color::Fg(color::White), style::Bold, q_x_idx, style::Reset); }
                if n_stack.is_empty() {
                    if self._debug { println!("{}{}>>>> n_stack was empty.  Exiting. {}", color::Fg(color::Red), style::Bold, style::Reset); }
                    return Ok(PathPlannerResult::SolutionNotFound("n_stack was empty.".to_string()));
                }
                q_x_idx = n_stack.pop().unwrap();
            }

        }

        if self._debug { println!("{}{}>>>> No solution found.  Exiting on outer loop. {}", color::Fg(color::Red), style::Bold, style::Reset); }
        return Ok(PathPlannerResult::SolutionNotFound("maximum number of loops reached in sprint local search".to_string()));
    }

    fn _solve_single_threaded_reverse(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let mut res = self._solve_single_threaded_forward(q_goal, q_init, lynx_vars, recorder, terminate)?;
        match res {
            PathPlannerResult::SolutionFound(s) => {
                let mut out_path = s.clone();
                out_path.reverse();
                return Ok(PathPlannerResult::SolutionFound(out_path));
            }
            PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(s) => {
                let mut out_path = s.clone();
                out_path.reverse();
                return Ok(PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(out_path));
            }
            PathPlannerResult::SolutionNotFound(s) => {
                return Ok(PathPlannerResult::SolutionNotFound(s));
            }
        }
    }

    fn _solve_single_threaded_random_forward_or_reverse(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let sampler = RangeFloatVecSampler::new(0.0, 1.0, 1);
        let s = sampler.float_vec_sampler_sample()?;
        if s[0] > 0.5 {
            return self._solve_single_threaded_forward(q_init, q_goal, lynx_vars, recorder, terminate);
        } else {
            return self._solve_single_threaded_reverse(q_init, q_goal, lynx_vars, recorder, terminate);
        }
    }

    fn _solve_parallel(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let mut solution_path = RwLock::new( LinearSplinePath::new_empty() );

        match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => { return Err("called solve_parallel on single threaded lynx_vars.".to_string()); }
            LynxVarsGeneric::SingleThreadedMutRef(_) => { return Err("called solve_parallel on single threaded lynx_vars.".to_string()); }
            LynxVarsGeneric::Parallel(l) => {
                let mut terminate_local = TerminationUtilOption::new();

                let mut piter = l.get_par_iter_mut();
                piter.for_each(|x| {
                   let mut terminate_local_on_thread = terminate_local.clone();
                    let local_search_result = self._solve_single_threaded_random_forward_or_reverse(q_init, q_goal, &mut LynxVarsGeneric::SingleThreadedMutRef(x), recorder, &mut terminate_local_on_thread);

                    if local_search_result.is_ok() {
                        let local_search_result_unwrap = local_search_result.unwrap();
                        match local_search_result_unwrap {
                            PathPlannerResult::SolutionFound(s) => {
                                terminate_local_on_thread.set_to_terminate();
                                *solution_path.write().unwrap() = s;
                            }
                            PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(_) => {  }
                            PathPlannerResult::SolutionNotFound(_) => {  }
                        }
                    }
                });
            }
        }

        let solution_path_unwrap = solution_path.read().unwrap();
        return if solution_path_unwrap.waypoints.len() == 0 {
            Ok(PathPlannerResult::SolutionNotFound("Solution not found on any thread of sprint local parallel search.".to_string()))
        } else {
            Ok(PathPlannerResult::SolutionFound((*solution_path_unwrap).clone()))
        }
    }
}
impl LocalSearch for SprintLocal {
    fn solve_local(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let res = self._collision_checker.in_collision(q_init, lynx_vars)?;
        if res.is_in_collision() { return Ok(PathPlannerResult::SolutionNotFound("q_init was in collision in sprint local.".to_string())) }

        let res = self._collision_checker.in_collision(q_goal, lynx_vars)?;
        if res.is_in_collision() { return Ok(PathPlannerResult::SolutionNotFound("q_goal was in collision in sprint local.".to_string())) }

        return match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => { self._solve_single_threaded_forward(q_init, q_goal, lynx_vars, recorder, terminate) }
            LynxVarsGeneric::SingleThreadedMutRef(_) => { self._solve_single_threaded_forward(q_init, q_goal, lynx_vars, recorder, terminate) },
            LynxVarsGeneric::Parallel(_) => {
                if self._force_single_threaded {
                    self._solve_single_threaded_forward(q_init, q_goal, lynx_vars, recorder, terminate)
                } else {
                    self._solve_parallel(q_init, q_goal, lynx_vars, recorder, terminate)
                }
            }
        }
    }
    fn name_local(&self) -> String { return "SprintLocal".to_string(); }
}
impl LynxVarsUser for SprintLocal { }

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct SprintLocalDataManager {
    _h_exploit: Vec< Option<(f64, f64) >>,
    _h_explore: Vec< Option<(f64, f64) >>,
    _h_obs: Vec< Option<Vec<usize>> >,
    _h_num: Vec< Option<f64> >,
    _all_obs_points: Vec< DVector<f64> >,
    _previous_checkpoint_idx: Vec< Option<usize> >,
    _predecessor_node_idx: Vec< Option<usize> >,
    _is_checkpoint: Vec< bool >,
    _is_dead_checkpoint: Vec< bool >,
    _q_n_star: DVector<f64>,
    _q_m_star: DVector<f64>
}

impl SprintLocalDataManager {
    pub fn new( q_n_star: &DVector<f64>, q_m_star: &DVector<f64> ) -> Self {
        let _h_exploit = Vec::new();
        let _h_explore = Vec::new();
        let _h_obs = Vec::new();
        let _h_num = Vec::new();

        let _all_obs_points = Vec::new();
        let _previous_checkpoint_idx = Vec::new();
        let _predecessor_node_idx = Vec::new();
        let _is_checkpoint = Vec::new();
        let _is_dead_checkpoint = Vec::new();

        let _q_n_star = q_n_star.clone();
        let _q_m_star = q_m_star.clone();

        let mut out_self = Self {_h_exploit, _h_explore, _h_obs, _h_num,
            _all_obs_points, _previous_checkpoint_idx, _predecessor_node_idx, _is_checkpoint, _is_dead_checkpoint, _q_n_star, _q_m_star };

        out_self._initialize(q_n_star, q_m_star);

        return out_self;
    }

    fn _initialize(&mut self, q_n_star: &DVector<f64>, q_m_star: &DVector<f64>) {
        let exploit = (q_n_star - q_m_star).norm();
        let explore = 0.0;

        self._h_exploit.push( Some( (0.0, exploit) ) );
        self._h_explore.push( Some( (0.0, explore) ) );
        self._h_obs.push( Some( Vec::new() ) );
        self._h_num.push( Some(1.0) );
        self._previous_checkpoint_idx.push( None );
        self._predecessor_node_idx.push( None );
        self._is_checkpoint.push( false );
        self._is_dead_checkpoint.push( false );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn label_q_x_as_checkpoint(&mut self, q_x_idx: usize, q_x: &DVector<f64>) -> Result<(), String> {
        if q_x_idx >= self._is_checkpoint.len() {
            println!("{}{}ERROR: q_x_idx ({:?}) is too large in label_q_x_as_checkpoint (maximum idx is {:?}).  Failed to label checkpoint.  {}", color::Fg(color::Red), style::Bold, q_x_idx, self._is_checkpoint.len() -1, style::Reset);
            return Err(format!("q_x_idx ({:?}) is too large in label_q_x_as_checkpoint (maximum idx is {:?})", q_x_idx, self._is_checkpoint.len() -1));
        }

        if self._is_checkpoint[q_x_idx] { return Ok(()); }

        self._is_checkpoint[q_x_idx] = true;

        let exploit = (q_x - &self._q_m_star).norm();
        let explore = (q_x - &self._q_n_star).norm();

        self._h_exploit[q_x_idx] = Some( (0.0, exploit) );
        self._h_explore[q_x_idx] = Some( (0.0, explore) );

        self._h_obs[q_x_idx] = Some( Vec::new() );
        self._h_num[q_x_idx] = Some( 1.0 );

        return Ok(());
    }

    pub fn add_freespace_q_c_star(&mut self, q_x_idx: usize, q_c_star: &DVector<f64>) -> Result<usize, String> {
        if q_x_idx >= self._previous_checkpoint_idx.len() {
            println!("{}{}ERROR: q_x_idx ({:?}) is too large in add_freespace_q_c_star (maximum idx is {:?}).  Failed to add node.  {}", color::Fg(color::Red), style::Bold, q_x_idx, self._previous_checkpoint_idx.len() -1, style::Reset);
            return Err(format!("q_x_idx ({:?}) is too large in add_freespace_q_c_star (maximum idx is {:?})", q_x_idx, self._previous_checkpoint_idx.len() -1));
        }

        self._h_exploit.push( None );
        self._h_explore.push( None );
        self._h_obs.push( None );
        self._h_num.push( None );
        self._is_checkpoint.push( false );
        self._is_dead_checkpoint.push( false );
        self._predecessor_node_idx.push(Some(q_x_idx));

        let exploit = (q_c_star - &self._q_m_star).norm();
        let explore = (q_c_star - &self._q_n_star).norm();

        if self.get_is_checkpoint(q_x_idx)? {
            self._previous_checkpoint_idx.push( Some(q_x_idx) );
            if exploit < self._h_exploit[q_x_idx].unwrap().1 {
                self._h_exploit[q_x_idx] = Some( (0.0, exploit) );
            } else {
                self._h_exploit[q_x_idx].as_mut().unwrap().0 += 1.0;
            }

            if explore > self._h_explore[q_x_idx].unwrap().1 {
                self._h_explore[q_x_idx] = Some( (0.0, explore) );
            } else {
                self._h_explore[q_x_idx].as_mut().unwrap().0 += 1.0;
            }

            *self._h_num[q_x_idx].as_mut().unwrap() += 1.0;

        } else {
            self._previous_checkpoint_idx.push( self._previous_checkpoint_idx[q_x_idx].clone() );
        }

        let mut curr_idx = self._previous_checkpoint_idx[ q_x_idx ];
        while curr_idx.is_some() {
            let curr_idx_u = curr_idx.unwrap();
            if exploit < self._h_exploit[curr_idx_u].unwrap().1 {
                self._h_exploit[curr_idx_u] = Some( (0.0, exploit) );
            } else {
                self._h_exploit[curr_idx_u].as_mut().unwrap().0 += 1.0;
            }

            if explore > self._h_explore[curr_idx_u].unwrap().1 {
                self._h_explore[curr_idx_u] = Some( (0.0, explore) );
            } else {
                self._h_explore[curr_idx_u].as_mut().unwrap().0 += 1.0;
            }

            *self._h_num[curr_idx_u].as_mut().unwrap() += 1.0;

            curr_idx = self._previous_checkpoint_idx[ curr_idx_u ];
        }

        return Ok( self._h_exploit.len() - 1 );
    }

    pub fn add_collision_q_c_star(&mut self, q_x_idx: usize, q_c_star: &DVector<f64>) -> Result<(), String> {
        self._all_obs_points.push( q_c_star.clone() );
        let obs_idx = self._all_obs_points.len() - 1;
        if self._is_checkpoint[q_x_idx] {
            self._h_obs[q_x_idx].as_mut().unwrap().push( obs_idx );
            self._h_exploit[q_x_idx].as_mut().unwrap().0 += 1.0;
            self._h_explore[q_x_idx].as_mut().unwrap().0 += 1.0;
        }

        let mut curr_idx = self.get_previous_checkpoint_idx(q_x_idx)?;
        while curr_idx.is_some() {
            let curr_idx_u = curr_idx.unwrap();
            self._h_obs[curr_idx_u].as_mut().unwrap().push( obs_idx );
            self._h_exploit[curr_idx_u].as_mut().unwrap().0 += 1.0;
            self._h_explore[curr_idx_u].as_mut().unwrap().0 += 1.0;
            curr_idx = self.get_previous_checkpoint_idx(curr_idx_u)?;
        }

        return Ok(());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_exploit(&self, node_idx: usize) -> Result<&Option<(f64, f64)>, String> {
        if node_idx >= self._h_exploit.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_exploit (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._h_exploit.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_exploit (maximum idx is {:?})", node_idx, self._h_exploit.len() -1));
        }
        return Ok(&self._h_exploit[node_idx]);
    }

    pub fn get_explore(&self, node_idx: usize) -> Result<&Option<(f64, f64)>, String> {
        if node_idx >= self._h_explore.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_explore (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._h_explore.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_explore (maximum idx is {:?})", node_idx, self._h_explore.len() -1));
        }
        return Ok(&self._h_explore[node_idx]);
    }

    pub fn get_obs(&self, node_idx: usize) -> Result<&Option<Vec<usize>>, String> {
        if node_idx >= self._h_obs.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_obs (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._h_obs.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_obs (maximum idx is {:?})", node_idx, self._h_obs.len() -1));
        }
        return Ok(&self._h_obs[node_idx]);
    }

    pub fn get_obs_points(&self, node_idx: usize) -> Result<Option<Vec<&DVector<f64>>>, String> {
        let obs_idxs = self.get_obs(node_idx)?;
        if obs_idxs.is_none() { return Ok( None ); }

        let obs_idxs_u = obs_idxs.as_ref().unwrap();

        let mut out_vec = obs_idxs_u.iter().map(|x| &self._all_obs_points[*x]).collect();

        return Ok( Some( out_vec )  );

    }

    pub fn get_num(&self, node_idx: usize) -> Result<&Option<f64>, String> {
        if node_idx >= self._h_num.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_num (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._h_num.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_num (maximum idx is {:?})", node_idx, self._h_num.len() -1));
        }
        return Ok(&self._h_num[node_idx]);
    }

    pub fn get_previous_checkpoint_idx(&self, node_idx: usize) -> Result<&Option<usize>, String> {
        if node_idx >= self._previous_checkpoint_idx.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_previous_checkpoint_idx (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._previous_checkpoint_idx.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_previous_checkpoint_idx (maximum idx is {:?})", node_idx, self._previous_checkpoint_idx.len() -1));
        }
        return Ok(&self._previous_checkpoint_idx[node_idx]);
    }

    pub fn get_predecessor_node_idx(&self, node_idx: usize) -> Result<&Option<usize>, String> {
        if node_idx >= self._predecessor_node_idx.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_predecessor_node (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._predecessor_node_idx.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_predecessor_node (maximum idx is {:?})", node_idx, self._predecessor_node_idx.len() -1));
        }
        return Ok(&self._predecessor_node_idx[node_idx]);
    }

    pub fn get_is_checkpoint(&self, node_idx: usize) -> Result<bool, String> {
        if node_idx >= self._is_checkpoint.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_is_checkpoint (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._is_checkpoint.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_is_checkpoint (maximum idx is {:?})", node_idx, self._is_checkpoint.len() -1));
        }
        return Ok(self._is_checkpoint[node_idx]);
    }

    pub fn get_is_dead_checkpoint(&self, node_idx: usize) -> Result<bool, String> {
        if node_idx >= self._is_dead_checkpoint.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_is_dead_checkpoint (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._is_dead_checkpoint.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_is_dead_checkpoint (maximum idx is {:?})", node_idx, self._is_dead_checkpoint.len() -1));
        }
        return Ok(self._is_checkpoint[node_idx]);
    }

    pub fn set_as_dead_checkpoint(&mut self, node_idx: usize) -> Result<(), String> {
        if node_idx >= self._is_dead_checkpoint.len() {
            println!("{}{}ERROR: node_idx ({:?}) is too large in get_is_dead_checkpoint (maximum idx is {:?}).  Failed to get any values.  {}", color::Fg(color::Red), style::Bold, node_idx, self._is_dead_checkpoint.len() -1, style::Reset);
            return Err(format!("node_idx ({:?}) is too large in get_is_dead_checkpoint (maximum idx is {:?})", node_idx, self._is_dead_checkpoint.len() -1));
        }
        self._is_dead_checkpoint[node_idx] = true;
        return Ok(());
    }

    pub fn get_previous_checkpoint_idxs(&self) -> &Vec< Option<usize> > {
        return &self._previous_checkpoint_idx;
    }

    pub fn get_q_n_star(&self) -> &DVector<f64> {
        return &self._q_n_star;
    }

    pub fn get_q_m_star(&self) -> &DVector<f64> {
        return &self._q_m_star;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _get_obs_points_from_obs_idxs( &self, obs_idxs: &Vec<usize> ) -> Result< Vec<&DVector<f64>>, String > {
        let mut obs_points = Vec::new();
        let l = obs_idxs.len();
        for i in 0..l {
            if obs_idxs[i] >= self._all_obs_points.len() {
                println!("{}{}ERROR: obs_idx ({:?}) is too large in _get_obs_points_from_obs_idxs (maximum idx is {:?}).  Failed to get any points.  {}", color::Fg(color::Red), style::Bold, obs_idxs[i], self._all_obs_points[i].len() -1, style::Reset);
                return Err(format!("obs_idx ({:?}) is too large in _get_obs_points_from_obs_idxs (maximum idx is {:?})", obs_idxs[i], self._all_obs_points[i].len() -1));
            }
            obs_points.push( &self._all_obs_points[obs_idxs[i]] );
        }
        return Ok( obs_points );
    }

    /* Algorithm 7 */
    pub fn get_nearby_collision_points( &self, q_x_idx: usize, max_num_points_: Option<usize> ) -> Result< Vec<&DVector<f64>>, String > {
        let mut max_num_points = 4 as usize;
        if max_num_points_.is_some() { max_num_points = max_num_points_.unwrap(); }

        let mut out_vec_idxs = Vec::new();
        let mut curr_count = 0 as usize;

        if self.get_is_checkpoint(q_x_idx)? {
            let obs_idxs = self.get_obs(q_x_idx)?;

            if obs_idxs.as_ref().is_some() {
                let obs_idxs_u = obs_idxs.as_ref().unwrap();
                let l = obs_idxs_u.len();
                for i in 0..l {
                    let add_idx = obs_idxs_u[l - 1 - i];
                    if !out_vec_idxs.contains(&add_idx) {
                        out_vec_idxs.push(add_idx);
                        curr_count += 1;
                        if curr_count >= max_num_points {
                            return self._get_obs_points_from_obs_idxs(&out_vec_idxs);
                        }
                    }
                }
            }
        }

        let mut curr_idx = self.get_previous_checkpoint_idx(q_x_idx)?;
        while curr_idx.as_ref().is_some() {
            let curr_idx_u = curr_idx.unwrap();

            let obs_idxs = self.get_obs(curr_idx_u)?;

            if obs_idxs.as_ref().is_some() {
                let obs_idxs_u = obs_idxs.as_ref().unwrap();
                let l = obs_idxs_u.len();
                for i in 0..l {
                    let add_idx = obs_idxs_u[l - 1 - i];
                    if !out_vec_idxs.contains(&add_idx) {
                        out_vec_idxs.push(add_idx);
                        curr_count += 1;
                        if curr_count >= max_num_points {
                            return self._get_obs_points_from_obs_idxs(&out_vec_idxs);
                        }
                    }
                }
            }

            curr_idx = self.get_previous_checkpoint_idx(curr_idx_u)?
        }

        return self._get_obs_points_from_obs_idxs(&out_vec_idxs);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let l = self._h_exploit.len();
        for i in 0..l {
            println!("Node {:?} >>>>>> ", i);
            println!("        _h_exploit: {:?}", self._h_exploit[i]);
            println!("        _h_explore: {:?}", self._h_explore[i]);
            if self._h_obs[i].is_none() {
                println!("        _h_obs: {:?}", self._h_obs[i]);
            } else {
                let mut obs = Vec::new();
                let l = self._h_obs[i].as_ref().unwrap().len();
                for j in 0..l {
                    obs.push(  self._all_obs_points[  self._h_obs[i].as_ref().unwrap()[j].clone()  ].clone()  );
                }
                println!("        _h_obs: {:?}", obs);
            }
            println!("        _h_num: {:?}", self._h_num[i]);
            println!("        _h_previous_checkpoint_idx: {:?}", self._previous_checkpoint_idx[i]);
            println!("        _h_predecessor_node_idx: {:?}", self._predecessor_node_idx[i]);
            println!("        _h_is_checkpoint: {:?}", self._is_checkpoint[i]);
            println!();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn probability_heuristic_2(q_x_idx: usize, sprint_local_data_manager: &mut SprintLocalDataManager, kappa_: Option<f64>, debug: bool) -> Result<bool, String> {
    if sprint_local_data_manager.get_is_checkpoint(q_x_idx)? {
        if debug { println!("{}         Probability heuristic 2 automatically returned false because {:?} was already a checkpoint. {}", color::Fg(color::White), q_x_idx, style::Reset); }
        return Ok( false );
    }

    let mut kappa = 0.45;
    if kappa_.is_some() { kappa = kappa_.unwrap(); }

    // let mut all_idxs = vec![];

    let mut curr_idx = sprint_local_data_manager.get_previous_checkpoint_idx(q_x_idx)?;
    while !curr_idx.is_none() {
        let curr_idx_u = curr_idx.unwrap();
        // if sprint_local_data_manager.get_is_dead_checkpoint(curr_idx_u)? { return Ok(false); }

        // all_idxs.push(curr_idx_u);

        let h_num = sprint_local_data_manager.get_num(curr_idx_u)?.unwrap();

        if !(h_num == 1.0) {
            let exploit = sprint_local_data_manager.get_exploit(curr_idx_u)?.unwrap();
            let explore = sprint_local_data_manager.get_explore(curr_idx_u)?.unwrap();

            let x_1 = exploit.0 / h_num;
            let x_2 = explore.0 / h_num;

            let x = x_1.min(x_2);
            let c = (1.0 / h_num.log(2.0));
            let g = (-x.powi(2) / (2.0 * c * c)).exp(); // probability value between 0 and 1

            if debug { println!("{}         Probability heuristic 2 investigation of checkpoint {:?} ---> {}", color::Fg(color::White), curr_idx_u, style::Reset); }
            if debug { println!("{}            exploit: {:?}, explore: {:?}, h_num: {:?} {}", color::Fg(color::White), exploit, explore, h_num, style::Reset); }
            if debug { println!("{}            x_1: {:?}, x_2: {:?}, x: {:?}, c: {:?} {}", color::Fg(color::White), x_1, x_2, x, c, style::Reset); }
            if debug { println!("{}            g: {:?}  {}", color::Fg(color::White), g, style::Reset); }

            if g < kappa {
                // for a in all_idxs {
                    // sprint_local_data_manager.set_as_dead_checkpoint(a);
                // }

                return Ok(false);
            }
        }

        curr_idx = sprint_local_data_manager.get_previous_checkpoint_idx(curr_idx_u)?;
    }

    return Ok(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn probability_heuristic_3(q_x_idx: usize, sprint_local_data_manager: &SprintLocalDataManager, planning_tree: &PlanningDAG, lambda: f64, debug: bool) ->  Result<DVector<f64>, String> {
    let q_x = planning_tree.get_node_ref(q_x_idx)?;
    let q_m_star = sprint_local_data_manager.get_q_m_star();
    let q_p_idx = sprint_local_data_manager.get_predecessor_node_idx(q_x_idx)?;
    let mut q_p_ = None;
    if q_p_idx.is_some() {
        q_p_ = Some(planning_tree.get_node_ref(q_p_idx.unwrap())?);
    }
    let n_obs = sprint_local_data_manager.get_nearby_collision_points(q_x_idx, None)?;
    return _probability_heuristic_3_manual_inputs(q_x, q_m_star, q_p_, &n_obs, lambda, debug);
}

fn _probability_heuristic_3_manual_inputs(q_x: &DVector<f64>, q_m_star: &DVector<f64>, q_p_: Option<&DVector<f64>>, n_obs: &Vec<&DVector<f64>>, lambda: f64, debug: bool) -> Result<DVector<f64>, String> {
    let w_1 = 1.0; let w_2 = 2.0; let w_3 = 5.0;

    let mut q_p = DVector::from_element(1, 0.0);
    if q_p_.is_some() {
        q_p = q_p_.unwrap().clone();
    } else {
        let q_x_minus_q_m_star = (q_x - q_m_star);
        q_p = (q_x + lambda * (&q_x_minus_q_m_star / q_x_minus_q_m_star.norm()));
    }

    let q_x_minus_q_p = (q_x - &q_p);
    let mut q_c = q_x + &q_x_minus_q_p;

    if n_obs.len() > 0 {
        let sampler = RangeFloatVecSampler::new(-lambda / 10.0, lambda / 10.0, q_x.len() );
        q_c += sampler.float_vec_sampler_sample()?;
    }

    let q_c_tmp = q_c.clone();
    q_c += w_1 * _d_g1_d_qc(&q_x_minus_q_p);
    q_c += w_2 * _d_g2_d_qc(&q_c_tmp, q_m_star, lambda);
    if n_obs.len() > 0 {
        q_c += w_3 * _d_g3_d_qc(&q_c_tmp, q_x, n_obs, lambda);
    }

    let q_c_minus_q_x = (&q_c - q_x);
    q_c = q_x + lambda * (&q_c_minus_q_x / q_c_minus_q_x.norm());

    if debug { println!("{}         Probability heuristic 3 summary ---> {}", color::Fg(color::White), style::Reset); }
    if debug { let n_obs_list: Vec<Vec<f64>> = n_obs.iter().map(|x| x.data.as_vec().clone()).collect(); println!("{}            n_obs: {:?} {}", color::Fg(color::White), n_obs_list, style::Reset); }
    if debug { println!("{}            q_p: {:?}, q_x: {:?} {}", color::Fg(color::White), q_p.data.as_vec(), q_x.data.as_vec(), style::Reset); }

    return Ok(q_c);
}

fn _d_g1_d_qc(q_x_minus_q_p: &DVector<f64>) -> DVector<f64> {
    return q_x_minus_q_p / (q_x_minus_q_p.norm());
}

fn _d_g2_d_qc(q_c: &DVector<f64>, q_m_star: &DVector<f64>, lambda: f64) -> DVector<f64> {
    let q_m_star_minus_q_c = (q_m_star - q_c);
    let q_m_star_minus_q_c_norm = q_m_star_minus_q_c.norm();

    let psi_2 = ( -q_m_star_minus_q_c_norm.powi(2) / (4.0 * lambda * lambda) ).exp() + 1.0;

    return psi_2 * (&q_m_star_minus_q_c / q_m_star_minus_q_c_norm);
}

fn _d_g3_d_qc(q_c: &DVector<f64>, q_x: &DVector<f64>, n_obs: &Vec<&DVector<f64>>, lambda: f64) -> DVector<f64> {
    let num_n_obs = n_obs.len();
    let num_n_obs_as_f64 = num_n_obs as f64;

    let mut d_g3_d_qc = DVector::from_element(q_c.len(), 0.0);

    for i in 0..num_n_obs {
        let proj_scalar = proj_scalar(n_obs[i], q_x, q_c);
        let psi_3_1 = heaviside(proj_scalar);
        if psi_3_1 == 1.0 {
            let proj = proj(n_obs[i], q_x, q_c);

            let proj_minus_q_obs = (&proj - n_obs[i]);
            let proj_minus_q_obs_norm = proj_minus_q_obs.norm();
            // let q_x_minus_q_obs = (q_x - n_obs[i]);

            // let psi_3_2 = 5.0*(-q_x_minus_q_obs.norm().powi(2) / (6.0 * lambda * lambda )).exp();
            let psi_3_3 = 5.0*(-proj_minus_q_obs_norm.powi(2) / (4.0 * lambda * lambda)).exp();

            d_g3_d_qc += psi_3_1 * 1.0 * psi_3_3 * (&proj_minus_q_obs / proj_minus_q_obs_norm);
        }
    }

    return (1.0 / (num_n_obs_as_f64)) * d_g3_d_qc;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
