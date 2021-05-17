use crate::utils::utils_path_planning::{local_search::*, global_search::*};
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use crate::utils::utils_runtime_management::termination_util::*;
use crate::utils::utils_path_planning::path_planner_result::*;
use crate::utils::utils_path_planning::utils_surge::prelude::*;
use termion::{style, color};
use nalgebra::DVector;
use std::sync::RwLock;
use rayon::prelude::*;
use rayon::iter::Zip;
use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use std::ops::Deref;
use meshopt::ErrorKind::Path;

#[derive(Clone)]
pub struct SurgeGlobal {
    _surge_objective_terms: Vec<SurgeObjectiveTermBox>,
    _surge_objective_term_weights: Vec<f64>,
    _milestone_sampler: LynxMultiFloatVecSamplerBox,
    _unidirectional: bool,
    _num_milestones: usize,
    _surge_parallel_mode: SurgeParallelMode
}

impl SurgeGlobal {
    pub fn new(surge_objective_terms: Vec<SurgeObjectiveTermBox>, surge_objective_term_weights: Vec<f64>, milestone_sampler: LynxMultiFloatVecSamplerBox, unidirectional: bool, num_milestones: usize, surge_parallel_mode: SurgeParallelMode) -> Result<Self, String> {
        if !(surge_objective_terms.len() == surge_objective_term_weights.len()) {
            return Err(format!("number of surge_objective_terms ({:?}) not equal to number of weights; ({:?})", surge_objective_terms.len(), surge_objective_term_weights.len()));
        }

        Ok(Self { _surge_objective_terms: surge_objective_terms, _surge_objective_term_weights: surge_objective_term_weights, _milestone_sampler: milestone_sampler, _unidirectional: unidirectional,_num_milestones: num_milestones, _surge_parallel_mode: surge_parallel_mode })
    }

    fn _solve_single_threaded_forward(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let start_to_goal_local_search = local_search.solve_local(q_init, q_goal, lynx_vars, recorder, terminate)?;
        match start_to_goal_local_search {
            PathPlannerResult::SolutionFound(s) => {
                return Ok(PathPlannerResult::SolutionFound(s));
            }
            _ => {}
        }

        let mut surge = SurgeBidirectional::new_empty();
        let l = self._surge_objective_terms.len();
        for i in 0..l {
            surge.add_surge_objective_term_box(self._surge_objective_terms[i].clone(), Some(self._surge_objective_term_weights[i]));
        }
        surge.add_start_state(q_init);
        surge.add_end_state(q_goal);

        let milestone_samples = self._milestone_sampler.lynx_multi_float_vec_sampler_sample(lynx_vars, self._num_milestones, self._num_milestones * 10, false)?;

        for m in &milestone_samples {
            surge.add_milestone_state( m );
        }

        surge.initialize_all_surge_objective_terms_with_surge_points_manager(lynx_vars, recorder)?;

        let lower_is_better = surge.get_lower_is_better()?;

        let mut count = 1;
        loop {
            // println!("loop {:?}", count); count+=1;
            if terminate.get_terminate() {
                return Ok(PathPlannerResult::SolutionNotFound("early terminate".to_string()));
            }

            let start = Instant::now();
            let n_best = surge.get_n_best_candidate_connections(1, lower_is_better, self._unidirectional, lynx_vars, recorder)?;
            // println!(">>> {:?}", start.elapsed());
            // println!("{:?}", n_best);

            if n_best.len() == 0 {
                let mut new_surge_global = Self::new(self._surge_objective_terms.clone(),
                                                     self._surge_objective_term_weights.clone(),
                                                     self._milestone_sampler.clone(),
                                                     self._unidirectional.clone(),
                                                     (self._num_milestones as f64 * 1.5) as usize,
                                                     self._surge_parallel_mode.clone())?;

                return new_surge_global._solve_single_threaded_forward(q_init, q_goal, local_search, lynx_vars, recorder, terminate);
            }

            let best = &n_best[0];
            surge.set_dead_connection(best.idx1, best.idx2);

            let q_n_star = &best.point1;
            let q_m_star = &best.point2;

            let start = Instant::now();
            let local_search_result = local_search.solve_local(q_n_star, q_m_star, lynx_vars, recorder, &mut TerminationUtilOption::new_none())?;
            let stop = start.elapsed();

            match local_search_result {
                PathPlannerResult::SolutionFound(s) => {
                    // println!("success, {:?}", stop);
                    surge.add_successful_connection(best.idx1, best.idx2, &s)?;
                    surge.update_all_surge_objective_terms_after_connection_attempt(best.idx1, best.idx2, true, lynx_vars, recorder)?;
                    if surge.has_at_least_one_solution_been_found() {
                        return Ok(PathPlannerResult::SolutionFound(surge.get_first_solution_path()?.unwrap()));
                    }
                }
                PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(_) => {
                    // println!("partial, {:?}", stop);
                    surge.update_all_surge_objective_terms_after_connection_attempt(best.idx1, best.idx2, false, lynx_vars, recorder)?;
                }
                PathPlannerResult::SolutionNotFound(_) => {
                    // println!("fail, {:?}", stop);
                    surge.update_all_surge_objective_terms_after_connection_attempt(best.idx1, best.idx2, false, lynx_vars, recorder)?;
                }
            }
            // println!("------------------------");
        }
    }

    fn _solve_single_threaded_reverse(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let path_planner_result = self._solve_single_threaded_forward(q_goal, q_init, local_search, lynx_vars, recorder, terminate)?;
        return match path_planner_result {
            PathPlannerResult::SolutionFound(s) => {
                let mut path_copy = s.clone();
                path_copy.reverse();
                Ok(PathPlannerResult::SolutionFound(path_copy))
            }
            PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(s) => {
                let mut partial_path_copy = s.clone();
                partial_path_copy.reverse();
                Ok(PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(partial_path_copy))
            }
            PathPlannerResult::SolutionNotFound(s) => { Ok(PathPlannerResult::SolutionNotFound(s)) }
        }
    }

    fn _solve_single_threaded_random_forward_or_reverse(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let sampler = RangeFloatVecSampler::new(0.0, 1.0, 1);
        let s = sampler.float_vec_sampler_sample()?;
        return if s[0] > 0.5 {
            self._solve_single_threaded_forward(q_init, q_goal, local_search, lynx_vars, recorder, terminate)
        } else {
            self._solve_single_threaded_reverse(q_init, q_goal, local_search, lynx_vars, recorder, terminate)
        }
    }

    fn _solve_parallel_batched(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let mut surge = SurgeBidirectional::new_empty();
        let l = self._surge_objective_terms.len();
        for i in 0..l {
            surge.add_surge_objective_term_box(self._surge_objective_terms[i].clone(), Some(self._surge_objective_term_weights[i]));
        }
        surge.add_start_state(q_init);
        surge.add_end_state(q_goal);

        let milestone_samples = self._milestone_sampler.lynx_multi_float_vec_sampler_sample(lynx_vars, self._num_milestones, self._num_milestones * 10, false)?;

        for m in &milestone_samples {
            surge.add_milestone_state( m );
        }

        surge.initialize_all_surge_objective_terms_with_surge_points_manager(lynx_vars, recorder)?;

        let lower_is_better = surge.get_lower_is_better()?;
        let num_threads = lynx_vars.get_num_threads();

        let mut surge_rwlock = RwLock::new(surge);

        let mut count = 1;
        loop {
            // println!("loop {:?}", count); count+=1;
            if terminate.get_terminate() {
                return Ok(PathPlannerResult::SolutionNotFound("early terminate".to_string()));
            }

            let start = Instant::now();
            let mut n_best : Vec<SurgeCandidateConnection> = surge_rwlock.write().unwrap().get_n_best_candidate_connections(num_threads, lower_is_better, self._unidirectional, lynx_vars, recorder)?;
            // println!(">>> {:?}", start.elapsed());

            if n_best.len() == 0 { return Ok(PathPlannerResult::SolutionNotFound("n_best_candidate_connections was empty".to_string())); }

            surge_rwlock.write().unwrap().set_dead_connections_from_candidate_connections(&n_best);

            match lynx_vars {
                LynxVarsGeneric::SingleThreaded(_) => { return Err("called parallel function with single threaded lynx_vars".to_string()); }
                LynxVarsGeneric::SingleThreadedMutRef(_) => { return Err("called parallel function with single threaded lynx_vars".to_string()); }
                LynxVarsGeneric::Parallel(l) => {
                    let mut piter = l.get_par_iter_mut();
                    let mut piter_zip : Zip<rayon::slice::IterMut<LynxVars>, rayon::slice::Iter<SurgeCandidateConnection>> = piter.zip(n_best.par_iter());
                    piter_zip.for_each(|(lynx_vars_on_thread, surge_candidate_connection)| {
                        let q_n_star = &surge_candidate_connection.point1;
                        let q_m_star = &surge_candidate_connection.point2;

                        let start = Instant::now();
                        let local_search_result = local_search.solve_local(q_n_star, q_m_star, &mut LynxVarsGeneric::SingleThreadedMutRef(lynx_vars_on_thread), recorder, &mut TerminationUtilOption::new_none());
                        let stop = start.elapsed();

                        if local_search_result.is_err() { return; }
                        else {
                            let local_search_result_unwrap = local_search_result.unwrap();
                            match local_search_result_unwrap {
                                PathPlannerResult::SolutionFound(s) => {
                                    // println!("success, {:?}", stop);
                                    surge_rwlock.write().unwrap().add_successful_connection(surge_candidate_connection.idx1, surge_candidate_connection.idx2, &s);
                                    surge_rwlock.write().unwrap().update_all_surge_objective_terms_after_connection_attempt(surge_candidate_connection.idx1, surge_candidate_connection.idx2, true, &mut LynxVarsGeneric::SingleThreadedMutRef(lynx_vars_on_thread), recorder);
                                }
                                PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(_) => {
                                    // println!("partial, {:?}", stop);
                                    surge_rwlock.write().unwrap().update_all_surge_objective_terms_after_connection_attempt(surge_candidate_connection.idx1, surge_candidate_connection.idx2, false, &mut LynxVarsGeneric::SingleThreadedMutRef(lynx_vars_on_thread), recorder);
                                }
                                PathPlannerResult::SolutionNotFound(_) => {
                                    // println!("fail, {:?}", stop);
                                    surge_rwlock.write().unwrap().update_all_surge_objective_terms_after_connection_attempt(surge_candidate_connection.idx1, surge_candidate_connection.idx2, false, &mut LynxVarsGeneric::SingleThreadedMutRef(lynx_vars_on_thread), recorder);
                                }
                            }
                        }
                    });
                }
            }

            if surge_rwlock.read().unwrap().has_at_least_one_solution_been_found() {
                return Ok(PathPlannerResult::SolutionFound(surge_rwlock.read().unwrap().get_first_solution_path()?.unwrap()));
            }
            // println!("------------------------");
        }
    }

    fn _solve_parallel_continuous(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let mut surge = SurgeBidirectional::new_empty();
        let l = self._surge_objective_terms.len();
        for i in 0..l {
            surge.add_surge_objective_term_box(self._surge_objective_terms[i].clone(), Some(self._surge_objective_term_weights[i]));
        }
        surge.add_start_state(q_init);
        surge.add_end_state(q_goal);

        let milestone_samples = self._milestone_sampler.lynx_multi_float_vec_sampler_sample(lynx_vars, self._num_milestones, self._num_milestones * 10, false)?;

        for m in &milestone_samples {
            surge.add_milestone_state( m );
        }

        surge.initialize_all_surge_objective_terms_with_surge_points_manager(lynx_vars, recorder)?;

        let lower_is_better = surge.get_lower_is_better()?;
        let num_threads = lynx_vars.get_num_threads();

        let mut surge_rwlock = RwLock::new(surge);
        let mut out_solution = RwLock::new(LinearSplinePath::new_empty());

        match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => { return Err("called parallel function with single threaded lynx_vars".to_string()); }
            LynxVarsGeneric::SingleThreadedMutRef(_) => { return Err("called parallel function with single threaded lynx_vars".to_string()); }
            LynxVarsGeneric::Parallel(l) => {
                let mut terminate_local = TerminationUtilOption::new();
                let par_iter = l.get_par_iter_mut();
                par_iter.for_each(|x| {
                    let mut terminate_on_thread = terminate_local.clone();
                    let recorder = &RecorderArcMutexOption::new_none();
                    loop {
                        if terminate_on_thread.get_terminate() { return; }
                        if terminate.get_terminate() { return; }

                        let surge_candidate_connection_res = surge_rwlock.write().as_mut().unwrap().get_n_best_candidate_connections(1, lower_is_better, self._unidirectional, &mut LynxVarsGeneric::SingleThreadedMutRef(x), recorder);
                        if surge_candidate_connection_res.is_err() { continue; }
                        let surge_candidate_connection_unwrap = surge_candidate_connection_res.as_ref().ok().unwrap();
                        if surge_candidate_connection_unwrap.len() == 0 { return; }
                        surge_rwlock.write().as_mut().unwrap().set_dead_connections_from_candidate_connections(surge_candidate_connection_unwrap);
                        let surge_candidate_connection = &surge_candidate_connection_unwrap[0];

                        let q_n_star = &surge_candidate_connection.point1;
                        let q_m_star = &surge_candidate_connection.point2;

                        let local_search_result = local_search.solve_local(q_n_star, q_m_star, &mut LynxVarsGeneric::SingleThreadedMutRef(x), recorder, &mut terminate_on_thread);
                        if local_search_result.is_err() { continue; }
                        let local_search_result_unwrap = local_search_result.as_ref().ok().unwrap();
                        match local_search_result_unwrap {
                            PathPlannerResult::SolutionFound(s) => {
                                // println!("success, {:?}", stop);
                                surge_rwlock.write().unwrap().add_successful_connection(surge_candidate_connection.idx1, surge_candidate_connection.idx2, &s);
                                surge_rwlock.write().unwrap().update_all_surge_objective_terms_after_connection_attempt(surge_candidate_connection.idx1, surge_candidate_connection.idx2, true, &mut LynxVarsGeneric::SingleThreadedMutRef(x), recorder);
                            }
                            PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(_) => {
                                // println!("partial, {:?}", stop);
                                surge_rwlock.write().unwrap().update_all_surge_objective_terms_after_connection_attempt(surge_candidate_connection.idx1, surge_candidate_connection.idx2, false, &mut LynxVarsGeneric::SingleThreadedMutRef(x), recorder);
                            }
                            PathPlannerResult::SolutionNotFound(_) => {
                                // println!("fail, {:?}", stop);
                                if terminate_on_thread.get_terminate() { return; }
                                surge_rwlock.write().unwrap().update_all_surge_objective_terms_after_connection_attempt(surge_candidate_connection.idx1, surge_candidate_connection.idx2, false, &mut LynxVarsGeneric::SingleThreadedMutRef(x), recorder);
                            }
                        }

                        if surge_rwlock.read().unwrap().has_at_least_one_solution_been_found() {
                            let first_solution_path_res = surge_rwlock.read().unwrap().get_first_solution_path();
                            if first_solution_path_res.is_ok() {
                                *out_solution.write().unwrap() = first_solution_path_res.ok().unwrap().unwrap();
                                terminate_on_thread.set_to_terminate();
                            }
                        }
                    }
                });
            }
        }

        let out_solution_unwrap = out_solution.read().unwrap();
        return if out_solution_unwrap.waypoints.is_empty() {
            Ok(PathPlannerResult::SolutionNotFound("no solution found.".to_string()))
        } else {
            Ok(PathPlannerResult::SolutionFound(out_solution_unwrap.clone()))
        }
    }

    fn _solve_parallel_independent(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let mut out_solution = RwLock::new(LinearSplinePath::new_empty());

        match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => { return Err("called parallel function with single threaded lynx_vars".to_string()); }
            LynxVarsGeneric::SingleThreadedMutRef(_) => { return Err("called parallel function with single threaded lynx_vars".to_string()); }
            LynxVarsGeneric::Parallel(l) => {
                let mut terminate_local = TerminationUtilOption::new();
                let par_iter = l.get_par_iter_mut();
                par_iter.for_each(|x| {
                    let mut terminate_on_thread = terminate_local.clone();
                    let recorder = &RecorderArcMutexOption::new_none();
                    let res = self._solve_single_threaded_random_forward_or_reverse(q_init, q_goal, local_search, &mut LynxVarsGeneric::SingleThreadedMutRef(x), recorder, &mut terminate_on_thread).expect("error on global search on one thread");

                    match res {
                        PathPlannerResult::SolutionFound(s) => {
                            terminate_on_thread.set_to_terminate();
                            *out_solution.write().unwrap() = s;
                        }
                        _ => { }
                    }
                });
            }
        }

        let out_solution_unwrap = out_solution.read().unwrap();
        return if out_solution_unwrap.waypoints.is_empty() {
            Ok(PathPlannerResult::SolutionNotFound("no solution found.".to_string()))
        } else {
            Ok(PathPlannerResult::SolutionFound(out_solution_unwrap.clone()))
        }
    }
}

impl GlobalSearch for SurgeGlobal {
    fn solve_global(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        return match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => { self._solve_single_threaded_forward(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
            LynxVarsGeneric::SingleThreadedMutRef(_) => { self._solve_single_threaded_forward(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
            LynxVarsGeneric::Parallel(_) => {
                match self._surge_parallel_mode {
                    SurgeParallelMode::ForceSingleThreaded => { self._solve_single_threaded_forward(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
                    SurgeParallelMode::Batched => { self._solve_parallel_batched(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
                    SurgeParallelMode::Continuous => { self._solve_parallel_continuous(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
                    SurgeParallelMode::Independent => { self._solve_parallel_independent(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
                }
            }
        }
    }
    fn name_global(&self) -> String { return "SurgeGlobal".to_string(); }
}
impl LynxVarsUser for SurgeGlobal { }

#[derive(Clone)]
pub enum SurgeParallelMode {
    ForceSingleThreaded, Batched, Continuous, Independent
}

