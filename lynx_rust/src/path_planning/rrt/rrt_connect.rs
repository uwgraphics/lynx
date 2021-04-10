use crate::utils::utils_path_planning::{local_search::*, global_search::*};
use crate::utils::utils_collisions::collision_checker::*;
use crate::utils::utils_recorders::prelude::*;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use crate::utils::utils_path_planning::path_planner_result::PathPlannerResult;
use crate::utils::utils_path_planning::utils_planning_graphs::planning_dag::*;
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_nearest_neighbor::kdtree_utils::*;
use crate::utils::utils_collisions::collision_check_result_enum::CollisionCheckResult;
use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::utils::utils_sampling::float_vec_sampler_traits::FloatVecSampler;
use crate::utils::utils_files_and_strings::string_utils::*;
use termion::{style, color};
use nalgebra::DVector;
use std::sync::RwLock;

#[derive(Clone)]
pub struct RRTConnect {
    _sampler: LynxFloatVecSamplerBox,
    _collision_checker: CollisionCheckerBox,
    _lambda: f64,
    _max_num_collision_checks: usize
}

impl RRTConnect {
    pub fn new(sampler: LynxFloatVecSamplerBox, collision_checker: CollisionCheckerBox, lambda: f64, max_num_collision_checks: usize) -> Self {
        Self { _sampler: sampler, _collision_checker: collision_checker, _lambda: lambda, _max_num_collision_checks: max_num_collision_checks }
    }
    fn _solve_single_threaded(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let mut planning_tree_a = PlanningDAG::new_unidirectional(q_init);
        let mut planning_tree_b = PlanningDAG::new_unidirectional(q_goal);
        let mut kdtree_a = KDTree::new_empty(q_init.len());
        let mut kdtree_b = KDTree::new_empty(q_init.len());
        kdtree_a.add_point(q_init)?;
        kdtree_b.add_point(q_goal)?;

        let mut curr_planning_tree = &mut planning_tree_a;
        let mut curr_kdtree = &mut kdtree_a;
        let mut other_planning_tree = &mut planning_tree_b;
        let mut other_kdtree = &mut kdtree_b;
        let mut curr_group = "a";
        let mut num_collision_checks = 0;

        loop {
            let sample = self._sampler.lynx_float_vec_sampler_sample(lynx_vars)?;
            let q_closest_res = curr_kdtree.get_closest_k(&sample, 1)[0];
            let q_closest_point_ref = curr_kdtree.get_point_ref_from_tuple(q_closest_res)?;

            let local_search_res = local_search.solve_local(q_closest_point_ref, &sample, lynx_vars, recorder, terminate)?;
            match local_search_res {
                PathPlannerResult::SolutionFound(s) => {
                    ////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////
                    let mut last_add_idx = q_closest_res.0;
                    let l = s.waypoints.len();
                    for i in 1..l {
                        let add_idx = curr_planning_tree.add_node_with_auto_two_waypoint_inflow_edge(&s.waypoints[i], last_add_idx)?;
                        curr_kdtree.add_point(&s.waypoints[i]);
                        last_add_idx = add_idx;
                    }

                    let q_init_ref = curr_planning_tree.get_node_ref(last_add_idx)?;
                    let q_closest_res_in_other_tree = other_kdtree.get_closest_k(q_init_ref, 1)[0];
                    let q_closest_point_in_other_tree_ref = other_kdtree.get_point_ref_from_tuple(q_closest_res_in_other_tree)?;
                    let local_search_to_other_tree_res = local_search.solve_local(q_init_ref, q_closest_point_in_other_tree_ref, lynx_vars, recorder, terminate)?;
                    match local_search_to_other_tree_res {
                        PathPlannerResult::SolutionFound(s) => {
                            let l = s.waypoints.len();
                            for i in 1..l {
                                let add_idx = curr_planning_tree.add_node_with_auto_two_waypoint_inflow_edge(&s.waypoints[i], last_add_idx)?;
                                last_add_idx = add_idx;
                            }
                            let path1 = curr_planning_tree.get_path_from_tree_root_to_node(last_add_idx)?;
                            let mut path2 = other_planning_tree.get_path_from_tree_root_to_node(q_closest_res_in_other_tree.0)?;
                            path2.reverse();
                            let mut out_path = path1.combine_ordered(&path2)?;
                            if curr_group == "b" { out_path.reverse(); }
                            return Ok(PathPlannerResult::SolutionFound(out_path));
                        }
                        PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(_) => {}
                        PathPlannerResult::SolutionNotFound(_) => {}
                    }
                    ////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////
                }
                PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(s) => {
                    ////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////
                    let mut last_add_idx = q_closest_res.0;
                    let l = s.waypoints.len();
                    for i in 1..l {
                        let add_idx = curr_planning_tree.add_node_with_auto_two_waypoint_inflow_edge(&s.waypoints[i], last_add_idx)?;
                        curr_kdtree.add_point(&s.waypoints[i]);
                        last_add_idx = add_idx;
                    }

                    let q_init_ref = curr_planning_tree.get_node_ref(last_add_idx)?;
                    let q_closest_res_in_other_tree = other_kdtree.get_closest(q_init_ref)?;
                    let q_closest_point_in_other_tree_ref = other_kdtree.get_point_ref_from_tuple(q_closest_res_in_other_tree)?;
                    let local_search_to_other_tree_res = local_search.solve_local(q_init_ref, q_closest_point_in_other_tree_ref, lynx_vars, recorder, terminate)?;
                    match local_search_to_other_tree_res {
                        PathPlannerResult::SolutionFound(s) => {
                            num_collision_checks += s.waypoints.len();
                            let l = s.waypoints.len();
                            for i in 1..l {
                                let add_idx = curr_planning_tree.add_node_with_auto_two_waypoint_inflow_edge(&s.waypoints[i], last_add_idx)?;
                                last_add_idx = add_idx;
                            }
                            let path1 = curr_planning_tree.get_path_from_tree_root_to_node(last_add_idx)?;
                            let mut path2 = other_planning_tree.get_path_from_tree_root_to_node(q_closest_res_in_other_tree.0)?;
                            path2.reverse();
                            let mut out_path = path1.combine_ordered(&path2)?;
                            if curr_group == "b" { out_path.reverse(); }
                            return Ok(PathPlannerResult::SolutionFound(out_path));
                        }
                        PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(s) => { num_collision_checks += s.waypoints.len(); }
                        PathPlannerResult::SolutionNotFound(_) => {}
                    }
                    ////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////
                }
                PathPlannerResult::SolutionNotFound(s) => {}
            }

            if curr_group == "a" {
                curr_planning_tree = &mut planning_tree_b;
                curr_kdtree = &mut kdtree_b;
                other_planning_tree = &mut planning_tree_a;
                other_kdtree = &mut kdtree_a;
                curr_group = "b";
            } else {
                curr_planning_tree = &mut planning_tree_a;
                curr_kdtree = &mut kdtree_a;
                other_planning_tree = &mut planning_tree_b;
                other_kdtree = &mut kdtree_b;
                curr_group = "a";
            }

            if num_collision_checks > self._max_num_collision_checks {
                return Ok(PathPlannerResult::SolutionNotFound(format!("Exceeded number of maximum collision checks in RRTConnect ({:?})", self._max_num_collision_checks)));
            }
        }
    }
    fn _solve_parallel(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        unimplemented!()
    }
}

impl GlobalSearch for RRTConnect {
    fn solve_global(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        return match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => { self._solve_single_threaded(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
            LynxVarsGeneric::SingleThreadedMutRef(_) => { self._solve_single_threaded(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
            LynxVarsGeneric::Parallel(_) => { self._solve_single_threaded(q_init, q_goal, local_search, lynx_vars, recorder, terminate) }
        }
    }
    fn name_global(&self) -> String {
        let name = "RRTConnect_".to_string() + usize_to_string(self._max_num_collision_checks).as_str();
        return name;
    }
}
impl LocalSearch for RRTConnect {
    fn solve_local(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let local_search_box = StraightLineLocalSearch::new(self._collision_checker.clone(), self._lambda).to_local_search_box();
        return self.solve_global(q_init, q_goal, &local_search_box, lynx_vars, recorder, terminate);
    }
    fn name_local(&self) -> String {
        let name = "RRTConnect_".to_string() + usize_to_string(self._max_num_collision_checks).as_str();
        return name;
    }
}
impl LynxVarsUser for RRTConnect { }
