use crate::utils::utils_path_planning::local_search::*;
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
pub struct RRT {
    _sampler: LynxFloatVecSamplerBox,
    _collision_checker: CollisionCheckerBox,
    _lambda: f64,
    _max_num_collision_checks: usize
}

impl RRT {
    pub fn new(sampler: LynxFloatVecSamplerBox, collision_checker: CollisionCheckerBox, lambda: f64, max_num_collision_checks: usize) -> Self {
        Self { _sampler: sampler, _collision_checker: collision_checker, _lambda: lambda, _max_num_collision_checks: max_num_collision_checks }
    }

    fn _solve_single_threaded(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let to_goal_sampler = RangeFloatVecSampler::new(0.0, 1.0, 1);
        let mut planning_tree = PlanningDAG::new_unidirectional(q_init);
        let mut kdtree = KDTree::new_empty(q_init.len());
        kdtree.add_point(q_init)?;
        let mut num_collision_checks = 0;

        loop {
            let mut q_rand = q_goal.clone();
            let mut to_goal_sample = to_goal_sampler.float_vec_sampler_sample()?[0];
            if to_goal_sample > 0.1 { q_rand = self._sampler.lynx_float_vec_sampler_sample(lynx_vars)?; }

            let mut q_new = q_rand.clone();
            let mut q_closest_res = kdtree.get_closest(&q_rand)?;
            let mut q_closest_point_ref = kdtree.get_point_ref_from_tuple(q_closest_res)?;
            let dir = (&q_new - q_closest_point_ref);
            let dir_n = dir.norm();
            if dir_n > self._lambda {
                q_new = q_closest_point_ref + self._lambda * ( &dir / dir_n );
            }

            let collision_check_res = self._collision_checker.in_collision(&q_new, lynx_vars)?;
            num_collision_checks += 1;
            match collision_check_res {
                CollisionCheckResult::NotInCollision => {
                    let add_idx = planning_tree.add_node_with_auto_two_waypoint_inflow_edge(&q_new, q_closest_res.0)?;
                    kdtree.add_point(&q_new)?;
                    if (&q_new - q_goal).norm() < self._lambda * 1.5 {
                        let add_idx = planning_tree.add_node_with_auto_two_waypoint_inflow_edge(q_goal, add_idx)?;
                        let solution_path = planning_tree.get_path_from_tree_root_to_node(add_idx)?;
                        return Ok(PathPlannerResult::SolutionFound(solution_path));
                    }
                }
                CollisionCheckResult::InCollision(_) => { }
                CollisionCheckResult::Error(s) => { return Err(s); }
            }

            if num_collision_checks >= self._max_num_collision_checks {
                return Ok(PathPlannerResult::SolutionNotFound(format!("Exceeded number of maximum collision checks in RRT ({:?})", self._max_num_collision_checks)));
            }
        }
    }

    fn _solve_parallel(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        unimplemented!()
    }
}

impl LocalSearch for RRT {
    fn solve_local(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        return match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => { self._solve_single_threaded(q_init, q_goal, lynx_vars, recorder, terminate) }
            LynxVarsGeneric::SingleThreadedMutRef(_) => { self._solve_single_threaded(q_init, q_goal, lynx_vars, recorder, terminate) }
            LynxVarsGeneric::Parallel(_) => { self._solve_single_threaded(q_init, q_goal, lynx_vars, recorder, terminate) }
        }
    }
    fn name_local(&self) -> String {
        let name = "RRT_".to_string() + usize_to_string(self._max_num_collision_checks).as_str();
        return name;
    }
}
impl LynxVarsUser for RRT { }