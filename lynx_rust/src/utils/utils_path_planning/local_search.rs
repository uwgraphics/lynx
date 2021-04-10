use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_collisions::collision_checker::*;
use crate::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use crate::utils::utils_path_planning::path_planner_result::PathPlannerResult;
use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::utils::utils_collisions::collision_check_result_enum::CollisionCheckResult;
use nalgebra::DVector;
use std::sync::{Arc, RwLock};

pub trait LocalSearch: Send + Sync + LynxVarsUser + AsLynxVarsUser + LocalSearchClone {
    fn solve_local(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String>;
    fn to_local_search_box(&self) -> LocalSearchBox {
        return LocalSearchBox(self.clone_box());
    }
    fn name_local(&self) -> String;
}

pub trait LocalSearchClone {
    fn clone_box(&self) -> Box<dyn LocalSearch>;
}
impl<T> LocalSearchClone for T where T: 'static + LocalSearch + Clone {
    fn clone_box(&self) -> Box<dyn LocalSearch> {
        Box::new(self.clone())
    }
}

pub struct LocalSearchBox(Box<dyn LocalSearch>);
impl LocalSearchBox {
    pub fn new(local_search: &dyn LocalSearch) -> Self {
        return Self( local_search.clone_box() );
    }

    pub fn solve_local(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        return self.0.solve_local(q_init, q_goal, lynx_vars, recorder, terminate);
    }

    pub fn name_local(&self) -> String { return self.0.name_local(); }
}
impl Clone for LocalSearchBox {
    fn clone(&self) -> Self {
        let c = self.0.clone_box();
        return Self(c);
    }
}
unsafe impl Send for LocalSearchBox { }
unsafe impl Sync for LocalSearchBox { }

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct StraightLineLocalSearch {
    _collision_checker: CollisionCheckerBox,
    _lambda: f64
}
impl StraightLineLocalSearch {
    pub fn new(collision_checker: CollisionCheckerBox, lambda: f64) -> Self {
        Self { _collision_checker: collision_checker, _lambda: lambda }
    }
}
impl LocalSearch for StraightLineLocalSearch {
    fn solve_local(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        let mut out_path = LinearSplinePath::new_empty();

        let mut dir = q_goal - q_init;
        dir = &dir / dir.norm();

        let mut curr_point = q_init.clone();
        let mut is_goal = false;

        loop {
            if terminate.get_terminate() {
                return Ok(PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(out_path));
            }

            let in_collision_res = self._collision_checker.in_collision(&curr_point, lynx_vars)?;
            match in_collision_res {
                CollisionCheckResult::NotInCollision => {
                    out_path.add_waypoint(&curr_point);
                    if is_goal {
                        return Ok(PathPlannerResult::SolutionFound(out_path));
                    }

                    let new_point = &curr_point + self._lambda * &dir;

                    let d1 = (&curr_point - q_goal).norm();
                    let d2 = (&new_point - q_goal).norm();
                    if d1 < d2 {
                        curr_point = q_goal.clone();
                        is_goal = true;
                    } else {
                        curr_point = new_point.clone();
                    }
                }
                CollisionCheckResult::InCollision(s) => { return Ok(PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(out_path)); }
                CollisionCheckResult::Error(s) => { return Err(s); }
            }
        }
    }
    fn name_local(&self) -> String { return "StraightLine".to_string(); }
}
impl LynxVarsUser for StraightLineLocalSearch { }