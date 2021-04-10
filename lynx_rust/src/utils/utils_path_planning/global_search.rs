use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_collisions::collision_checker::*;
use crate::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use crate::utils::utils_path_planning::local_search::*;
use crate::utils::utils_path_planning::path_planner_result::PathPlannerResult;
use nalgebra::DVector;
use std::sync::{Arc, RwLock};

pub trait GlobalSearch: Send + Sync + LynxVarsUser + AsLynxVarsUser + GlobalSearchClone {
    fn solve_global(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String>;
    fn to_global_search_box(&self) -> GlobalSearchBox {
        return GlobalSearchBox(self.clone_box());
    }
    fn name_global(&self) -> String;
}

pub trait GlobalSearchClone {
    fn clone_box(&self) -> Box<dyn GlobalSearch>;
}
impl<T> GlobalSearchClone for T where T: 'static + GlobalSearch + Clone {
    fn clone_box(&self) -> Box<dyn GlobalSearch> {
        Box::new(self.clone())
    }
}

pub struct GlobalSearchBox(Box<dyn GlobalSearch>);
impl GlobalSearchBox {
    pub fn new(global_search: &dyn GlobalSearch) -> Self {
        return Self( global_search.clone_box() );
    }

    pub fn solve_global(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        return self.0.solve_global(q_init, q_goal, local_search, lynx_vars, recorder, terminate);
    }

    pub fn name_global(&self) -> String { return self.0.name_global(); }
}
impl Clone for GlobalSearchBox {
    fn clone(&self) -> Self {
        let c = self.0.clone_box();
        return Self(c);
    }
}
unsafe impl Send for GlobalSearchBox { }
unsafe impl Sync for GlobalSearchBox { }

