use crate::path_planning::surge_global::*;
use crate::utils::utils_path_planning::{local_search::*, global_search::*};
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use crate::utils::utils_runtime_management::termination_util::*;
use crate::utils::utils_path_planning::path_planner_result::*;
use crate::utils::utils_path_planning::utils_surge::prelude::*;
use termion::{style, color};
use nalgebra::DVector;

#[derive(Clone)]
pub struct SprintGlobal {
    _surge_global: SurgeGlobal
}

impl SprintGlobal {
    pub fn new(milestone_sampler: LynxMultiFloatVecSamplerBox, unidirectional: bool, num_milestones: usize, surge_parallel_mode: SurgeParallelMode) -> Self {
        let mut surge_objective_terms = Vec::new();
        surge_objective_terms.push( SurgeObjectivePairwiseDis.to_surge_objective_term_box() );
        surge_objective_terms.push( SurgeObjectiveDisToEndStateSet2::new_empty().to_surge_objective_term_box() );

        let surge_objective_term_weights = vec![1.0, 2.0];

        let mut surge_global = SurgeGlobal::new(surge_objective_terms, surge_objective_term_weights, milestone_sampler, unidirectional, num_milestones, surge_parallel_mode).unwrap();
        return Self { _surge_global: surge_global };
    }
}

impl GlobalSearch for SprintGlobal {
    fn solve_global(&self, q_init: &DVector<f64>, q_goal: &DVector<f64>, local_search: &LocalSearchBox, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, terminate: &mut TerminationUtilOption) -> Result<PathPlannerResult, String> {
        return self._surge_global.solve_global(q_init, q_goal, local_search, lynx_vars, recorder, terminate);
    }
    fn name_global(&self) -> String { return "SprintGlobal".to_string(); }
}
impl LynxVarsUser for SprintGlobal { }

