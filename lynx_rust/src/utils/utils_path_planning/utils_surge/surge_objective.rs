use crate::utils::utils_path_planning::utils_surge::surge_objective_term::*;
use crate::utils::utils_path_planning::utils_surge::surge_connection_info::SurgeConnectionInfo;
use crate::utils::utils_path_planning::utils_surge::surge_points_manager::SurgePointsManager;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use termion::{color, style};
use std::time::Instant;
use nalgebra::{DVector};

#[derive(Clone)]
pub struct SurgeObjective {
    _surge_objective_term_box: SurgeObjectiveTermBox,
    _last_update_instant: Instant
}

impl SurgeObjective {
    pub fn new(surge_objective_term: &dyn SurgeObjectiveTerm) -> Self {
        let _surge_objective_term_box = SurgeObjectiveTermBox::new(surge_objective_term);
        return Self::new_from_surge_objective_term_box(_surge_objective_term_box);
    }

    pub fn new_from_surge_objective_term_box(surge_objective_term_box: SurgeObjectiveTermBox) -> Self {
        let _last_update_instant = Instant::now();
        return Self { _surge_objective_term_box: surge_objective_term_box, _last_update_instant };
    }

    pub fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        return self._surge_objective_term_box.call(surge_objective_idx, surge_connection_info, surge_points_manager, lynx_vars, recorder);
    }

    pub fn initialize_with_surge_points_manager(&mut self, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        return self._surge_objective_term_box.initialize_with_surge_points_manager(surge_points_manager, lynx_vars, recorder);
    }

    pub fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        let update_res = self._surge_objective_term_box.update_after_connection_attempt(point1_idx, point2_idx, successful_connection, surge_points_manager, lynx_vars, recorder)?;
        if update_res { self._last_update_instant = Instant::now(); }
        Ok(())
    }

    pub fn lower_is_better(&self) -> bool {
        return self._surge_objective_term_box.lower_is_better();
    }

    pub fn get_name(&self) -> String {
        return self._surge_objective_term_box.get_name();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_last_update_instant(&self) -> &Instant {
        return &self._last_update_instant;
    }

    pub fn print_summary(&self) {
        println!("{}surge objective term {:?}, last update instant {:?}{}", color::Fg(color::Blue), self._surge_objective_term_box.get_name(), self._last_update_instant, style::Reset);
    }
}