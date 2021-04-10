use crate::utils::utils_path_planning::utils_surge::surge_objective::SurgeObjective;
use crate::utils::utils_path_planning::utils_surge::surge_connection_info::SurgeConnectionInfo;
use crate::utils::utils_path_planning::utils_surge::surge_objective_term::*;
use crate::utils::utils_path_planning::utils_surge::surge_points_manager::SurgePointsManager;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use termion::{color, style};
use std::time::Instant;
use nalgebra::{DVector};
use std::cmp::Ordering;

#[derive(Clone)]
pub struct SurgeObjectiveManager {
    _surge_objectives: Vec<SurgeObjective>,
    _weights: Vec<f64>,
    _num_objectives: usize
}

impl SurgeObjectiveManager {
    pub fn new_empty() -> Self {
        let _surge_objectives = Vec::new();
        let _weights = Vec::new();
        let _num_objectives = 0;
        return Self { _surge_objectives, _weights, _num_objectives };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_surge_objective_term(&mut self, surge_objective_term: &dyn SurgeObjectiveTerm, weight: Option<f64>) {
        self._surge_objectives.push( SurgeObjective::new(surge_objective_term) );
        if weight.is_none() {
            self._weights.push(1.0);
        } else {
            self._weights.push( weight.unwrap() );
        }
        self._num_objectives += 1;
    }

    pub fn add_surge_objective_term_box(&mut self, surge_objective_term_box: SurgeObjectiveTermBox, weight: Option<f64>) {
        self._surge_objectives.push( SurgeObjective::new_from_surge_objective_term_box(surge_objective_term_box) );
        if weight.is_none() {
            self._weights.push(1.0);
        } else {
            self._weights.push( weight.unwrap() );
        }
        self._num_objectives += 1;
    }

    pub fn initialize_all_with_surge_points_manager(&mut self, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        let l = self._surge_objectives.len();
        for i in 0..l {
            self._surge_objectives[i].initialize_with_surge_points_manager(surge_points_manager, lynx_vars, recorder)?;
        }

        Ok(())
    }

    pub fn update_all_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        let l = self._surge_objectives.len();
        for i in 0..l {
            self._surge_objectives[i].update_after_connection_attempt(point1_idx, point2_idx, successful_connection, surge_points_manager, lynx_vars, recorder)?;
        }

        Ok(())
    }

    pub fn call_surge_objective(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        if surge_objective_idx >= self._surge_objectives.len() {
            return Err(format!("surge objective idx {:?} is too high for number of objectives ({:?})", surge_objective_idx, self._surge_objectives.len()));
        }

        let res = self._surge_objectives[surge_objective_idx].call(surge_objective_idx, surge_connection_info, surge_points_manager, lynx_vars, recorder)?;
        return Ok(self._weights[surge_objective_idx] * res);
    }

    pub fn get_last_update_instant_on_one_objective(&self, surge_objective_idx: usize) -> Result<&Instant, String> {
        if surge_objective_idx >= self._surge_objectives.len() {
            return Err(format!("surge objective idx {:?} is too high for number of objectives ({:?})", surge_objective_idx, self._surge_objectives.len()));
        }

        return Ok( self._surge_objectives[surge_objective_idx].get_last_update_instant() );
    }

    pub fn get_lower_is_better(&self) -> Result<bool, String> {
        let mut lower_is_better = true;
        let l = self._surge_objectives.len();
        if l == 0 { return Err("cannot decide on lower_is_better in surge_objective_manager with 0 objectives.".to_string()); }
        for i in 0..l {
            if i == 0 {
                lower_is_better = self._surge_objectives[i].lower_is_better();
            }
            else {
                let peq_check = self._surge_objectives[i].lower_is_better().partial_cmp(&lower_is_better).unwrap();
                match peq_check {
                    Ordering::Less => { return Err("incompatible objectives, some were lower is better and some were higher is better.".to_string()); }
                    Ordering::Equal => {}
                    Ordering::Greater => { return Err("incompatible objectives, some were lower is better and some were higher is better.".to_string()); }
                }
            }
        }
        return Ok(lower_is_better);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_num_surge_objectives(&self) -> usize {
        return self._num_objectives;
    }

    pub fn print_summary(&self) {
        let l = self._surge_objectives.len();
        for i in 0..l {
            print!("{:?} ---> ", i);
            self._surge_objectives[i].print_summary();
        }
    }
}

