use std::time::Instant;
use crate::utils::utils_path_planning::utils_surge::surge_objective_manager::SurgeObjectiveManager;
use crate::utils::utils_path_planning::utils_surge::surge_points_manager::SurgePointsManager;
use crate::utils::utils_vars::lynx_vars_generic::LynxVarsGeneric;
use crate::utils::utils_recorders::prelude::*;


#[derive(Clone, Debug)]
pub struct SurgeConnectionInfo {
    _idx1: usize,
    _idx2: usize,
    _objective_values: Vec<f64>,
    _objective_last_update_instants: Vec<Instant>,
    _float_cache: Vec<Vec<f64>>,
    _usize_cache: Vec<Vec<usize>>,
    _int_cache: Vec<Vec<i32>>,
    _dead_connection: bool
}

impl SurgeConnectionInfo {
    pub fn new_empty(idx1: usize, idx2: usize) -> Self {
        let _objective_values = Vec::new();
        let _objective_value_timestamps = Vec::new();
        let _float_cache = Vec::new();
        let _usize_cache = Vec::new();
        let _int_cache = Vec::new();
        let _dead_connection = false;

        return Self { _idx1: idx1, _idx2: idx2, _objective_values, _objective_last_update_instants: _objective_value_timestamps, _float_cache, _usize_cache,_int_cache, _dead_connection }
    }

    pub fn new(idx1: usize, idx2: usize, surge_objective_manager: &mut SurgeObjectiveManager, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Self, String> {
        let mut out_self = Self::new_empty(idx1, idx2);
        out_self._update_objective_values_and_timestamps(surge_objective_manager, surge_points_manager, lynx_vars, recorder)?;
        return Ok(out_self);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_total_objective_score(&mut self, surge_objective_manager: &mut SurgeObjectiveManager, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        self._update_objective_values_and_timestamps(surge_objective_manager, surge_points_manager, lynx_vars, recorder)?;

        let mut out_sum = 0.0;
        let l = self._objective_values.len();
        for i in 0..l {
            out_sum += self._objective_values[i];
        }

        Ok(out_sum)
    }

    fn _update_objective_values_and_timestamps(&mut self, surge_objective_manager: &mut SurgeObjectiveManager, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        let l = surge_objective_manager.get_num_surge_objectives();
        for i in 0..l {
            if i >= self._objective_values.len() {
                self._objective_values.push(0.0);
                self._float_cache.push(Vec::new());
                self._usize_cache.push(Vec::new());
                self._int_cache.push(Vec::new());
            }
            if i >= self._objective_last_update_instants.len() {
                self._objective_last_update_instants.push( Instant::now() );
            }

            let last_update_instant = surge_objective_manager.get_last_update_instant_on_one_objective(i)?.clone();
            if !(&last_update_instant == &self._objective_last_update_instants[i]) {
                let objective_value = surge_objective_manager.call_surge_objective(i, self, surge_points_manager, lynx_vars, recorder)?;
                self._objective_values[i] = objective_value;
                self._objective_last_update_instants[i] = last_update_instant.clone();
            }
        }

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn is_dead_connection(&self) -> bool {
        return self._dead_connection;
    }

    pub fn set_as_dead_connection(&mut self) {
        self._dead_connection = true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_idx1(&self) -> usize { return self._idx1; }

    pub fn get_idx2(&self) -> usize { return self._idx2; }

    pub fn get_objective_value(&self, surge_objective_idx: usize) -> Result<f64, String> {
        if surge_objective_idx >= self._objective_values.len() {
            return Err(format!("surge_objective_idx {:?} is too large for size of objective_values ({:?}) in SurgeConnectionInfo", surge_objective_idx, self._objective_values.len()));
        }

        return Ok(self._objective_values[surge_objective_idx]);
    }

    pub fn get_float_cache_ref(&self, surge_objective_idx: usize) -> Result<&Vec<f64>, String> {
        if surge_objective_idx >= self._float_cache.len() {
            return Err(format!("surge_objective_idx {:?} is too large for size of float_cache ({:?}) in SurgeConnectionInfo", surge_objective_idx, self._float_cache.len()));
        }
        return Ok(&self._float_cache[surge_objective_idx]);
    }

    pub fn get_float_cache_mut_ref(&mut self, surge_objective_idx: usize) -> Result<&mut Vec<f64>, String> {
        if surge_objective_idx >= self._float_cache.len() {
            return Err(format!("surge_objective_idx {:?} is too large for size of float_cache ({:?}) in SurgeConnectionInfo", surge_objective_idx, self._float_cache.len()));
        }
        return Ok(&mut self._float_cache[surge_objective_idx]);
    }

    pub fn get_usize_cache_ref(&self, surge_objective_idx: usize) -> Result<&Vec<usize>, String> {
        if surge_objective_idx >= self._usize_cache.len() {
            return Err(format!("surge_objective_idx {:?} is too large for size of usize_cache ({:?}) in SurgeConnectionInfo", surge_objective_idx, self._usize_cache.len()));
        }
        return Ok(&self._usize_cache[surge_objective_idx]);
    }

    pub fn get_usize_cache_mut_ref(&mut self, surge_objective_idx: usize) -> Result<&mut Vec<usize>, String> {
        if surge_objective_idx >= self._usize_cache.len() {
            return Err(format!("surge_objective_idx {:?} is too large for size of usize_cache ({:?}) in SurgeConnectionInfo", surge_objective_idx, self._usize_cache.len()));
        }
        return Ok(&mut self._usize_cache[surge_objective_idx]);
    }

    pub fn get_int_cache_ref(&self, surge_objective_idx: usize) -> Result<&Vec<i32>, String> {
        if surge_objective_idx >= self._float_cache.len() {
            return Err(format!("surge_objective_idx {:?} is too large for size of int_cache ({:?}) in SurgeConnectionInfo", surge_objective_idx, self._int_cache.len()));
        }
        return Ok(&self._int_cache[surge_objective_idx]);
    }

    pub fn get_int_cache_mut_ref(&mut self, surge_objective_idx: usize) -> Result<&mut Vec<i32>, String> {
        if surge_objective_idx >= self._int_cache.len() {
            return Err(format!("surge_objective_idx {:?} is too large for size of int_cache ({:?}) in SurgeConnectionInfo", surge_objective_idx, self._int_cache.len()));
        }
        return Ok(&mut self._int_cache[surge_objective_idx]);
    }
}