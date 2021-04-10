use crate::utils::utils_path_planning::utils_surge::surge_connection_info::SurgeConnectionInfo;
use crate::utils::utils_path_planning::utils_surge::surge_objective_manager::SurgeObjectiveManager;
use crate::utils::utils_path_planning::utils_surge::surge_points_manager::SurgePointsManager;
use crate::utils::utils_vars::lynx_vars_generic::LynxVarsGeneric;
use crate::utils::utils_recorders::prelude::*;
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct SurgeConnectionManager {
    _connection_infos: Vec<SurgeConnectionInfo>,
    _connection_hashmap: HashMap<(usize, usize), usize>
}

impl SurgeConnectionManager {
    pub fn new_empty() -> Self {
        let _connection_infos = Vec::new();
        let _connection_hashmap = HashMap::new();

        return Self { _connection_infos, _connection_hashmap };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_connection_mut_ref(&mut self, idx1: usize, idx2: usize, surge_objective_manager: &mut SurgeObjectiveManager, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<&mut SurgeConnectionInfo, String> {
        let res = self._get_connection_idx(idx1, idx2);
        if res.is_some() {
            return Ok(&mut self._connection_infos[res.unwrap()]);
        } else {
            let mut new_connection_info = SurgeConnectionInfo::new(idx1, idx2, surge_objective_manager, surge_points_manager, lynx_vars, recorder)?;
            self._connection_infos.push(new_connection_info);
            let add_idx = self._connection_infos.len() - 1;
            self._connection_hashmap.insert( (idx1, idx2), add_idx );

            return Ok(self._get_connection_mut_ref(idx1, idx2).unwrap());
        }
    }

    pub fn get_connection_ref(&mut self, idx1: usize, idx2: usize, surge_objective_manager: &mut SurgeObjectiveManager, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<&SurgeConnectionInfo, String> {
        let res = self._get_connection_idx(idx1, idx2);
        if res.is_some() {
            return Ok(&self._connection_infos[res.unwrap()]);
        } else {
            let mut new_connection_info = SurgeConnectionInfo::new(idx1, idx2, surge_objective_manager, surge_points_manager, lynx_vars, recorder)?;
            self._connection_infos.push(new_connection_info);
            let add_idx = self._connection_infos.len() - 1;
            self._connection_hashmap.insert( (idx1, idx2), add_idx );

            return Ok(self._get_connection_ref(idx1, idx2).unwrap());
        }
    }

    fn _get_connection_idx(&self, idx1: usize, idx2: usize) -> Option<usize> {
        let res = self._connection_hashmap.get(&(idx1, idx2));
        if res.is_some() {
            return Some(*res.unwrap());
        } else {
            return None;
        }
    }

    fn _get_connection_mut_ref(&mut self, idx1: usize, idx2: usize) -> Option<&mut SurgeConnectionInfo> {
        let res = self._connection_hashmap.get(&(idx1, idx2));
        if res.is_none() { return None }

        return Some(&mut self._connection_infos[*res.unwrap()]);
    }

    fn _get_connection_ref(&self, idx1: usize, idx2: usize) -> Option<&SurgeConnectionInfo> {
        let res = self._connection_hashmap.get(&(idx1, idx2));
        if res.is_none() { return None; }

        return Some(&self._connection_infos[*res.unwrap()]);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn is_dead_connection(&self, idx1: usize, idx2: usize) -> Result<bool, String> {
        let connection_info = self._get_connection_ref(idx1, idx2);
        if connection_info.is_some() {
            return Ok(connection_info.as_ref().unwrap().is_dead_connection());
        } else {
            return Err("connection not found".to_string());
        }
    }

    pub fn set_dead_connection(&mut self, idx1: usize, idx2: usize) -> Result<(), String> {
        let mut connection_info = self._get_connection_mut_ref(idx1, idx2);
        if connection_info.is_some() {
            connection_info.as_mut().unwrap().set_as_dead_connection();
            Ok(())
        } else {
            return Err("connection not found".to_string());
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print(&self) {
        println!("{:?}", self);
    }
}