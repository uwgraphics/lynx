use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use crate::utils::utils_path_planning::utils_surge::surge_points_manager::SurgePointsManager;
use crate::utils::utils_path_planning::utils_surge::surge_connection_info::SurgeConnectionInfo;
use crate::utils::utils_math::common_functions::*;
use nalgebra::{DVector};

pub trait SurgeObjectiveTerm : SurgeObjectiveTermClone + LynxVarsUser + AsLynxVarsUser {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String>;
    fn initialize_with_surge_points_manager(&mut self, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> { return Ok(()); }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String>;
    fn get_name(&self) -> String;
    fn to_surge_objective_term_box(&self) -> SurgeObjectiveTermBox {
        return SurgeObjectiveTermBox{_surge_objective_term_box: self.clone_box()};
    }
    fn lower_is_better(&self) -> bool;
}

pub trait SurgeObjectiveTermClone {
    fn clone_box(&self) -> Box<dyn SurgeObjectiveTerm>;
}
impl<T> SurgeObjectiveTermClone for T where T: 'static + SurgeObjectiveTerm + Clone {
    fn clone_box(&self) -> Box<dyn SurgeObjectiveTerm> {
        Box::new(self.clone())
    }
}

pub struct SurgeObjectiveTermBox {
    _surge_objective_term_box: Box<dyn SurgeObjectiveTerm>
}

impl SurgeObjectiveTermBox {
    pub fn new(surge_objective_term: &dyn SurgeObjectiveTerm) -> Self {
        let _surge_objective_term_box = surge_objective_term.clone_box();
        Self { _surge_objective_term_box }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        return self._surge_objective_term_box.call(surge_objective_idx, surge_connection_info, surge_points_manager, lynx_vars, recorder);
    }
    pub fn initialize_with_surge_points_manager(&mut self, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        return self._surge_objective_term_box.initialize_with_surge_points_manager(surge_points_manager, lynx_vars, recorder);
    }
    pub fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        return self._surge_objective_term_box.update_after_connection_attempt(point1_idx, point2_idx, successful_connection, surge_points_manager, lynx_vars, recorder);
    }
    pub fn get_name(&self) -> String {
        return self._surge_objective_term_box.get_name();
    }
    pub fn lower_is_better(&self) -> bool { return self._surge_objective_term_box.lower_is_better(); }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_name(&self) {
        println!("{:?}", self.get_name());
    }
}
impl Clone for SurgeObjectiveTermBox {
    fn clone(&self) -> Self {
        let b = self._surge_objective_term_box.clone_box();
        return Self { _surge_objective_term_box: b };
    }
}
unsafe impl Send for SurgeObjectiveTermBox { }
unsafe impl Sync for SurgeObjectiveTermBox { }

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct SurgeObjectivePairwiseDis;
impl SurgeObjectiveTerm for SurgeObjectivePairwiseDis {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        let point1 = surge_points_manager.get_point_ref(surge_connection_info.get_idx1())?;
        let point2 = surge_points_manager.get_point_ref(surge_connection_info.get_idx2())?;
        let dis = (point1 - point2).norm();
        return Ok(dis);
    }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        return Ok(false);
    }
    fn get_name(&self) -> String { return "PairwiseDistance".to_string(); }
    fn lower_is_better(&self) -> bool { return true; }
}
impl LynxVarsUser for SurgeObjectivePairwiseDis { }

#[derive(Clone)]
pub struct SurgeObjectiveDisToEndStateSet {
    _goal_set_length: usize
}
impl SurgeObjectiveDisToEndStateSet {
    pub fn new_empty() -> Self {
        Self { _goal_set_length: usize::max_value() }
    }
}
impl SurgeObjectiveTerm for SurgeObjectiveDisToEndStateSet {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        let point2 = surge_points_manager.get_point_ref(surge_connection_info.get_idx2())?;

        let mut min_dis = std::f64::INFINITY;
        let end_state_idxs = surge_points_manager.get_end_state_idxs_ref();
        for e in end_state_idxs {
            let end_state = surge_points_manager.get_point_ref(*e)?;
            let dis = (point2 - end_state).norm();
            if dis < min_dis {
                min_dis = dis;
                if min_dis == 0.0 {
                    return Ok(min_dis);
                }
            }
        }
        self._goal_set_length = end_state_idxs.len();

        return Ok(min_dis);
    }
    fn initialize_with_surge_points_manager(&mut self, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        self._goal_set_length = surge_points_manager.get_end_state_idxs_ref().len();
        Ok(())
    }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        let end_state_idxs = surge_points_manager.get_end_state_idxs_ref();
        if end_state_idxs.len() == self._goal_set_length {
            return Ok(false);
        } else {
            self._goal_set_length = end_state_idxs.len();
            return Ok(true);
        }
    }
    fn get_name(&self) -> String {
        return "DisToEndStateSet".to_string();
    }
    fn lower_is_better(&self) -> bool { return true; }
}
impl LynxVarsUser for SurgeObjectiveDisToEndStateSet { }

#[derive(Clone)]
pub struct SurgeObjectiveDisToEndStateSet2 {
    _goal_set_length: usize
}
impl SurgeObjectiveDisToEndStateSet2 {
    pub fn new_empty() -> Self {
        Self { _goal_set_length: usize::max_value() }
    }
}
impl SurgeObjectiveTerm for SurgeObjectiveDisToEndStateSet2 {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        let idx2 = surge_connection_info.get_idx2();

        let is_region_endpoint_an_end_dag_node = surge_points_manager.is_end_dag_node_idx(idx2)?;

        let point2 = surge_points_manager.get_point_ref(idx2)?;

        let mut pertinent_target_dag_node_idxs = &vec![];

        if is_region_endpoint_an_end_dag_node { pertinent_target_dag_node_idxs = surge_points_manager.get_end_dag_node_idxs_ref(); }
        else { pertinent_target_dag_node_idxs = surge_points_manager.get_start_dag_node_idxs_ref(); }

        let float_cache_length = surge_connection_info.get_float_cache_ref(surge_objective_idx)?.len();
        if float_cache_length == 0 {
            surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?.push(std::f64::INFINITY);
            surge_connection_info.get_usize_cache_mut_ref(surge_objective_idx)?.push(usize::max_value());
        }

        let mut min_dis = surge_connection_info.get_float_cache_ref(surge_objective_idx)?[0];
        let mut curr_bookmark = surge_connection_info.get_usize_cache_ref(surge_objective_idx)?[0];
        let mut updated_minimum = false;

        let l = pertinent_target_dag_node_idxs.len();
        for i in 0..l {
            if min_dis.is_infinite() || i > curr_bookmark {
                let curr_idx = pertinent_target_dag_node_idxs[i];
                let curr_point = surge_points_manager.get_point_ref(curr_idx)?;
                let curr_dis = (point2 - curr_point).norm();
                if curr_dis < min_dis {
                    min_dis = curr_dis;
                    updated_minimum = true;
                }
                curr_bookmark = i;
            }
        }

        surge_connection_info.get_usize_cache_mut_ref(surge_objective_idx)?[0] = curr_bookmark;
        return Ok(min_dis);

        /*
        let mut min_dis = std::f64::INFINITY;
        let end_state_idxs = surge_points_manager.get_end_state_idxs_ref();
        for e in end_state_idxs {
            let end_state = surge_points_manager.get_point_ref(*e)?;
            let dis = (point2 - end_state).norm();
            if dis < min_dis {
                min_dis = dis;
                if min_dis == 0.0 {
                    return Ok(min_dis);
                }
            }
        }
        self._goal_set_length = end_state_idxs.len();

        return Ok(min_dis);
         */
    }
    fn initialize_with_surge_points_manager(&mut self, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        self._goal_set_length = surge_points_manager.get_end_state_idxs_ref().len();
        Ok(())
    }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        let end_state_idxs = surge_points_manager.get_end_state_idxs_ref();
        if end_state_idxs.len() == self._goal_set_length {
            return Ok(false);
        } else {
            self._goal_set_length = end_state_idxs.len();
            return Ok(true);
        }
    }
    fn get_name(&self) -> String {
        return "DisToEndStateSet".to_string();
    }
    fn lower_is_better(&self) -> bool { return true; }
}
impl LynxVarsUser for SurgeObjectiveDisToEndStateSet2 { }

#[derive(Clone)]
pub struct SurgeObjectiveCloserToSingleEndStateRatio;
impl SurgeObjectiveTerm for SurgeObjectiveCloserToSingleEndStateRatio {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        let end_state_idxs_ref = surge_points_manager.get_end_state_idxs_ref();
        if end_state_idxs_ref.len() == 0 { return Err("at least one end state is required in SurgeObjectiveCloserToFirstGoalRatio".to_string()); }

        let end_state_ref = surge_points_manager.get_point_ref(end_state_idxs_ref[0])?;
        let point1_ref = surge_points_manager.get_point_ref(surge_connection_info.get_idx1())?;
        let point2_ref = surge_points_manager.get_point_ref(surge_connection_info.get_idx2())?;

        let ratio = (point2_ref - end_state_ref).norm() / (point1_ref - end_state_ref).norm();

        let c = 1.0;
        let g = (-ratio.powi(2) / (2.0 * (c * c)) ).exp();

        return Ok(g);
    }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        return Ok(false);
    }
    fn get_name(&self) -> String {
        return "CloserToFirstGoalRatio".to_string();
    }
    fn lower_is_better(&self) -> bool { return false; }
}
impl LynxVarsUser for SurgeObjectiveCloserToSingleEndStateRatio { }

#[derive(Clone)]
pub struct SurgeObjectiveLocalMinAvoid {
    _local_min_region_idxs: Vec<(usize, usize)>
}
impl SurgeObjectiveLocalMinAvoid {
    pub fn new_empty() -> Self {
        return Self { _local_min_region_idxs: Vec::new() }
    }
}
impl SurgeObjectiveTerm for SurgeObjectiveLocalMinAvoid {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        let point1 = surge_points_manager.get_point_ref(surge_connection_info.get_idx1())?;
        let point2 = surge_points_manager.get_point_ref(surge_connection_info.get_idx2())?;
        let start_state_idxs = surge_points_manager.get_start_state_idxs_ref();
        let end_state_idxs = surge_points_manager.get_end_state_idxs_ref();
        let start_state = surge_points_manager.get_point_ref(start_state_idxs[0])?;
        let end_state = surge_points_manager.get_point_ref(end_state_idxs[0])?;

        let mut float_cache = surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?;

        let mut x: f64 = 1.0;

        let l = self._local_min_region_idxs.len();
        for i in 0..l {
            if i >= float_cache.len() {
                let q_a = surge_points_manager.get_point_ref(self._local_min_region_idxs[i].0)?;
                let q_b = surge_points_manager.get_point_ref(self._local_min_region_idxs[i].1)?;
                let p1 = proj_plus(point1, q_b, q_a);
                let p2 = proj_plus(point2, q_a, q_b);
                let d1 = (&p1 - point1).norm();
                let d2 = (&p2 - point2).norm();
                let v = (d1 + d2) / ( 0.5 * (start_state - end_state).norm() );
                if x.is_infinite() && v == 0.0 {
                    x = 0.0;
                } else {
                    x *= v;
                }
                float_cache.push( v );
            } else {
                if x.is_infinite() && float_cache[i] == 0.0 {
                    x = 0.0;
                } else {
                    x *= float_cache[i];
                }
            }
        }

        let c = 0.25;
        let g = -(-x.powi(2) / (2.0 * (c * c)) ).exp() + 1.0;

        Ok(g)
    }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        if !successful_connection {
            self._local_min_region_idxs.push((point1_idx, point2_idx));
            return Ok(true);
        } else {
            return Ok(false);
        }
    }
    fn get_name(&self) -> String {
        return "LocalMinAvoid".to_string();
    }
    fn lower_is_better(&self) -> bool { return false; }
}
impl LynxVarsUser for SurgeObjectiveLocalMinAvoid { }

#[derive(Clone)]
pub struct SurgeObjectiveLocalMinAvoid2 {
    _local_min_region_idxs: Vec<(usize, usize)>
}
impl SurgeObjectiveLocalMinAvoid2 {
    pub fn new_empty() -> Self {
        return Self { _local_min_region_idxs: Vec::new() }
    }
}
impl SurgeObjectiveTerm for SurgeObjectiveLocalMinAvoid2 {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        if self._local_min_region_idxs.len() == 0 { return Ok(1.0); }

        let point1 = surge_points_manager.get_point_ref(surge_connection_info.get_idx1())?;
        let point2 = surge_points_manager.get_point_ref(surge_connection_info.get_idx2())?;
        let start_state_idxs = surge_points_manager.get_start_state_idxs_ref();
        let end_state_idxs = surge_points_manager.get_end_state_idxs_ref();
        let start_state = surge_points_manager.get_point_ref(start_state_idxs[0])?;
        let end_state = surge_points_manager.get_point_ref(end_state_idxs[0])?;

        let float_cache_length = surge_connection_info.get_float_cache_ref(surge_objective_idx)?.len();
        if float_cache_length == 0 {
            surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?.push(1.0);
            surge_connection_info.get_usize_cache_mut_ref(surge_objective_idx)?.push(usize::max_value());
        }

        let mut curr_x_val = surge_connection_info.get_float_cache_ref(surge_objective_idx)?[0];
        let mut curr_bookmark = surge_connection_info.get_usize_cache_ref(surge_objective_idx)?[0];

        let l = self._local_min_region_idxs.len();
        for i in 0..l {
            if curr_bookmark == usize::max_value() || i > curr_bookmark {
                let q_a = surge_points_manager.get_point_ref(self._local_min_region_idxs[i].0)?;
                let q_b = surge_points_manager.get_point_ref(self._local_min_region_idxs[i].1)?;
                let p1 = proj_plus(point1, q_b, q_a);
                let p2 = proj_plus(point2, q_a, q_b);
                let d1 = (&p1 - point1).norm();
                let d2 = (&p2 - point2).norm();
                let v = (d1 + d2) / ( 1.0 * (start_state - end_state).norm() );
                if curr_x_val.is_infinite() && v == 0.0 {
                    curr_x_val = 0.0;
                } else {
                    curr_x_val *= v;
                }
                curr_bookmark = i;
            }
        }

        surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?[0] = curr_x_val;
        surge_connection_info.get_usize_cache_mut_ref(surge_objective_idx)?[0] = curr_bookmark;

        let c = 0.25;
        let g = -(-curr_x_val.powi(2) / (2.0 * (c * c)) ).exp() + 1.0;

        Ok(g)
    }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        if !successful_connection {
            self._local_min_region_idxs.push((point1_idx, point2_idx));
            return Ok(true);
        } else {
            return Ok(false);
        }
    }
    fn get_name(&self) -> String {
        return "LocalMinAvoid".to_string();
    }
    fn lower_is_better(&self) -> bool { return false; }
}
impl LynxVarsUser for SurgeObjectiveLocalMinAvoid2 { }

#[derive(Clone)]
pub struct SurgeObjectiveCloserToTargetDag {
    _num_start_dag_nodes: usize,
    _num_end_dag_nodes: usize
}
impl SurgeObjectiveCloserToTargetDag {
    pub fn new_empty() -> Self {
        return Self { _num_start_dag_nodes: 0, _num_end_dag_nodes: 0 }
    }
}
impl SurgeObjectiveTerm for SurgeObjectiveCloserToTargetDag {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        let idx1 = surge_connection_info.get_idx1();
        let idx2 = surge_connection_info.get_idx2();

        if surge_points_manager.is_start_state_idx(idx2)? { return Ok(1.0); }
        if surge_points_manager.is_end_state_idx(idx2)? { return Ok(1.0); }

        let is_region_endpoint_an_end_dag_node = surge_points_manager.is_end_dag_node_idx(idx2)?;

        let point1 = surge_points_manager.get_point_ref(idx1)?;
        let point2 = surge_points_manager.get_point_ref(idx2)?;

        let float_cache_mut_ref = surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?;

        let mut pertinent_target_dag_node_idxs = &vec![];
        if is_region_endpoint_an_end_dag_node { pertinent_target_dag_node_idxs = surge_points_manager.get_end_dag_node_idxs_ref(); }
        else { pertinent_target_dag_node_idxs = surge_points_manager.get_start_dag_node_idxs_ref(); }

        let mut min_dis = std::f64::INFINITY;
        let mut min_dis_endpoint = point2; // just a placeholder

        let l = pertinent_target_dag_node_idxs.len();
        for i in 0..l {
            if i < float_cache_mut_ref.len() {
                let curr_dis = float_cache_mut_ref[i];
                if curr_dis < min_dis {
                    min_dis = curr_dis;
                    min_dis_endpoint = surge_points_manager.get_point_ref(pertinent_target_dag_node_idxs[i])?;
                }
            } else {
                let curr_idx = pertinent_target_dag_node_idxs[i];
                let curr_point = surge_points_manager.get_point_ref(curr_idx)?;
                let curr_dis = (point2 - curr_point).norm();
                if curr_dis < min_dis { min_dis = curr_dis; }
                min_dis_endpoint = curr_point;
                float_cache_mut_ref.push(curr_dis);
            }
        }

        let ratio = (point2 - min_dis_endpoint).norm() / (point1 - min_dis_endpoint).norm();
        let c = 1.0;
        let g = (-ratio.powi(2) / (2.0 * (c * c)) ).exp();

        return Ok(g);
    }
    fn initialize_with_surge_points_manager(&mut self, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        self._num_start_dag_nodes = surge_points_manager.get_start_dag_node_idxs_ref().len();
        self._num_end_dag_nodes = surge_points_manager.get_end_dag_node_idxs_ref().len();
        Ok(())
    }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        let num_start_dag_nodes = surge_points_manager.get_start_dag_node_idxs_ref().len();
        let num_end_dag_nodes = surge_points_manager.get_end_dag_node_idxs_ref().len();
        return if num_start_dag_nodes == self._num_start_dag_nodes && num_end_dag_nodes == self._num_end_dag_nodes {
            Ok(false)
        } else {
            self._num_start_dag_nodes = num_start_dag_nodes;
            self._num_end_dag_nodes = num_end_dag_nodes;
            Ok(true)
        }
    }
    fn get_name(&self) -> String {
        return "CloserToTargetDag".to_string();
    }
    fn lower_is_better(&self) -> bool { return false; }
}
impl LynxVarsUser for SurgeObjectiveCloserToTargetDag { }

#[derive(Clone)]
pub struct SurgeObjectiveCloserToTargetDag2 {
    _num_start_dag_nodes: usize,
    _num_end_dag_nodes: usize
}
impl SurgeObjectiveCloserToTargetDag2 {
    pub fn new_empty() -> Self {
        return Self { _num_start_dag_nodes: 0, _num_end_dag_nodes: 0 }
    }
}
impl SurgeObjectiveTerm for SurgeObjectiveCloserToTargetDag2 {
    fn call(&mut self, surge_objective_idx: usize, surge_connection_info: &mut SurgeConnectionInfo, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        let idx1 = surge_connection_info.get_idx1();
        let idx2 = surge_connection_info.get_idx2();

        let is_region_endpoint_an_end_dag_node = surge_points_manager.is_end_dag_node_idx(idx2)?;

        let point1 = surge_points_manager.get_point_ref(idx1)?;
        let point2 = surge_points_manager.get_point_ref(idx2)?;

        let mut pertinent_target_dag_node_idxs = &vec![];

        if is_region_endpoint_an_end_dag_node { pertinent_target_dag_node_idxs = surge_points_manager.get_start_dag_node_idxs_ref(); }
        else { pertinent_target_dag_node_idxs = surge_points_manager.get_end_dag_node_idxs_ref(); }

        let float_cache_length = surge_connection_info.get_float_cache_ref(surge_objective_idx)?.len();
        if float_cache_length == 0 {
            surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?.push(std::f64::INFINITY);
            surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?.push(std::f64::INFINITY);
            surge_connection_info.get_usize_cache_mut_ref(surge_objective_idx)?.push(usize::max_value());
            surge_connection_info.get_usize_cache_mut_ref(surge_objective_idx)?.push(usize::max_value());
        }

        let mut min_dis = surge_connection_info.get_float_cache_ref(surge_objective_idx)?[0];
        let mut previous_return_value = surge_connection_info.get_float_cache_ref(surge_objective_idx)?[1];
        let mut curr_bookmark = surge_connection_info.get_usize_cache_ref(surge_objective_idx)?[0];
        let mut min_point_idx = surge_connection_info.get_usize_cache_ref(surge_objective_idx)?[1];
        let mut updated_minimum = false;

        let l = pertinent_target_dag_node_idxs.len();
        for i in 0..l {
            if min_dis.is_infinite() || i > curr_bookmark {
                let curr_idx = pertinent_target_dag_node_idxs[i];
                let curr_point = surge_points_manager.get_point_ref(curr_idx)?;
                let curr_dis = (point2 - curr_point).norm();
                if curr_dis < min_dis {
                    min_dis = curr_dis;
                    min_point_idx = pertinent_target_dag_node_idxs[i];
                    updated_minimum = true;
                }
                curr_bookmark = i;
            }
        }

        surge_connection_info.get_usize_cache_mut_ref(surge_objective_idx)?[0] = curr_bookmark;

        if updated_minimum {
            surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?[0] = min_dis;
            surge_connection_info.get_usize_cache_mut_ref(surge_objective_idx)?[1] = min_point_idx;
            let min_dis_endpoint = surge_points_manager.get_point_ref(min_point_idx)?;

            let ratio = (point2 - min_dis_endpoint).norm() / (point1 - min_dis_endpoint).norm();
            let c = 1.0;
            let g = (-ratio.powi(2) / (2.0 * (c * c)) ).exp();
            surge_connection_info.get_float_cache_mut_ref(surge_objective_idx)?[1] = g;

            return Ok(g);
        } else {
            return Ok(previous_return_value);
        }
    }
    fn initialize_with_surge_points_manager(&mut self, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        self._num_start_dag_nodes = surge_points_manager.get_start_dag_node_idxs_ref().len();
        self._num_end_dag_nodes = surge_points_manager.get_end_dag_node_idxs_ref().len();
        Ok(())
    }
    fn update_after_connection_attempt(&mut self, point1_idx: usize, point2_idx: usize, successful_connection: bool, surge_points_manager: &SurgePointsManager, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<bool, String> {
        let num_start_dag_nodes = surge_points_manager.get_start_dag_node_idxs_ref().len();
        let num_end_dag_nodes = surge_points_manager.get_end_dag_node_idxs_ref().len();
        return if num_start_dag_nodes == self._num_start_dag_nodes && num_end_dag_nodes == self._num_end_dag_nodes {
            Ok(false)
        } else {
            self._num_start_dag_nodes = num_start_dag_nodes;
            self._num_end_dag_nodes = num_end_dag_nodes;
            Ok(true)
        }
    }
    fn get_name(&self) -> String {
        return "CloserToTargetDag".to_string();
    }
    fn lower_is_better(&self) -> bool { return false; }
}
impl LynxVarsUser for SurgeObjectiveCloserToTargetDag2 { }


