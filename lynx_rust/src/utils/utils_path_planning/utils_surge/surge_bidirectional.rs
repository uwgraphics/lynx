use crate::utils::utils_path_planning::utils_surge::{surge_connection_manager::*, surge_connection_info::*,
                                                     surge_objective_manager::*, surge_objective::*,
                                                     surge_objective_term::*, surge_points_manager::*,
                                                     planning_dag_and_surge_points_manager_idx_util::*};
use crate::utils::utils_path_planning::utils_planning_graphs::planning_dag::PlanningDAG;
use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_recorders::prelude::*;
use nalgebra::DVector;
use serde::{Serialize, Deserialize};

#[derive(Clone)]
pub struct SurgeBidirectional {
    _planning_dag: PlanningDAG,
    _surge_points_manager: SurgePointsManager,
    _planning_dag_and_surge_points_manager_idx_util: PlanningDAGAndSurgePointsManagerIdxUtil,
    _surge_connection_manager: SurgeConnectionManager,
    _surge_objective_manager: SurgeObjectiveManager,
}

impl SurgeBidirectional {
    pub fn new_empty() -> Self {
        let _planning_dag = PlanningDAG::new_empty();
        let _surge_points_manager = SurgePointsManager::new_empty();
        let _planning_dag_and_surge_points_manager_idx_util = PlanningDAGAndSurgePointsManagerIdxUtil::new_empty();
        let _surge_connection_manager = SurgeConnectionManager::new_empty();
        let _surge_objective_manager = SurgeObjectiveManager::new_empty();

        return Self { _planning_dag, _surge_points_manager, _planning_dag_and_surge_points_manager_idx_util,
            _surge_connection_manager, _surge_objective_manager };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_surge_objective_term(&mut self, surge_objective_term: &dyn SurgeObjectiveTerm, weight: Option<f64>) {
        self._surge_objective_manager.add_surge_objective_term(surge_objective_term, weight);
    }

    pub fn add_surge_objective_term_box(&mut self, surge_objective_term_box: SurgeObjectiveTermBox, weight: Option<f64>) {
        self._surge_objective_manager.add_surge_objective_term_box(surge_objective_term_box, weight);
    }

    pub fn add_start_state(&mut self, state: &DVector<f64>) {
        let surge_points_manager_idx = self._surge_points_manager.add_start_state(state);
        let planning_dag_idx = self._planning_dag.add_another_root_node(state);
        self._planning_dag_and_surge_points_manager_idx_util.add_idx_pairing(surge_points_manager_idx, planning_dag_idx);
    }

    pub fn add_end_state(&mut self, state: &DVector<f64>) {
        let surge_points_manager_idx = self._surge_points_manager.add_end_state(state);
        let planning_dag_idx = self._planning_dag.add_another_root_node(state);
        self._planning_dag_and_surge_points_manager_idx_util.add_idx_pairing(surge_points_manager_idx, planning_dag_idx);
    }

    pub fn add_milestone_state(&mut self, state: &DVector<f64>) {
        self._surge_points_manager.add_milestone_state(state);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_n_best_candidate_connections(&mut self, n: usize, lower_is_better: bool, unidirectional: bool, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        if unidirectional { return self._get_n_best_candidate_connections_unidirectional(n, lower_is_better, lynx_vars, recorder); }
        else { return self._get_n_best_candidate_connections_bidirectional(n, lower_is_better, lynx_vars, recorder); }
    }

    fn _get_n_best_candidate_connections_bidirectional(&mut self, n: usize, lower_is_better: bool, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        return if lower_is_better { self._get_n_best_candidate_connections_bidirectional_lower_is_better(n, lynx_vars, recorder) }
        else { self._get_n_best_candidate_connections_bidirectional_higher_is_better(n, lynx_vars, recorder) }
    }

    fn _get_n_best_candidate_connections_unidirectional(&mut self, n: usize, lower_is_better: bool, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        return if lower_is_better { self._get_n_best_candidate_connections_from_start_dag_nodes_lower_is_better(n, lynx_vars, recorder) }
        else { self._get_n_best_candidate_connections_from_start_dag_nodes_higher_is_better(n, lynx_vars, recorder) }
    }

    fn _get_n_best_candidate_connections_bidirectional_lower_is_better(&mut self, n: usize, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        if n == 0 {
            return Err("n cannot be 0 in get_best_n_candidate_connections".to_string());
        }

        let mut out_vec: Vec<SurgeCandidateConnection> = Vec::new();

        let best_from_start_dag_nodes = self._get_n_best_candidate_connections_from_start_dag_nodes_lower_is_better(n, lynx_vars, recorder)?;
        let best_from_end_dag_nodes = self._get_n_best_candidate_connections_from_end_dag_nodes_lower_is_better(n, lynx_vars, recorder)?;

        let mut start_dag_vec_idx = 0;
        let mut end_dag_vec_idx = 0;

        loop {
            if out_vec.len() == n {
                break;
            } else if start_dag_vec_idx >= best_from_start_dag_nodes.len() && end_dag_vec_idx >= best_from_end_dag_nodes.len() {
                break;
            } else if start_dag_vec_idx >= best_from_start_dag_nodes.len() {
                out_vec.push( best_from_end_dag_nodes[end_dag_vec_idx].clone() );
                end_dag_vec_idx += 1;
            } else if end_dag_vec_idx >= best_from_end_dag_nodes.len() {
                out_vec.push(best_from_start_dag_nodes[start_dag_vec_idx].clone() );
                start_dag_vec_idx += 1;
            } else {
                if best_from_start_dag_nodes[start_dag_vec_idx].connection_score < best_from_end_dag_nodes[end_dag_vec_idx].connection_score {
                    out_vec.push(best_from_start_dag_nodes[start_dag_vec_idx].clone());
                    start_dag_vec_idx += 1;
                } else {
                    out_vec.push(best_from_end_dag_nodes[end_dag_vec_idx].clone());
                    end_dag_vec_idx += 1;
                }
            }
        }

        Ok(out_vec)
    }
    fn _get_n_best_candidate_connections_from_start_dag_nodes_lower_is_better(&mut self, n: usize, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        if n == 0 {
            return Err("n cannot be 0 in get_best_n_candidate_connections".to_string());
        }

        let mut out_vec: Vec<SurgeCandidateConnection> = Vec::new();

        let start_dag_node_idxs = self._surge_points_manager.get_start_dag_node_idxs_ref();
        let target_state_idxs = self._surge_points_manager.get_start_dag_target_state_idxs_ref();

        for node in start_dag_node_idxs {
            for target in target_state_idxs {
                let mut connection_info_ref = self._surge_connection_manager.get_connection_mut_ref(*node, *target, &mut self._surge_objective_manager, &self._surge_points_manager, lynx_vars, recorder)?;
                if !(connection_info_ref.is_dead_connection()) {
                    let connection_score: f64 = connection_info_ref.get_total_objective_score(&mut self._surge_objective_manager, &self._surge_points_manager, lynx_vars, recorder)?;
                    if out_vec.len() < n || out_vec[n - 1].connection_score > connection_score {
                        let point1 = self._surge_points_manager.get_point_ref(*node)?.clone();
                        let point2 = self._surge_points_manager.get_point_ref(*target)?.clone();
                        let surge_candidate_connection = SurgeCandidateConnection::new(*node, *target, point1, point2, connection_score);

                        let binary_search_res = out_vec.binary_search_by(|x| x.connection_score.partial_cmp(&connection_score).unwrap());
                        match binary_search_res {
                            Ok(i) => out_vec.insert(i, surge_candidate_connection),
                            Err(i) => out_vec.insert(i, surge_candidate_connection)
                        }
                    }

                    if out_vec.len() > n {
                        out_vec.remove(n);
                    }
                }
            }
        }

        Ok(out_vec)
    }
    fn _get_n_best_candidate_connections_from_end_dag_nodes_lower_is_better(&mut self, n: usize, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        if n == 0 {
            return Err("n cannot be 0 in get_best_n_candidate_connections".to_string());
        }

        let mut out_vec: Vec<SurgeCandidateConnection> = Vec::new();

        let end_dag_node_idxs = self._surge_points_manager.get_end_dag_node_idxs_ref();
        let target_state_idxs = self._surge_points_manager.get_end_dag_target_state_idxs_ref();

        for node in end_dag_node_idxs {
            for target in target_state_idxs {
                let mut connection_info_ref = self._surge_connection_manager.get_connection_mut_ref(*node, *target, &mut self._surge_objective_manager, &self._surge_points_manager, lynx_vars, recorder)?;
                if !(connection_info_ref.is_dead_connection()) {
                    let connection_score: f64 = connection_info_ref.get_total_objective_score(&mut self._surge_objective_manager, &self._surge_points_manager, lynx_vars, recorder)?;
                    if out_vec.len() < n || out_vec[n - 1].connection_score > connection_score {
                        let point1 = self._surge_points_manager.get_point_ref(*node)?.clone();
                        let point2 = self._surge_points_manager.get_point_ref(*target)?.clone();
                        let surge_candidate_connection = SurgeCandidateConnection::new(*node, *target, point1, point2, connection_score);

                        let binary_search_res = out_vec.binary_search_by(|x| x.connection_score.partial_cmp(&connection_score).unwrap());
                        match binary_search_res {
                            Ok(i) => out_vec.insert(i, surge_candidate_connection),
                            Err(i) => out_vec.insert(i, surge_candidate_connection)
                        }
                    }

                    if out_vec.len() > n {
                        out_vec.remove(n);
                    }
                }
            }
        }

        Ok(out_vec)
    }

    fn _get_n_best_candidate_connections_bidirectional_higher_is_better(&mut self, n: usize, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        if n == 0 {
            return Err("n cannot be 0 in get_best_n_candidate_connections".to_string());
        }

        let mut out_vec: Vec<SurgeCandidateConnection> = Vec::new();

        let best_from_start_dag_nodes = self._get_n_best_candidate_connections_from_start_dag_nodes_higher_is_better(n, lynx_vars, recorder)?;
        let best_from_end_dag_nodes = self._get_n_best_candidate_connections_from_end_dag_nodes_higher_is_better(n, lynx_vars, recorder)?;

        let mut start_dag_vec_idx = 0;
        let mut end_dag_vec_idx = 0;

        loop {
            if out_vec.len() == n {
                break;
            } else if start_dag_vec_idx >= best_from_start_dag_nodes.len() && end_dag_vec_idx >= best_from_end_dag_nodes.len() {
                break;
            } else if start_dag_vec_idx >= best_from_start_dag_nodes.len() {
                out_vec.push( best_from_end_dag_nodes[end_dag_vec_idx].clone() );
                end_dag_vec_idx += 1;
            } else if end_dag_vec_idx >= best_from_end_dag_nodes.len() {
                out_vec.push(best_from_start_dag_nodes[start_dag_vec_idx].clone() );
                start_dag_vec_idx += 1;
            } else {
                if best_from_start_dag_nodes[start_dag_vec_idx].connection_score > best_from_end_dag_nodes[end_dag_vec_idx].connection_score {
                    out_vec.push(best_from_start_dag_nodes[start_dag_vec_idx].clone());
                    start_dag_vec_idx += 1;
                } else {
                    out_vec.push(best_from_end_dag_nodes[end_dag_vec_idx].clone());
                    end_dag_vec_idx += 1;
                }
            }
        }

        Ok(out_vec)
    }
    fn _get_n_best_candidate_connections_from_start_dag_nodes_higher_is_better(&mut self, n: usize, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        if n == 0 {
            return Err("n cannot be 0 in get_best_n_candidate_connections".to_string());
        }

        let mut out_vec: Vec<SurgeCandidateConnection> = Vec::new();

        let start_dag_node_idxs = self._surge_points_manager.get_start_dag_node_idxs_ref();
        let target_state_idxs = self._surge_points_manager.get_start_dag_target_state_idxs_ref();

        for node in start_dag_node_idxs {
            for target in target_state_idxs {
                let mut connection_info_ref = self._surge_connection_manager.get_connection_mut_ref(*node, *target, &mut self._surge_objective_manager, &self._surge_points_manager, lynx_vars, recorder)?;
                if !(connection_info_ref.is_dead_connection()) {
                    let connection_score: f64 = connection_info_ref.get_total_objective_score(&mut self._surge_objective_manager, &self._surge_points_manager, lynx_vars, recorder)?;
                    if out_vec.len() < n || out_vec[n - 1].connection_score < connection_score {
                        let point1 = self._surge_points_manager.get_point_ref(*node)?.clone();
                        let point2 = self._surge_points_manager.get_point_ref(*target)?.clone();
                        let surge_candidate_connection = SurgeCandidateConnection::new(*node, *target, point1, point2, connection_score);

                        let binary_search_res = out_vec.binary_search_by(|x| connection_score.partial_cmp(&x.connection_score).unwrap());
                        match binary_search_res {
                            Ok(i) => out_vec.insert(i, surge_candidate_connection),
                            Err(i) => out_vec.insert(i, surge_candidate_connection)
                        }
                    }

                    if out_vec.len() > n {
                        out_vec.remove(n);
                    }
                }
            }
        }

        Ok(out_vec)
    }
    fn _get_n_best_candidate_connections_from_end_dag_nodes_higher_is_better(&mut self, n: usize, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<Vec<SurgeCandidateConnection>, String> {
        if n == 0 {
            return Err("n cannot be 0 in get_best_n_candidate_connections".to_string());
        }

        let mut out_vec: Vec<SurgeCandidateConnection> = Vec::new();

        let end_dag_node_idxs = self._surge_points_manager.get_end_dag_node_idxs_ref();
        let target_state_idxs = self._surge_points_manager.get_end_dag_target_state_idxs_ref();

        for node in end_dag_node_idxs {
            for target in target_state_idxs {
                let mut connection_info_ref = self._surge_connection_manager.get_connection_mut_ref(*node, *target, &mut self._surge_objective_manager, &self._surge_points_manager, lynx_vars, recorder)?;
                println!("{:?}", connection_info_ref);
                if !(connection_info_ref.is_dead_connection()) {
                    let connection_score: f64 = connection_info_ref.get_total_objective_score(&mut self._surge_objective_manager, &self._surge_points_manager, lynx_vars, recorder)?;
                    if out_vec.len() < n || out_vec[n - 1].connection_score < connection_score {
                        let point1 = self._surge_points_manager.get_point_ref(*node)?.clone();
                        let point2 = self._surge_points_manager.get_point_ref(*target)?.clone();
                        let surge_candidate_connection = SurgeCandidateConnection::new(*node, *target, point1, point2, connection_score);

                        /*
                        let l = out_vec.len();
                        for i in 0..l {
                            let cmp = connection_score.partial_cmp(&out_vec[i].connection_score);
                            if cmp.is_none() {
                                println!("{:?}, {:?}", connection_score, out_vec[i]);
                            }
                        }
                        */

                        let binary_search_res = out_vec.binary_search_by(|x| connection_score.partial_cmp(&x.connection_score).unwrap());
                        match binary_search_res {
                            Ok(i) => out_vec.insert(i, surge_candidate_connection),
                            Err(i) => out_vec.insert(i, surge_candidate_connection)
                        }
                    }

                    if out_vec.len() > n {
                        out_vec.remove(n);
                    }
                }
            }
        }

        Ok(out_vec)
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_dead_connection(&mut self, idx1: usize, idx2: usize) -> Result<(), String> {
        self._surge_connection_manager.set_dead_connection(idx1, idx2)?;
        Ok(())
    }

    pub fn set_dead_connections_from_candidate_connections(&mut self, surge_candidate_connections: &Vec<SurgeCandidateConnection>) {
        for s in surge_candidate_connections {
            self.set_dead_connection(s.idx1, s.idx2);
        }
    }

    pub fn add_successful_connection(&mut self, idx1: usize, idx2: usize, path: &LinearSplinePath) -> Result<(), String> {
        self._surge_points_manager.convert_auto(idx1, idx2)?;
        let planning_dag_start_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_idx_from_surge_points_manager_idx(idx1)?;
        let planning_dag_end_idx = self._planning_dag.add_node_using_inflow_edge(path, planning_dag_start_idx)?;
        self._planning_dag_and_surge_points_manager_idx_util.add_idx_pairing(idx2, planning_dag_end_idx)?;

        Ok(())
    }

    pub fn add_successful_connection_from_candidate_connection(&mut self, surge_candidate_connection: &SurgeCandidateConnection, path: &LinearSplinePath) -> Result<(), String> {
        return self.add_successful_connection(surge_candidate_connection.idx1, surge_candidate_connection.idx2, path);
    }

    pub fn add_successful_connection_auto_linear_path(&mut self, idx1: usize, idx2: usize, step_size: f64) -> Result<(), String> {
        let point_ref_1 = self._surge_points_manager.get_point_ref(idx1)?;
        let point_ref_2 = self._surge_points_manager.get_point_ref(idx2)?;

        let linear_path = LinearSplinePath::new_linear_interpolation(point_ref_1, point_ref_2, step_size);

        return self.add_successful_connection(idx1, idx2, &linear_path);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn initialize_all_surge_objective_terms_with_surge_points_manager(&mut self, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        self._surge_objective_manager.initialize_all_with_surge_points_manager(&self._surge_points_manager, lynx_vars, recorder)?;

        Ok(())
    }

    pub fn update_all_surge_objective_terms_after_connection_attempt(&mut self, idx1: usize, idx2: usize, successful_connection: bool, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<(), String> {
        self._surge_objective_manager.update_all_after_connection_attempt(idx1, idx2, successful_connection, &self._surge_points_manager, lynx_vars, recorder)?;

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn has_at_least_one_solution_been_found(&self) -> bool {
        let l1 = self._surge_points_manager.get_reached_by_start_dag_idxs_ref().len();
        let l2 = self._surge_points_manager.get_reached_by_end_dag_idxs_ref().len();
        return if l1 > 0 || l2 > 0 { true } else { false }
    }

    pub fn get_all_solution_paths(&self) -> Result<Vec<LinearSplinePath>, String> {
        let mut out_vec = self._get_all_solution_paths_reached_last_by_start_dag()?;
        let mut paths_reached_last_by_end_dag = self._get_all_solution_paths_reached_last_by_end_dag()?;
        out_vec.extend(paths_reached_last_by_end_dag);
        return Ok(out_vec);
    }

    pub fn get_first_solution_path(&self) -> Result<Option<LinearSplinePath>, String> {
        let first_solution_reached_last_by_start_dag = self._get_first_solution_reached_last_by_start_dag()?;
        if first_solution_reached_last_by_start_dag.is_some() { return Ok(first_solution_reached_last_by_start_dag); }

        let first_solution_reached_last_by_end_dag = self._get_first_solution_reached_last_by_end_dag()?;
        if first_solution_reached_last_by_end_dag.is_some() { return Ok(first_solution_reached_last_by_end_dag); }

        return Ok(None);
    }

    fn _get_all_solution_paths_reached_last_by_start_dag(&self) -> Result<Vec<LinearSplinePath>, String> {
        let mut out_vec = Vec::new();

        let reached_by_start_dag_idxs = self._surge_points_manager.get_reached_by_start_dag_idxs_ref();

        for r in reached_by_start_dag_idxs {
            let start_planning_dag_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_overflow_idx_from_surge_points_manager_idx(*r)?;
            let end_planning_dag_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_idx_from_surge_points_manager_idx(*r)?;

            let mut start_path = self._planning_dag.get_path_from_tree_root_to_node(start_planning_dag_idx)?;
            let mut end_path = self._planning_dag.get_path_from_tree_root_to_node(end_planning_dag_idx)?;
            end_path.reverse();

            let mut combined_path = start_path.combine_ordered(&end_path)?;
            out_vec.push(combined_path);
        }

        return Ok(out_vec);
    }

    fn _get_all_solution_paths_reached_last_by_end_dag(&self) -> Result<Vec<LinearSplinePath>, String> {
        let mut out_vec = Vec::new();

        let reached_by_end_dag_idxs = self._surge_points_manager.get_reached_by_end_dag_idxs_ref();

        for r in reached_by_end_dag_idxs {
            let start_planning_dag_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_idx_from_surge_points_manager_idx(*r)?;
            let end_planning_dag_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_overflow_idx_from_surge_points_manager_idx(*r)?;

            let mut start_path = self._planning_dag.get_path_from_tree_root_to_node(start_planning_dag_idx)?;
            let mut end_path = self._planning_dag.get_path_from_tree_root_to_node(end_planning_dag_idx)?;
            end_path.reverse();

            let mut combined_path = start_path.combine_ordered(&end_path)?;
            out_vec.push(combined_path);
        }

        return Ok(out_vec);
    }

    fn _get_first_solution_reached_last_by_start_dag(&self) -> Result<Option<LinearSplinePath>, String> {
        let reached_by_start_dag_idxs = self._surge_points_manager.get_reached_by_start_dag_idxs_ref();
        if reached_by_start_dag_idxs.len() == 0 { return Ok(None); }

        let idx = reached_by_start_dag_idxs[0];

        let start_planning_dag_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_overflow_idx_from_surge_points_manager_idx(idx)?;
        let end_planning_dag_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_idx_from_surge_points_manager_idx(idx)?;

        let mut start_path = self._planning_dag.get_path_from_tree_root_to_node(start_planning_dag_idx)?;
        let mut end_path = self._planning_dag.get_path_from_tree_root_to_node(end_planning_dag_idx)?;
        end_path.reverse();

        let mut combined_path = start_path.combine_ordered(&end_path)?;

        return Ok(Some(combined_path));
    }

    fn _get_first_solution_reached_last_by_end_dag(&self) -> Result<Option<LinearSplinePath>, String> {
        let reached_by_end_dag_idxs = self._surge_points_manager.get_reached_by_end_dag_idxs_ref();
        if reached_by_end_dag_idxs.len() == 0 { return Ok(None); }

        let idx = reached_by_end_dag_idxs[0];

        let start_planning_dag_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_idx_from_surge_points_manager_idx(idx)?;
        let end_planning_dag_idx = self._planning_dag_and_surge_points_manager_idx_util.get_planning_dag_overflow_idx_from_surge_points_manager_idx(idx)?;

        let mut start_path = self._planning_dag.get_path_from_tree_root_to_node(start_planning_dag_idx)?;
        let mut end_path = self._planning_dag.get_path_from_tree_root_to_node(end_planning_dag_idx)?;
        end_path.reverse();

        let mut combined_path = start_path.combine_ordered(&end_path)?;

        return Ok(Some(combined_path));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_planning_dag_ref(&self) -> &PlanningDAG { return &self._planning_dag; }

    pub fn get_surge_points_manager_ref(&self) -> &SurgePointsManager { return &self._surge_points_manager; }

    pub fn get_planning_dag_and_surge_points_manager_idx_util_ref(&self) -> &PlanningDAGAndSurgePointsManagerIdxUtil { return &self._planning_dag_and_surge_points_manager_idx_util; }

    pub fn get_surge_connection_manager_ref(&self) -> &SurgeConnectionManager { return &self._surge_connection_manager; }

    pub fn get_surge_objective_manager(&self) -> &SurgeObjectiveManager { return &self._surge_objective_manager; }

    pub fn get_lower_is_better(&self) -> Result<bool, String> {
        return self._surge_objective_manager.get_lower_is_better();
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SurgeCandidateConnection {
    pub idx1: usize,
    pub idx2: usize,
    pub point1: DVector<f64>,
    pub point2: DVector<f64>,
    pub connection_score: f64
}

impl SurgeCandidateConnection {
    pub fn new(idx1: usize, idx2: usize, point1: DVector<f64>, point2: DVector<f64>, connection_score: f64) -> Self {
        Self { idx1, idx2, point1, point2, connection_score }
    }

    pub fn print_summary(&self) {
        println!("idx1: {:?}, point1: {:?}", self.idx1, self.point1);
        println!("idx2: {:?}, point2: {:?}", self.idx2, self.point2);
        println!("connection score: {:?}", self.connection_score);
    }
}
unsafe impl Send for SurgeCandidateConnection { }
unsafe impl Sync for SurgeCandidateConnection { }

