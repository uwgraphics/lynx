use nalgebra::{DVector};
use termion::{style, color};
use crate::utils::utils_path_planning::utils_surge::surge_points_manager::SurgePointType::*;

#[derive(Clone)]
pub struct SurgePointsManager {
    _all_points: Vec<DVector<f64>>,
    _surge_point_types: Vec<Vec<SurgePointType>>,
    _start_state_idxs: Vec<usize>,
    _end_state_idxs: Vec<usize>,
    _start_dag_node_idxs: Vec<usize>,
    _start_dag_target_state_idxs: Vec<usize>,
    _end_dag_node_idxs: Vec<usize>,
    _end_dag_target_state_idxs: Vec<usize>,
    _milestone_idxs: Vec<usize>,
    _reached_by_start_dag_idxs: Vec<usize>,
    _reached_by_end_dag_idxs: Vec<usize>
}

impl SurgePointsManager {
    pub fn new_empty() -> Self {
        let _all_points = Vec::new();
        let _surge_point_types = Vec::new();
        let _start_state_idxs = Vec::new();
        let _end_state_idxs = Vec::new();
        let _start_dag_node_idxs = Vec::new();
        let _start_dag_target_state_idxs = Vec::new();
        let _end_dag_node_idxs = Vec::new();
        let _end_dag_target_state_idxs = Vec::new();
        let _milestone_idxs = Vec::new();
        let _reached_by_start_dag_idxs = Vec::new();
        let _reached_by_end_dag_idxs = Vec::new();

        return Self { _all_points, _surge_point_types,_start_state_idxs, _end_state_idxs, _start_dag_node_idxs,
            _start_dag_target_state_idxs, _end_dag_node_idxs, _end_dag_target_state_idxs, _milestone_idxs,
            _reached_by_start_dag_idxs, _reached_by_end_dag_idxs };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_start_state(&mut self, state: &DVector<f64>) -> usize {
        self._all_points.push( state.clone() );
        let add_idx = self._all_points.len() - 1;

        self._start_state_idxs.push(add_idx);
        self._start_dag_node_idxs.push(add_idx);
        self._end_dag_target_state_idxs.push(add_idx);

        self._surge_point_types.push( vec![ StartState, StartDAGNode, EndDAGTargetState ] );

        return add_idx;
    }

    pub fn add_end_state(&mut self, state: &DVector<f64>) -> usize {
        self._all_points.push( state.clone() );
        let add_idx = self._all_points.len() - 1;

        self._end_state_idxs.push(add_idx);
        self._end_dag_node_idxs.push(add_idx);
        self._start_dag_target_state_idxs.push(add_idx);

        self._surge_point_types.push( vec![ EndState, EndDAGNode, StartDAGTargetState ] );

        return add_idx;
    }

    pub fn add_milestone_state(&mut self, state: &DVector<f64>) -> usize {
        self._all_points.push( state.clone() );
        let add_idx = self._all_points.len() - 1;

        self._milestone_idxs.push(add_idx);
        self._start_dag_target_state_idxs.push(add_idx);
        self._end_dag_target_state_idxs.push(add_idx);

        self._surge_point_types.push( vec![ Milestone, StartDAGTargetState, EndDAGTargetState ] );

        return add_idx
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn convert_auto(&mut self, idx1: usize, idx2: usize) -> Result<(), String> {
        if self.is_start_dag_node_idx(idx1)? && self.is_milestone_idx(idx2)? {
            return self.convert_milestone_to_start_dag_node(idx2);
        } else if self.is_end_dag_node_idx(idx1)? && self.is_milestone_idx(idx2)? {
            return self.convert_milestone_to_end_dag_node(idx2);
        } else if self.is_start_dag_node_idx(idx1)? && self.is_end_state_idx(idx2)? {
            return self.convert_end_state_to_reached_by_start_dag(idx2);
        } else if self.is_end_dag_node_idx(idx1)? && self.is_start_state_idx(idx2)? {
            return self.convert_start_state_to_reached_by_end_dag(idx2);
        } else if self.is_start_dag_node_idx(idx1)? && self.is_end_dag_node_idx(idx2)? {
            return self.convert_end_dag_node_to_reached_by_start_dag(idx2);
        } else if self.is_end_dag_node_idx(idx1)? && self.is_start_dag_node_idx(idx2)? {
            return self.convert_start_dag_node_to_reached_by_end_dag(idx2);
        }

        return Err(format!("idx1 {:?} types {:?} and idx2 {:?} types {:?} were incompatible for auto conversion", idx1, self._surge_point_types[idx1], idx2, self._surge_point_types[idx2]));
    }

    pub fn convert_milestone_to_start_dag_node(&mut self, idx: usize) -> Result<(), String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        let is_milestone_idx = self.is_milestone_idx(idx)?;
        if is_milestone_idx {

            let binary_search_res = self._milestone_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._milestone_idxs.remove(i); }
                _ => {  }
            }

            let binary_search_res = self._start_dag_target_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._start_dag_target_state_idxs.remove(i); }
                _ => {  }
            }

            let binary_search_res = self._start_dag_node_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Err(i) => { self._start_dag_node_idxs.insert(i, idx); },
                _ => { }
            }

            self._surge_point_types[idx] = vec![ StartDAGNode, EndDAGTargetState ];

        } else {
            return Err(format!("idx {:?} is not a milestone.", idx));
        }

        Ok(())
    }

    pub fn convert_milestone_to_end_dag_node(&mut self, idx: usize) -> Result<(), String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        let is_milestone_idx = self.is_milestone_idx(idx)?;
        if is_milestone_idx {
            let binary_search_res = self._milestone_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._milestone_idxs.remove(i); }
                _ => {  }
            }

            let binary_search_res = self._end_dag_target_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._end_dag_target_state_idxs.remove(i); }
                _ => {  }
            }

            let binary_search_res = self._end_dag_node_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Err(i) => { self._end_dag_node_idxs.insert(i, idx); },
                _ => { }
            }

            self._surge_point_types[idx] = vec![ EndDAGNode, StartDAGTargetState ];

        } else {
            return Err(format!("idx {:?} is not a milestone.", idx));
        }

        Ok(())
    }

    pub fn convert_end_state_to_reached_by_start_dag(&mut self, idx: usize) -> Result<(), String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        let is_end_state = self.is_end_state_idx(idx)?;
        if is_end_state {
            /*
            let binary_search_res = self._end_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._end_state_idxs.remove(i); }
                _ => {  }
            }
            */

            let binary_search_res = self._start_dag_target_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._start_dag_target_state_idxs.remove(i); }
                _ => {  }
            }

            let binary_search_res = self._reached_by_start_dag_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Err(i) => { self._reached_by_start_dag_idxs.insert(i, idx); },
                _ => { }
            }

            self._surge_point_types[idx] = vec![ EndState, EndDAGNode, ReachedByStartDAG ];

            Ok(())
        } else {
            return Err(format!("idx {:?} is not an end state.", idx));
        }
    }

    pub fn convert_start_state_to_reached_by_end_dag(&mut self, idx: usize) -> Result<(), String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        let is_start_state = self.is_start_state_idx(idx)?;
        if is_start_state {
            /*
            let binary_search_res = self._start_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._start_state_idxs.remove(i); }
                _ => {  }
            }
            */

            let binary_search_res = self._end_dag_target_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._end_dag_target_state_idxs.remove(i); }
                _ => {  }
            }

            let binary_search_res = self._reached_by_end_dag_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Err(i) => { self._reached_by_end_dag_idxs.insert(i, idx); },
                _ => { }
            }

            self._surge_point_types[idx] = vec![ StartState, StartDAGNode, ReachedByENDDAG ];

            Ok(())
        } else {
            return Err(format!("idx {:?} is not a start state.", idx));
        }
    }

    pub fn convert_end_dag_node_to_reached_by_start_dag(&mut self, idx: usize) -> Result<(), String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }
        let is_end_dag_node = self.is_end_dag_node_idx(idx)?;
        if is_end_dag_node {
            /*
            let binary_search_res = self._start_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._start_state_idxs.remove(i); }
                _ => {  }
            }
            */

            let binary_search_res = self._start_dag_target_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._start_dag_target_state_idxs.remove(i); }
                _ => {  }
            }

            let binary_search_res = self._reached_by_start_dag_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Err(i) => { self._reached_by_start_dag_idxs.insert(i, idx); },
                _ => { }
            }

            self._surge_point_types[idx] = vec![ EndDAGNode, ReachedByStartDAG ];

            Ok(())
        } else {
            return Err(format!("idx {:?} is not a start dag node.", idx));
        }
    }

    pub fn convert_start_dag_node_to_reached_by_end_dag(&mut self, idx: usize) -> Result<(), String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }
        let is_start_dag_node = self.is_start_dag_node_idx(idx)?;
        if is_start_dag_node {
            /*
            let binary_search_res = self._start_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._start_state_idxs.remove(i); }
                _ => {  }
            }
            */

            let binary_search_res = self._end_dag_target_state_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Ok(i) => { self._end_dag_target_state_idxs.remove(i); }
                _ => {  }
            }

            let binary_search_res = self._reached_by_end_dag_idxs.binary_search_by(|x| x.cmp(&idx));
            match binary_search_res {
                Err(i) => { self._reached_by_end_dag_idxs.insert(i, idx); },
                _ => { }
            }

            self._surge_point_types[idx] = vec![ StartDAGNode, ReachedByENDDAG ];

            Ok(())
        } else {
            return Err(format!("idx {:?} is not a start dag node.", idx));
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn is_start_state_idx(&self, idx: usize) -> Result<bool, String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        return Ok( self._surge_point_types[idx].contains(&StartState) );
    }

    pub fn is_end_state_idx(&self, idx: usize) -> Result<bool, String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        return Ok( self._surge_point_types[idx].contains(&EndState) );
    }

    pub fn is_start_dag_node_idx(&self, idx: usize) -> Result<bool, String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        return Ok( self._surge_point_types[idx].contains(&StartDAGNode) );
    }

    pub fn is_start_dag_target_state_idx(&self, idx: usize) -> Result<bool, String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        return Ok( self._surge_point_types[idx].contains(&StartDAGTargetState) );
    }

    pub fn is_end_dag_node_idx(&self, idx: usize) -> Result<bool, String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        return Ok( self._surge_point_types[idx].contains(&EndDAGNode) );
    }

    pub fn is_end_dag_target_state_idx(&self, idx: usize) -> Result<bool, String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        return Ok( self._surge_point_types[idx].contains(&EndDAGTargetState) );
    }

    pub fn is_milestone_idx(&self, idx: usize) -> Result<bool, String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        return Ok( self._surge_point_types[idx].contains(&Milestone) );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn are_all_start_and_end_states_reached(&self) -> bool {
        return self.are_all_start_states_reached() && self.are_all_end_states_reached();
    }

    pub fn are_all_start_states_reached(&self) -> bool {
        if self._start_state_idxs.len() == self._reached_by_end_dag_idxs.len() {
            return true;
        } else {
            return false;
        }
    }

    pub fn are_all_end_states_reached(&self) -> bool {
        if self._end_state_idxs.len() == self._reached_by_start_dag_idxs.len() {
            return true;
        } else {
            return false;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_point_ref(&self, idx: usize) -> Result<&DVector<f64>, String> {
        if idx >= self._all_points.len() {
            return Err(format!("idx {:?} is too high for number of points ({:?})", idx, self._all_points.len()));
        }

        return Ok(&self._all_points[idx]);
    }

    pub fn get_all_points_ref(&self) -> &Vec<DVector<f64>> {
        return &self._all_points;
    }

    pub fn get_start_state_idxs_ref(&self) -> &Vec<usize> {
        return &self._start_state_idxs;
    }

    pub fn get_end_state_idxs_ref(&self) -> &Vec<usize> {
        return &self._end_state_idxs;
    }

    pub fn get_start_dag_node_idxs_ref(&self) -> &Vec<usize> {
        return &self._start_dag_node_idxs;
    }

    pub fn get_start_dag_target_state_idxs_ref(&self) -> &Vec<usize> {
        return &self._start_dag_target_state_idxs;
    }

    pub fn get_end_dag_node_idxs_ref(&self) -> &Vec<usize> {
        return &self._end_dag_node_idxs;
    }

    pub fn get_end_dag_target_state_idxs_ref(&self) -> &Vec<usize> {
        return &self._end_dag_target_state_idxs;
    }

    pub fn get_milestone_idxs_ref(&self) -> &Vec<usize> {
        return &self._milestone_idxs;
    }

    pub fn get_reached_by_start_dag_idxs_ref(&self) -> &Vec<usize> {
        return &self._reached_by_start_dag_idxs;
    }

    pub fn get_reached_by_end_dag_idxs_ref(&self) -> &Vec<usize> {
        return &self._reached_by_end_dag_idxs;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        println!("{}{} surge points manager summary{}", style::Bold, color::Fg(color::LightCyan), style::Reset);
        println!("{}    num points: {:?} {}", color::Fg(color::LightCyan), self._all_points.len(),style::Reset);
        println!("{}    num reached by start dag: {:?} {}", color::Fg(color::LightCyan), self._reached_by_start_dag_idxs.len(),style::Reset);
        println!("{}    num reached by end dag: {:?} {}", color::Fg(color::LightCyan), self._reached_by_end_dag_idxs.len(),style::Reset);
        println!("{}    num start states: {:?} {}", color::Fg(color::LightCyan), self._start_state_idxs.len(),style::Reset);
        println!("{}    num end states: {:?} {}", color::Fg(color::LightCyan), self._end_state_idxs.len(),style::Reset);
        println!("{}    num start dag nodes: {:?} {}", color::Fg(color::LightCyan), self._start_dag_node_idxs.len(),style::Reset);
        println!("{}    num end dag nodes: {:?} {}", color::Fg(color::LightCyan), self._end_dag_node_idxs.len(),style::Reset);
        println!("{}    num milestones: {:?} {}", color::Fg(color::LightCyan), self._milestone_idxs.len(),style::Reset);
        println!("---------------------------------------------------------------------------------");
        print!("{} all points: {}", color::Fg(color::LightWhite), style::Reset);
        let l = self._all_points.len();
        for i in 0..l {
            print!("{}{:?}: {:?}{}, ", color::Fg(color::LightWhite), i, self._all_points[i].data.as_vec(), style::Reset);
        }
        print!("\n");
        print!("{} all points types: {}", color::Fg(color::LightWhite), style::Reset);
        for i in 0..l {
            print!("{}{:?}: {:?}{}, ", color::Fg(color::LightWhite), i, self._surge_point_types[i], style::Reset);
        }
        print!("\n");
        println!("---------------------------------------------------------------------------------");
        println!("{} start state idxs: {:?}{}", color::Fg(color::LightGreen), self._start_state_idxs,  style::Reset);
        println!("{} start dag node idxs: {:?}{}", color::Fg(color::LightGreen), self._start_dag_node_idxs,  style::Reset);
        println!("{} >>> start dag target state idxs: {:?}{}", color::Fg(color::LightGreen), self._start_dag_target_state_idxs,  style::Reset);
        println!("{} *** reached by start dag idxs: {:?}{}", color::Fg(color::LightGreen), self._reached_by_start_dag_idxs,  style::Reset);
        println!("---------------------------------------------------------------------------------");
        println!("{} end state idxs: {:?}{}", color::Fg(color::AnsiValue::rgb(4,3, 1)), self._end_state_idxs,  style::Reset);
        println!("{} end dag node idxs: {:?}{}", color::Fg(color::AnsiValue::rgb(4,3, 1)), self._end_dag_node_idxs,  style::Reset);
        println!("{} >>> end dag target state idxs: {:?}{}", color::Fg(color::AnsiValue::rgb(4,3, 1)), self._end_dag_target_state_idxs,  style::Reset);
        println!("{} *** reached by end dag idxs: {:?}{}", color::Fg(color::AnsiValue::rgb(4,3, 1)), self._reached_by_end_dag_idxs,  style::Reset);
        println!("---------------------------------------------------------------------------------");
        println!("{} milestone idxs: {:?}{}", color::Fg(color::AnsiValue::rgb(2,1, 4)), self._milestone_idxs,  style::Reset);

    }
}

#[derive(Clone, PartialEq, Debug)]
enum SurgePointType {
    StartState, EndState, StartDAGNode, StartDAGTargetState, EndDAGNode, EndDAGTargetState, Milestone, ReachedByStartDAG, ReachedByENDDAG
}