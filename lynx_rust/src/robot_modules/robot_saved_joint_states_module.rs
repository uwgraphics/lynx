use nalgebra::DVector;
use std::collections::HashMap;
use termion::{style, color};
use serde::{Serialize, Deserialize};
use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use crate::robot_modules::robot_dof_module::RobotDOFModule;
use crate::utils::utils_files_and_strings::robot_folder_utils::*;

#[derive(Serialize, Deserialize, Clone)]
pub struct RobotSavedJointStatesModule {
    _robot_name: String,
    _configuration_name: String,
    _num_dofs: usize,
    _saved_states: Vec<DVector<f64>>,
    _names_hashmap: HashMap<String, usize>,
    _default_joint_state_idx: Option<usize>
}

impl RobotSavedJointStatesModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule, robot_dof_module: &RobotDOFModule) -> Self {
        let mut out_self = Self {
            _robot_name: robot_configuration_module.robot_model_module.robot_name.clone(),
            _configuration_name: robot_configuration_module.configuration_name.clone(),
            _num_dofs: robot_dof_module.get_num_dofs(),
            _saved_states: Vec::new(),
            _names_hashmap: HashMap::new(),
            _default_joint_state_idx: None
        };

        out_self._load_from_file();
        out_self.save_to_file();
        return out_self;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_state(&mut self, state: &DVector<f64>, name: String) {
        if state.len() != self._num_dofs { return; }

        let hashmap_check = self._names_hashmap.get(&name);
        if hashmap_check.is_some() {
            let idx = hashmap_check.unwrap();
            self._saved_states[*idx] = state.clone();
        } else {
            let add_idx = self._saved_states.len();
            self._saved_states.push(state.clone());
            self._names_hashmap.insert(name.clone(), add_idx);
            if name.clone() == "default".to_string() {
                self._default_joint_state_idx = Some(add_idx);
            }
        }
    }

    pub fn get_state_by_name(&self, name: &String) -> Option<DVector<f64>> {
        let idx = self._names_hashmap.get(name);
        return self.get_state_by_idx(*idx.unwrap());
    }

    pub fn get_state_by_idx(&self, idx: usize) -> Option<DVector<f64>> {
        return if idx >= self._saved_states.len() { None } else {
            Some(self._saved_states[idx].clone())
        }
    }

    pub fn get_default_joint_state(&self) -> Option<DVector<f64>> {
        return if self._default_joint_state_idx.is_some() {
            Some(self._saved_states[self._default_joint_state_idx.unwrap()].clone())
        } else {
            None
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _load_from_file(&mut self) -> bool {
        let file_name = self._configuration_name.clone() + ".json";
        let res = load_from_json_file_relative_to_robot_directory!(Self, self._robot_name.clone(), "autogenerated_metadata/saved_joint_states".to_string(), file_name);
        return if res.is_err() {
            false
        } else {
            *self = res.unwrap();
            true
        }
    }

    pub fn save_to_file(&self) {
        let file_name = self._configuration_name.clone() + ".json";
        dump_to_json_file_relative_to_robot_directory!(self, self._robot_name.clone(), "autogenerated_metadata/saved_joint_states".to_string(), file_name);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let keys = self._names_hashmap.keys();
        let mut count = 0;
        for k in keys {
            let value = self._names_hashmap.get(k).unwrap();
            println!("{}{} saved joint state {} ---> {}", style::Bold, color::Fg(color::Blue), count, style::Reset);
            println!("      name: {}, state: {:?}", k, self._saved_states[*value]);
            count += 1;
        }
    }
}


