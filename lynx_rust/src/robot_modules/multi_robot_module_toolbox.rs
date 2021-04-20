use crate::robot_modules::robot_module_toolbox::*;
use crate::utils::utils_collisions::collision_object_group_queries::*;
use crate::robot_modules::robot_fk_module::*;
use nalgebra::DVector;
use std::slice::{Iter, IterMut};
use termion::{style, color};

#[derive(Clone, Debug)]
pub struct MultiRobotModuleToolbox {
    _robot_module_toolboxes: Vec<RobotModuleToolbox>,
    _dofs_per_robot: Vec<usize>,
    _total_num_dofs: usize,
    _num_robots: usize
}
impl MultiRobotModuleToolbox {
    pub fn new(robot_names: Vec<&str>, configuration_names: Vec<Option<&str>>, mobile_base_bounds_filenames: Vec<Option<&str>>) -> Result<Self, String> {
        if robot_names.len() == configuration_names.len() && configuration_names.len() == mobile_base_bounds_filenames.len() {
            let mut out_self = Self::new_empty();
            let l = robot_names.len();
            for i in 0..l {
                out_self.add_robot_module_toolbox(robot_names[i], configuration_names[i], mobile_base_bounds_filenames[i])?;
            }
            return Ok(out_self);
        }
        else {
            return Err(format!("number of robot names ({:?}), number of configuration names ({:?}), and number of mobile_base_bounds_filenames ({:?}) must be equal in MultiRobotModuleToolbox", robot_names.len(), configuration_names.len(), mobile_base_bounds_filenames.len()));
        }
    }

    pub fn new_empty() -> Self {
        return Self { _robot_module_toolboxes: Vec::new(), _dofs_per_robot: Vec::new(), _total_num_dofs: 0, _num_robots: 0 };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_robot_module_toolbox(&mut self, robot_name: &str, configuration_name: Option<&str>, mobile_bounds_file_name: Option<&str>) -> Result<(), String> {
        let robot_module_toolbox = RobotModuleToolbox::new_lite(robot_name, configuration_name, mobile_bounds_file_name)?;
        let dofs = robot_module_toolbox.get_dof_module_ref().get_num_dofs();
        self._dofs_per_robot.push(dofs);
        self._total_num_dofs += dofs;
        self._robot_module_toolboxes.push(robot_module_toolbox);
        self._num_robots += 1;
        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn split_full_state_vector_into_robot_state_vectors(&self, state_vector: &DVector<f64>) -> Result<Vec<DVector<f64>>, String> {
        if state_vector.len() != self._total_num_dofs {
            return Err(format!("state vector length {:?} does not equal total num dofs {:?} in MultiRobotModuleToolbox", state_vector.len(), self._total_num_dofs));
        }

        if self._robot_module_toolboxes.len() == 1 { return Ok(vec![state_vector.clone()]); }

        let mut out_vec = Vec::new();

        let mut curr_idx = 0;

        for i in 0..self._num_robots {
            out_vec.push(DVector::from_element(self._dofs_per_robot[i], 0.0));
            for j in 0..self._dofs_per_robot[i] {
                out_vec[i][j] = state_vector[curr_idx];
                curr_idx += 1;
            }
        }

        return Ok(out_vec);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn compute_fk(&self, full_state_vec: &DVector<f64>) -> Result<VecOfRobotFKResult, String> {
        let mut out_vec = VecOfRobotFKResult::new_emtpy();

        let robot_state_vecs = self.split_full_state_vector_into_robot_state_vectors(full_state_vec)?;
        for i in 0..self._num_robots {
            out_vec.add_robot_fk_result( self._robot_module_toolboxes[i].get_fk_module_ref().compute_fk(&robot_state_vecs[i])? );
        }

        return Ok(out_vec);
    }

    pub fn print_results_next_to_link_names(&self, fk_res: &VecOfRobotFKResult) {
        for i in 0..self._num_robots {
            println!("{}{}Robot {:?} ---> {}", style::Bold, color::Fg(color::Magenta), i, style::Reset);
            self._robot_module_toolboxes[i]
                .get_fk_module_ref()
                .print_results_next_to_link_names(&fk_res.get_robot_fk_results_ref()[i],
                                                  &self._robot_module_toolboxes[i].get_configuration_module_ref());
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_iter(&self) -> Iter<RobotModuleToolbox> {
        return self._robot_module_toolboxes.iter();
    }

    pub fn get_iter_mut(&mut self) -> IterMut<RobotModuleToolbox> {
        return self._robot_module_toolboxes.iter_mut();
    }

    pub fn get_robot_module_toolboxes_ref(&self) -> &Vec<RobotModuleToolbox> { return &self._robot_module_toolboxes; }

    pub fn get_robot_module_toolboxes_mut_ref(&mut self) -> &mut Vec<RobotModuleToolbox> { return &mut self._robot_module_toolboxes; }

    pub fn get_dofs_per_robot_ref(&self) -> &Vec<usize> { return &self._dofs_per_robot; }

    pub fn get_total_num_dofs(&self) -> usize { return self._total_num_dofs; }

    pub fn get_num_robots(&self) -> usize { return self._num_robots; }
}


