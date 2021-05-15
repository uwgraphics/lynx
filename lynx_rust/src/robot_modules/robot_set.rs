use crate::robot_modules::robot::*;
use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use crate::robot_modules::robot_fk_module::*;
use crate::utils::utils_files_and_strings::prelude::*;
use crate::utils::utils_se3::implicit_dual_quaternion::*;
use crate::utils::utils_parsing::yaml_parsing_utils::get_yaml_obj;
use crate::utils::utils_collisions::prelude::*;
use crate::robot_modules::robot_fk_module::*;
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;
use crate::robot_modules::robot_bounds_module::BoundsCheckResult;
use nalgebra::DVector;
use std::slice::{Iter, IterMut};
use termion::{style, color};
use yaml_rust::yaml::Yaml::Null;
use nalgebra::{UnitQuaternion, Vector3};


#[derive(Clone, Debug)]
pub struct RobotSet {
    _robots: Vec<Robot>,
    _dofs_per_robot: Vec<usize>,
    _total_num_dofs: usize,
    _num_robots: usize,
    _robot_set_result_vector_idxs_to_robot_idxs: Vec<(usize, usize)>
}

impl RobotSet {
    pub fn new(robot_names: Vec<&str>, configuration_names: Vec<Option<&str>>) -> Result<Self, String> {
        if robot_names.len() == configuration_names.len() {
            let mut out_self = Self::new_empty();
            let l = robot_names.len();
            for i in 0..l {
                out_self.add_robot(robot_names[i], configuration_names[i])?;
            }
            return Ok(out_self);
        }
        else {
            return Err(format!("number of robot names ({:?}) and number of configuration names ({:?}) must be equal in MultiRobotModuleToolbox", robot_names.len(), configuration_names.len()));
        }
    }

    pub fn new_from_set_name(robot_set_name: &str) -> Result<Self, String> {
        let mut out_self = Self::new_empty();
        out_self._load_from_yaml_file(robot_set_name)?;
        return Ok(out_self);
    }

    pub fn new_empty() -> Self {
        let _robot_set_result_vector_idxs_to_robot_idxs = Vec::new();

        return Self { _robots: Vec::new(), _dofs_per_robot: Vec::new(), _total_num_dofs: 0, _num_robots: 0, _robot_set_result_vector_idxs_to_robot_idxs };
    }

    fn _new_from_robot_configuration_modules(robot_configuration_modules: Vec<RobotConfigurationModule>) -> Result<Self, String> {
        let mut out_self = Self::new_empty();
        let l = robot_configuration_modules.len();
        for i in 0..l {
            out_self.add_robot_from_robot_configuration_module(&robot_configuration_modules[i])?;
        }
        return Ok(out_self);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_robot(&mut self, robot_name: &str, configuration_name: Option<&str>) -> Result<(), String> {
        let robot_module_toolbox = Robot::new(robot_name, configuration_name)?;
        self._add_robot(robot_module_toolbox);

        Ok(())
    }

    pub fn add_robot_from_robot_configuration_module(&mut self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let robot_module_toolbox = Robot::new_from_configuration_module(robot_configuration_module)?;
        self._add_robot(robot_module_toolbox);

        Ok(())
    }

    fn _add_robot(&mut self, robot_module_toolbox: Robot) {
        let dofs = robot_module_toolbox.get_dof_module_ref().get_num_dofs();
        self._dofs_per_robot.push(dofs);
        self._total_num_dofs += dofs;
        self._robots.push(robot_module_toolbox);
        self._num_robots += 1;
        self._set_robot_set_result_vector_idxs_to_robot_idxs();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn split_full_state_vector_into_robot_state_vectors(&self, state_vector: &DVector<f64>) -> Result<Vec<DVector<f64>>, String> {
        if state_vector.len() != self._total_num_dofs {
            return Err(format!("state vector length {:?} does not equal total num dofs {:?} in MultiRobotModuleToolbox", state_vector.len(), self._total_num_dofs));
        }

        if self._robots.len() == 1 { return Ok(vec![state_vector.clone()]); }

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

    pub fn check_if_state_is_within_bounds(&self, full_state_vec: &DVector<f64>) -> Result<BoundsCheckResult, String> {
        let robot_state_vecs = self.split_full_state_vector_into_robot_state_vectors(full_state_vec)?;

        for i in 0..self._num_robots {
            let res = self._robots[i].get_bounds_module_ref().check_if_state_is_within_bounds(&robot_state_vecs[i]);
            match res {
                BoundsCheckResult::InBounds => {  }
                BoundsCheckResult::OutOfBounds(s) => { return Ok(BoundsCheckResult::OutOfBounds(s)) }
                BoundsCheckResult::Error(s) => { return Err(s); }
            }
        }

        return Ok(BoundsCheckResult::InBounds);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn self_intersect_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();
        let l = self.get_num_robots();
        for i in 0..l {
            let res = self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().self_intersect_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            let label = format!("self intersect check on robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_distance_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfDistanceCheckMultipleResult, String> {
        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();
        let l = self.get_num_robots();
        for i in 0..l {
            let res = self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().self_distance_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            let label = format!("self distance check on robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_contact_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VecOfContactCheckMultipleResult, String> {
        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();
        let l = self.get_num_robots();
        for i in 0..l {
            let res = self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().self_contact_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected, margin)?;

            let label = format!("self contact check on robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_intersect_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        if subset_check_idxs.len() != self.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self.get_num_robots()));
        }

        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();
        let l = self.get_num_robots();
        for i in 0..l {
            let res = self.get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_intersect_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            let label = format!("self intersect subset check on robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_distance_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfDistanceCheckMultipleResult, String> {
        if subset_check_idxs.len() != self.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self.get_num_robots()));
        }

        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();
        let l = self.get_num_robots();
        for i in 0..l {
            let res = self.get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_distance_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            let label = format!("self distance check subset on robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_contact_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VecOfContactCheckMultipleResult, String> {
        if subset_check_idxs.len() != self.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self.get_num_robots()));
        }

        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();
        let l = self.get_num_robots();
        for i in 0..l {
            let res = self.get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_contact_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected, margin)?;

            let label = format!("self contact check subset on robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }
        return Ok(out_vec);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn multi_robot_intersect_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let l = self.get_num_robots().clone();
        for i in 0..l {
            self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self.get_robots_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self.get_robots_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self.get_robots_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = intersect_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, None)?;

                    let label = format!("multi robot intersect check between robot {} ({:?}) and robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self.get_robots_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res, label);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res, label);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_distance_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfDistanceCheckMultipleResult, String> {
        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let l = self.get_num_robots().clone();
        for i in 0..l {
            self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self.get_robots_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self.get_robots_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self.get_robots_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = distance_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, None, None)?;

                    let label = format!("multi robot distance check between robot {} ({:?}) and robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self.get_robots_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res, label);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res, label);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_contact_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VecOfContactCheckMultipleResult, String> {
        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let l = self.get_num_robots().clone();
        for i in 0..l {
            self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self.get_robots_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self.get_robots_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self.get_robots_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = contact_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, margin, None, None)?;


                    let label = format!("multi robot contact check between robot {} ({:?}) and robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self.get_robots_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res, label);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res, label);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_intersect_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._robot_set_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._robot_set_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let mut count = 0;

        let l = self.get_num_robots().clone();
        for i in 0..l {
            self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self.get_robots_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self.get_robots_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self.get_robots_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = intersect_check_between_multiple_collision_objects_subset(&subset_check_idxs[count], collision_objects1, collision_objects2, stop_at_first_detected, None)?;
                    count += 1;

                    let label = format!("multi robot intersect subset check between robot {} ({:?}) and robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self.get_robots_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res, label);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res, label);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_distance_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfDistanceCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._robot_set_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._robot_set_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let mut count = 0;

        let l = self.get_num_robots().clone();
        for i in 0..l {
            self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self.get_robots_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self.get_robots_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self.get_robots_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = distance_check_between_multiple_collision_objects_subset(&subset_check_idxs[count], collision_objects1, collision_objects2, stop_at_first_detected, None, None)?;
                    count += 1;

                    let label = format!("multi robot distance check subset between robot {} ({:?}) and robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self.get_robots_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res, label);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res, label);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_contact_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VecOfContactCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._robot_set_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._robot_set_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let mut count = 0;

        let l = self.get_num_robots().clone();
        for i in 0..l {
            self.get_robots_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self.get_robots_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self.get_robots_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self.get_robots_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = contact_check_between_multiple_collision_objects_subset(&subset_check_idxs[count], collision_objects1, collision_objects2, stop_at_first_detected, margin, None, None)?;
                    count += 1;

                    let label = format!("multi robot contact subset check between robot {} ({:?}) and robot {} ({:?})", i, self.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self.get_robots_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res, label);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res, label);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn compute_fk(&self, full_state_vec: &DVector<f64>) -> Result<VecOfRobotFKResult, String> {
        let mut out_vec = VecOfRobotFKResult::new_emtpy();

        let robot_state_vecs = self.split_full_state_vector_into_robot_state_vectors(full_state_vec)?;
        for i in 0..self._num_robots {
            out_vec.add_robot_fk_result( self._robots[i].get_fk_module_ref().compute_fk(&robot_state_vecs[i])? );
        }

        return Ok(out_vec);
    }

    pub fn print_results_next_to_link_names(&self, fk_res: &VecOfRobotFKResult) {
        for i in 0..self._num_robots {
            println!("{}{}Robot {:?} ---> {}", style::Bold, color::Fg(color::Magenta), i, style::Reset);
            self._robots[i]
                .get_fk_module_ref()
                .print_results_next_to_link_names(&fk_res.get_robot_fk_results_ref()[i],
                                                  &self._robots[i].get_configuration_module_ref());
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_link_order(&self) {
        let l = self._robots.len();
        for i in 0..l {
            println!("{}{}Robot {} ---> {}", style::Bold, color::Fg(color::Magenta), i, style::Reset);
            self._robots[i].get_configuration_module_ref().robot_model_module.print_link_order();
        }
    }

    pub fn print_joint_dof_order(&self) {
        let l = self._robots.len();
        for i in 0..l {
            println!("{}{}Robot {} ---> {}", style::Bold, color::Fg(color::Magenta), i, style::Reset);
            self._robots[i].get_dof_module_ref().print_joint_dof_order();
        }
    }

    pub fn print_bounds(&self) {
        let l = self._robots.len();
        for i in 0..l {
            println!("{}{}Robot {} ---> {}", style::Bold, color::Fg(color::Magenta), i, style::Reset);
            self._robots[i].get_bounds_module_ref().print_bounds();
        }
    }

    pub fn print_salient_links(&self) {
        let l = self._robots.len();
        for i in 0..l {
            println!("{}{}Robot {} ---> {}", style::Bold, color::Fg(color::Magenta), i, style::Reset);
            self._robots[i].get_salient_links_module_ref().print_summary();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _load_from_yaml_file(&mut self, robot_set_name: &str) -> Result<(), String> {
        let fp = get_path_to_src() + "robot_sets/" + robot_set_name + ".yaml";

        let path_exists = check_if_path_exists(fp.clone());
        if !path_exists {
            return Err(format!("robot_set {} does not exist.", robot_set_name));
        }

        let y = get_yaml_obj(fp.clone())?;

        let robots_by_filenames_vec = if y[0]["robots_by_filenames"] == Null { vec![ ] } else { y[0]["robots_by_filenames"].as_vec().expect("robots_by_filename must be included in robot_sets yaml").to_vec() };

        let l = robots_by_filenames_vec.len();
        for i in 0..l {
            let robot_name = robots_by_filenames_vec[i]["robot_name"].as_str().expect("robot_name must be included in robots_by_filename in robots_sets yaml");
            let configuration_name = if robots_by_filenames_vec[i]["configuration_name"] == Null { None } else { Some( robots_by_filenames_vec[i]["configuration_name"].as_str().expect("configuration_name must be included in robots_by_filename in robots_sets yaml") ) };

            let robot_configuration = RobotConfigurationModule::new(robot_name, configuration_name)?;

            self.add_robot_from_robot_configuration_module(&robot_configuration);
        }

        let robots_by_filenames_vec = if y[0]["robots_by_manual_inputs"] == Null { vec![ ] } else { y[0]["robots_by_manual_inputs"].as_vec().expect("robots_by_manual_inputs must be included in robot_sets yaml").to_vec() };
        let l = robots_by_filenames_vec.len();
        for i in 0..l {
            let robot_name = robots_by_filenames_vec[i]["robot_name"].as_str().expect("robot_name must be included in robots_by_filename in robots_sets yaml");

            let mut dead_end_link_names = Vec::new();
            let mut inactive_joint_names = Vec::new();
            let mut mobile_base_mode = "static".to_string();

            let dead_end_link_names_ = robots_by_filenames_vec[i]["dead_end_links"].as_vec().unwrap();
            let l = dead_end_link_names_.len();
            for j in 0..l {
                dead_end_link_names.push( dead_end_link_names_[j].as_str().unwrap().to_string() );
            }

            let inactive_joint_names_ = robots_by_filenames_vec[i]["inactive_joints"].as_vec().unwrap();
            let l = inactive_joint_names_.len();
            for j in 0..l {
                inactive_joint_names.push( inactive_joint_names_[j].as_str().unwrap().to_string() );
            }

            mobile_base_mode = robots_by_filenames_vec[i]["mobile_base_mode"].as_str().expect("mobile_base_mode must be included in robot configuration yaml.").to_string();

            let mut base_offset = ImplicitDualQuaternion::new_identity();

            let mut mobile_base_bounds_filename = if robots_by_filenames_vec[i]["mobile_base_bounds_filename"] == Null { None } else { Some(robots_by_filenames_vec[i]["mobile_base_bounds_filename"].as_str().expect("mobile_base_bounds_filename must be included in robot configuration yaml.").to_string()) };

            let mut base_posiition_offset_vec3 = Vector3::new(0.,0.,0.);
            let base_position_offset = robots_by_filenames_vec[i]["base_position_offset"].as_vec().unwrap();
            if base_position_offset.len() != 3 {
                return Err(format!("base_position_offset ({:?}) must be a float vec of length 3", base_position_offset));
            }
            for j in 0..3 {
                base_posiition_offset_vec3[j] = base_position_offset[j].as_f64().expect("base_position_offset has a value that is not f64.");
            }
            base_offset.translation = base_posiition_offset_vec3;
            base_offset.set_is_identity();

            let mut euler_angles = [0.,0.,0.];
            let base_orientation_offset = robots_by_filenames_vec[i]["base_orientation_offset"].as_vec().unwrap();
            if base_orientation_offset.len() != 3 {
                return Err(format!("base_orientation_offset ({:?}) must be a float vec of length 3", base_orientation_offset));
            }
            for j in 0..3 {
                euler_angles[j] = base_orientation_offset[j].as_f64().expect("base_orientation_offset has a value that is not f64.");
            }
            let mut q: UnitQuaternion<f64> = UnitQuaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
            base_offset.quat = q;
            base_offset.set_is_identity();

            let robot_configuration = RobotConfigurationModule::new_manual_inputs(robot_name, "manual", base_offset, dead_end_link_names, inactive_joint_names, mobile_base_mode, mobile_base_bounds_filename);

            self.add_robot_from_robot_configuration_module(&robot_configuration);
        }

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _set_robot_set_result_vector_idxs_to_robot_idxs(&mut self) {
        let mut out_vec = Vec::new();

        let l = self.get_num_robots();
        for i in 0..l {
            for j in 0..l {
                if i < j {
                    out_vec.push( (i, j) );
                }
            }
        }

        self._robot_set_result_vector_idxs_to_robot_idxs = out_vec;
    }

    pub fn get_robot_set_result_vector_idxs_to_robot_idxs_ref(&self) -> &Vec<(usize, usize)> {
        return &self._robot_set_result_vector_idxs_to_robot_idxs;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_iter(&self) -> Iter<Robot> {
        return self._robots.iter();
    }

    pub fn get_iter_mut(&mut self) -> IterMut<Robot> {
        return self._robots.iter_mut();
    }

    pub fn get_robots_ref(&self) -> &Vec<Robot> { return &self._robots; }

    pub fn get_robots_mut_ref(&mut self) -> &mut Vec<Robot> { return &mut self._robots; }

    pub fn get_dofs_per_robot_ref(&self) -> &Vec<usize> { return &self._dofs_per_robot; }

    pub fn get_total_num_dofs(&self) -> usize { return self._total_num_dofs; }

    pub fn get_num_robots(&self) -> usize { return self._num_robots; }
}


