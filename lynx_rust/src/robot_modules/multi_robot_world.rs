use crate::robot_modules::multi_robot_module_toolbox::*;
use crate::utils::utils_collisions::prelude::*;
use crate::robot_modules::robot_fk_module::*;
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;
use termion::{style, color};

#[derive(Clone, Debug)]
pub struct MultiRobotWorld {
    _multi_robot_module_toolbox: MultiRobotModuleToolbox,
    _collision_environment: Option<CollisionEnvironment>,
    _multi_robot_result_vector_idxs_to_robot_idxs: Vec<(usize, usize)>
}

impl MultiRobotWorld {
    pub fn new(robot_names: Vec<&str>, configuration_names: Vec<Option<&str>>, mobile_base_bounds_filenames: Vec<Option<&str>>, environment_name: Option<&str>) -> Result<Self, String> {
        let _multi_robot_module_toolbox = MultiRobotModuleToolbox::new(robot_names, configuration_names, mobile_base_bounds_filenames)?;
        let mut _collision_environment = None;
        if environment_name.is_some() {
            _collision_environment = Some(CollisionEnvironment::new(environment_name.unwrap())?);
        }

        let _multi_robot_result_vector_idxs_to_robot_idxs = Vec::new();

        let mut out_self = Self { _multi_robot_module_toolbox, _collision_environment, _multi_robot_result_vector_idxs_to_robot_idxs};

        out_self._set_multi_robot_result_vector_idxs_to_robot_idxs();

        return Ok(out_self);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn self_intersect_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().self_intersect_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            let label = format!("self intersect check on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().self_distance_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            let label = format!("self distance check on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().self_contact_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected, margin)?;

            let label = format!("self contact check on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_intersect_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            let label = format!("self intersect subset check on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_distance_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            let label = format!("self distance check subset on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_contact_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected, margin)?;

            let label = format!("self contact check subset on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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

    pub fn environment_intersect_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VecOfIntersectCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_intersect_check(&fk_res.get_robot_fk_results_ref()[i],
                                             link_geometry_type.clone(),
                                             &self._collision_environment.as_ref().unwrap(),
                                             stop_at_first_detected)?;

            let label = format!("environment intersect check on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_distance_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfDistanceCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VecOfDistanceCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_distance_check(&fk_res.get_robot_fk_results_ref()[i],
                                             link_geometry_type.clone(),
                                             &self._collision_environment.as_ref().unwrap(),
                                             stop_at_first_detected)?;

            let label = format!("environment distance check on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_contact_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VecOfContactCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VecOfContactCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected, margin));
        }
        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_contact_check(&fk_res.get_robot_fk_results_ref()[i],
                                            link_geometry_type.clone(),
                                            &self._collision_environment.as_ref().unwrap(),
                                            stop_at_first_detected, margin)?;

            let label = format!("environment contact check on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_intersect_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfIntersectCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_intersect_check_subset(&subset_check_idxs[i],
                                                  &fk_res.get_robot_fk_results_ref()[i],
                                                  link_geometry_type.clone(),
                                                  &self._collision_environment.as_ref().unwrap(),
                                                  stop_at_first_detected)?;

            let label = format!("environment intersect subset check on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_distance_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfDistanceCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfDistanceCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_distance_check_subset(&subset_check_idxs[i],
                                                    &fk_res.get_robot_fk_results_ref()[i],
                                                    link_geometry_type.clone(),
                                                    &self._collision_environment.as_ref().unwrap(),
                                                    stop_at_first_detected)?;

            let label = format!("environment distance check subset on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_contact_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VecOfContactCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfContactCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected, margin));
        }
        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_contact_check_subset(&subset_check_idxs[i],
                                                  &fk_res.get_robot_fk_results_ref()[i],
                                                  link_geometry_type.clone(),
                                                  &self._collision_environment.as_ref().unwrap(),
                                           stop_at_first_detected, margin)?;

            let label = format!("environment contact check subset on robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = intersect_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, None)?;

                    let label = format!("multi robot intersect check between robot {} ({:?}) and robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
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

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = distance_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, None, None)?;

                    let label = format!("multi robot distance check between robot {} ({:?}) and robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
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

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = contact_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, margin, None, None)?;


                    let label = format!("multi robot contact check between robot {} ({:?}) and robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._multi_robot_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._multi_robot_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let mut count = 0;

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = intersect_check_between_multiple_collision_objects_subset(&subset_check_idxs[count], collision_objects1, collision_objects2, stop_at_first_detected, None)?;
                    count += 1;

                    let label = format!("multi robot intersect subset check between robot {} ({:?}) and robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._multi_robot_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._multi_robot_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let mut count = 0;

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = distance_check_between_multiple_collision_objects_subset(&subset_check_idxs[count], collision_objects1, collision_objects2, stop_at_first_detected, None, None)?;
                    count += 1;

                    let label = format!("multi robot distance check subset between robot {} ({:?}) and robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._multi_robot_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._multi_robot_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let mut count = 0;

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = contact_check_between_multiple_collision_objects_subset(&subset_check_idxs[count], collision_objects1, collision_objects2, stop_at_first_detected, margin, None, None)?;
                    count += 1;

                    let label = format!("multi robot contact subset check between robot {} ({:?}) and robot {} ({:?})", i, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone(),
                                        j, self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_configuration_module_ref().robot_model_module.robot_name.clone());
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

    fn _set_multi_robot_result_vector_idxs_to_robot_idxs(&mut self) {
        let mut out_vec = Vec::new();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            for j in 0..l {
                if i < j {
                    out_vec.push( (i, j) );
                }
            }
        }

        self._multi_robot_result_vector_idxs_to_robot_idxs = out_vec;
    }

    pub fn get_multi_robot_result_vector_idxs_to_robot_idxs_ref(&self) -> &Vec<(usize, usize)> {
        return &self._multi_robot_result_vector_idxs_to_robot_idxs;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn update_collision_environment(&mut self, environment_name: Option<&str>) -> Result<(), String> {
        self._collision_environment = None;
        if environment_name.is_some() {
            self._collision_environment = Some(CollisionEnvironment::new(environment_name.unwrap())?);
        }
        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_multi_robot_module_toolbox_mut_ref(&mut self) -> &mut MultiRobotModuleToolbox {
        return &mut self._multi_robot_module_toolbox;
    }

    pub fn get_multi_robot_module_toolbox_ref(&self) -> &MultiRobotModuleToolbox {
        return &self._multi_robot_module_toolbox;
    }

    pub fn get_collision_environment_option_mut_ref(&mut self) -> &mut Option<CollisionEnvironment> {
        return &mut self._collision_environment;
    }

    pub fn get_collision_environment_option_ref(&self) -> &Option<CollisionEnvironment> {
        return &self._collision_environment;
    }
}
