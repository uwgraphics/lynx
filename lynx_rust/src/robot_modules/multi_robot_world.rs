use crate::robot_modules::multi_robot_module_toolbox::*;
use crate::utils::utils_collisions::collision_environment::CollisionEnvironment;
use crate::utils::utils_collisions::collision_object_utils::*;
use crate::robot_modules::robot_fk_module::RobotFKResult;
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

    pub fn self_intersect_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfIntersectCheckMultipleResult, String> {
        let mut out_vec = VectorOfIntersectCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().self_intersect_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_distance_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfDistanceCheckMultipleResult, String> {
        let mut out_vec = VectorOfDistanceCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().self_distance_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_contact_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VectorOfContactCheckMultipleResult, String> {
        let mut out_vec = VectorOfContactCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().self_contact_check(&fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected, margin)?;

            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_intersect_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfIntersectCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        let mut out_vec = VectorOfIntersectCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_intersect_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_distance_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfDistanceCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        let mut out_vec = VectorOfDistanceCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_distance_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected)?;

            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }
        return Ok(out_vec);
    }

    pub fn self_contact_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VectorOfContactCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        let mut out_vec = VectorOfContactCheckMultipleResult::new_empty();
        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .self_contact_check_subset(&subset_check_idxs[i], &fk_res.get_robot_fk_results_ref()[i], link_geometry_type.clone(), stop_at_first_detected, margin)?;

            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }
        return Ok(out_vec);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn environment_intersect_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfIntersectCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VectorOfIntersectCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VectorOfIntersectCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_intersect_check(&fk_res.get_robot_fk_results_ref()[i],
                                             link_geometry_type.clone(),
                                             &self._collision_environment.as_ref().unwrap(),
                                             stop_at_first_detected)?;
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_distance_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfDistanceCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VectorOfDistanceCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VectorOfDistanceCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_distance_check(&fk_res.get_robot_fk_results_ref()[i],
                                             link_geometry_type.clone(),
                                             &self._collision_environment.as_ref().unwrap(),
                                             stop_at_first_detected)?;
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_contact_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VectorOfContactCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VectorOfContactCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected, margin));
        }
        let mut out_vec = VectorOfContactCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots();
        for i in 0..l {
            let res = self._multi_robot_module_toolbox
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_contact_check(&fk_res.get_robot_fk_results_ref()[i],
                                            link_geometry_type.clone(),
                                            &self._collision_environment.as_ref().unwrap(),
                                            stop_at_first_detected, margin)?;
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_intersect_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfIntersectCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VectorOfIntersectCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VectorOfIntersectCheckMultipleResult::new_empty();

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
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_distance_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfDistanceCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VectorOfDistanceCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VectorOfDistanceCheckMultipleResult::new_empty();

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
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_contact_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VectorOfContactCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_module_toolbox.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._multi_robot_module_toolbox.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VectorOfContactCheckMultipleResult::new_no_intersections_found(self._multi_robot_module_toolbox.get_num_robots(), stop_at_first_detected, margin));
        }
        let mut out_vec = VectorOfContactCheckMultipleResult::new_empty();

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
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res);
            }
        }

        return Ok(out_vec);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn multi_robot_intersect_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfIntersectCheckMultipleResult, String> {
        let mut out_vec = VectorOfIntersectCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = intersect_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, None)?;

                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_distance_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfDistanceCheckMultipleResult, String> {
        let mut out_vec = VectorOfDistanceCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = distance_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, None, None)?;

                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_contact_check(&mut self, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VectorOfContactCheckMultipleResult, String> {
        let mut out_vec = VectorOfContactCheckMultipleResult::new_empty();

        let l = self._multi_robot_module_toolbox.get_num_robots().clone();
        for i in 0..l {
            self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[i].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[i], &link_geometry_type);
            for j in 0..l {
                if i < j {
                    self._multi_robot_module_toolbox.get_robot_module_toolboxes_mut_ref()[j].get_core_collision_module_mut_ref().set_poses_on_links(&fk_res.get_robot_fk_results_ref()[j], &link_geometry_type);

                    let collision_objects1 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[i].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);
                    let collision_objects2 = self._multi_robot_module_toolbox.get_robot_module_toolboxes_ref()[j].get_core_collision_module_ref().get_link_geometry_collision_objects_ref(&link_geometry_type);

                    let res = contact_check_between_multiple_collision_objects(collision_objects1, collision_objects2, stop_at_first_detected, margin, None, None)?;

                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_intersect_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfIntersectCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._multi_robot_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VectorOfIntersectCheckMultipleResult::new_empty();

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

                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_distance_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VectorOfDistanceCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._multi_robot_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VectorOfDistanceCheckMultipleResult::new_empty();

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

                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res);
                    }
                }
            }
        }
        return Ok(out_vec);
    }

    pub fn multi_robot_contact_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VectorOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VectorOfContactCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._multi_robot_result_vector_idxs_to_robot_idxs.len() {
            return Err(format!("number of subset_check_idxs {:?} must equal length of multi_robot_result_vector_idxs {:?}", subset_check_idxs.len(), self._multi_robot_result_vector_idxs_to_robot_idxs.len()));
        }

        let mut out_vec = VectorOfContactCheckMultipleResult::new_empty();

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

                    if res.is_in_collision() && stop_at_first_detected {
                        out_vec.add_new_result(res);
                        return Ok(out_vec);
                    } else {
                        out_vec.add_new_result(res);
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

    /*
    pub fn get_robot_module_toolbox_mut_ref(&mut self) -> &mut RobotModuleToolbox {
        return &mut self._robot_module_toolbox;
    }

    pub fn get_robot_module_toolbox_ref(&self) -> &RobotModuleToolbox {
        return &self._robot_module_toolbox;
    }

    pub fn get_collision_environment_option_mut_ref(&mut self) -> &mut Option<CollisionEnvironment> {
        return &mut self._collision_environment;
    }

    pub fn get_collision_environment_option_ref(&self) -> &Option<CollisionEnvironment> {
        return &self._collision_environment;
    }

     */
}



#[derive(Debug, Clone)]
pub struct VectorOfIntersectCheckMultipleResult {
    _intersect_check_multiple_results: Vec<IntersectCheckMultipleResult>
}
impl VectorOfIntersectCheckMultipleResult {
    pub fn new_empty() -> Self {
        return Self { _intersect_check_multiple_results: Vec::new() }
    }

    pub fn new_no_intersections_found(num: usize, stopped_at_first_detected: bool) -> Self {
        let mut out_vec = Vec::new();
        for _ in 0..num {
            out_vec.push( IntersectCheckMultipleResult::NoIntersectionsFound(IntersectionCheckMultipleInfo::new(stopped_at_first_detected)) );
        }
        return Self { _intersect_check_multiple_results: out_vec }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_new_result(&mut self, result: IntersectCheckMultipleResult) {
        self._intersect_check_multiple_results.push(result);
    }

    pub fn in_collision(&self) -> bool {
        let mut in_collision = false;
        let l = self._intersect_check_multiple_results.len();
        for i in 0..l {
            match self._intersect_check_multiple_results[i] {
                IntersectCheckMultipleResult::IntersectionFound(_) => { return true; }
                IntersectCheckMultipleResult::NoIntersectionsFound(_) => {}
            }
        }

        return in_collision;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_intersect_check_multiple_results_ref(&self) -> &Vec<IntersectCheckMultipleResult> {
        return &self._intersect_check_multiple_results;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let mut count = 0;
        for r in &self._intersect_check_multiple_results {
            println!("{}{}Result {:?} ---> {}", style::Bold, color::Fg(color::Magenta), count, style::Reset);
            r.print_summary();
            count += 1;
        }
    }
}


#[derive(Debug, Clone)]
pub struct VectorOfDistanceCheckMultipleResult {
    _distance_check_multiple_results: Vec<DistanceCheckMultipleResult>
}
impl VectorOfDistanceCheckMultipleResult {
    pub fn new_empty() -> Self {
        return Self { _distance_check_multiple_results: Vec::new() }
    }

    pub fn new_no_intersections_found(num: usize, stopped_at_first_detected: bool) -> Self {
        let mut out_vec = Vec::new();
        for _ in 0..num {
            out_vec.push( DistanceCheckMultipleResult::NoIntersectionsFound(DistanceCheckMultipleInfo::new(stopped_at_first_detected)) );
        }
        return Self { _distance_check_multiple_results: out_vec }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_new_result(&mut self, result: DistanceCheckMultipleResult) {
        self._distance_check_multiple_results.push(result);
    }

    pub fn in_collision(&self) -> bool {
        let mut in_collision = false;
        let l = self._distance_check_multiple_results.len();
        for i in 0..l {
            match self._distance_check_multiple_results[i] {
                DistanceCheckMultipleResult::IntersectionFound(_) => { return true; }
                DistanceCheckMultipleResult::NoIntersectionsFound(_) => {}
            }
        }

        return in_collision;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_distance_check_multiple_results_ref(&self) -> &Vec<DistanceCheckMultipleResult> {
        return &self._distance_check_multiple_results;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let mut count = 0;
        for r in &self._distance_check_multiple_results {
            println!("{}{}Result {:?} ---> {}", style::Bold, color::Fg(color::Magenta), count, style::Reset);
            r.print_summary();
            count += 1;
        }
    }

}


#[derive(Debug, Clone)]
pub struct VectorOfContactCheckMultipleResult {
    _contact_check_multiple_results: Vec<ContactCheckMultipleResult>
}
impl VectorOfContactCheckMultipleResult {
    pub fn new_empty() -> Self {
        return Self { _contact_check_multiple_results: Vec::new() }
    }

    pub fn new_no_intersections_found(num: usize, stopped_at_first_detected: bool, margin: Option<f64>) -> Self {
        let mut out_vec = Vec::new();
        for _ in 0..num {
            out_vec.push( ContactCheckMultipleResult::NoIntersectionsFound(ContactCheckMultipleInfo::new(stopped_at_first_detected, margin)) );
        }
        return Self { _contact_check_multiple_results: out_vec }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_new_result(&mut self, result: ContactCheckMultipleResult) {
        self._contact_check_multiple_results.push(result);
    }

    pub fn in_collision(&self) -> bool {
        let mut in_collision = false;
        let l = self._contact_check_multiple_results.len();
        for i in 0..l {
            match self._contact_check_multiple_results[i] {
                ContactCheckMultipleResult::IntersectionFound(_) => { return true; }
                ContactCheckMultipleResult::NoIntersectionsFound(_) => {}
            }
        }

        return in_collision;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_distance_check_multiple_results_ref(&self) -> &Vec<ContactCheckMultipleResult> {
        return &self._contact_check_multiple_results;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let mut count = 0;
        for r in &self._contact_check_multiple_results {
            println!("{}{}Result {:?} ---> {}", style::Bold, color::Fg(color::Magenta), count, style::Reset);
            r.print_summary();
            count += 1;
        }
    }
}