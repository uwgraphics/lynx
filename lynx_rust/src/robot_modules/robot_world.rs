use crate::robot_modules::robot_set::*;
use crate::utils::utils_collisions::prelude::*;
use crate::robot_modules::robot_fk_module::*;
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;
use termion::{style, color};

#[derive(Clone, Debug)]
pub struct RobotWorld {
    _robot_set: RobotSet,
    _collision_environment: Option<CollisionEnvironment>
}

impl RobotWorld {
    pub fn new_from_set_name(robot_set_name: &str, environment_name: Option<&str>) -> Result<Self, String> {
        let _robot_set = RobotSet::new_from_set_name(robot_set_name)?;
        let mut _collision_environment = None;
        if environment_name.is_some() {
            _collision_environment = Some(CollisionEnvironment::new(environment_name.unwrap())?);
        }

        let mut out_self = Self { _robot_set, _collision_environment };

        return Ok(out_self);
    }

    pub fn new(robot_names: Vec<&str>, configuration_names: Vec<Option<&str>>, environment_name: Option<&str>) -> Result<Self, String> {
        let _robot_set = RobotSet::new(robot_names, configuration_names)?;
        let mut _collision_environment = None;
        if environment_name.is_some() {
            _collision_environment = Some(CollisionEnvironment::new(environment_name.unwrap())?);
        }

        let mut out_self = Self { _robot_set, _collision_environment };

        return Ok(out_self);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn environment_intersect_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VecOfIntersectCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_intersect_check(&fk_res.get_robot_fk_results_ref()[i],
                                             link_geometry_type.clone(),
                                             &self._collision_environment.as_ref().unwrap(),
                                             stop_at_first_detected)?;

            let label = format!("environment intersect check on robot {} ({:?})", i, self._robot_set.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
            return Ok(VecOfDistanceCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_distance_check(&fk_res.get_robot_fk_results_ref()[i],
                                             link_geometry_type.clone(),
                                             &self._collision_environment.as_ref().unwrap(),
                                             stop_at_first_detected)?;

            let label = format!("environment distance check on robot {} ({:?})", i, self._robot_set.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
            return Ok(VecOfContactCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected, margin));
        }
        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_contact_check(&fk_res.get_robot_fk_results_ref()[i],
                                            link_geometry_type.clone(),
                                            &self._collision_environment.as_ref().unwrap(),
                                            stop_at_first_detected, margin)?;

            let label = format!("environment contact check on robot {} ({:?})", i, self._robot_set.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._robot_set.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._robot_set.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfIntersectCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_intersect_check_subset(&subset_check_idxs[i],
                                                  &fk_res.get_robot_fk_results_ref()[i],
                                                  link_geometry_type.clone(),
                                                  &self._collision_environment.as_ref().unwrap(),
                                                  stop_at_first_detected)?;

            let label = format!("environment intersect subset check on robot {} ({:?})", i, self._robot_set.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._robot_set.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._robot_set.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfDistanceCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_distance_check_subset(&subset_check_idxs[i],
                                                    &fk_res.get_robot_fk_results_ref()[i],
                                                    link_geometry_type.clone(),
                                                    &self._collision_environment.as_ref().unwrap(),
                                                    stop_at_first_detected)?;

            let label = format!("environment distance check subset on robot {} ({:?})", i, self._robot_set.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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
        if subset_check_idxs.len() != self._robot_set.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._robot_set.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfContactCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected, margin));
        }
        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robot_module_toolboxes_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_contact_check_subset(&subset_check_idxs[i],
                                                  &fk_res.get_robot_fk_results_ref()[i],
                                                  link_geometry_type.clone(),
                                                  &self._collision_environment.as_ref().unwrap(),
                                           stop_at_first_detected, margin)?;

            let label = format!("environment contact check subset on robot {} ({:?})", i, self._robot_set.get_robot_module_toolboxes_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
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

    pub fn update_collision_environment(&mut self, environment_name: Option<&str>) -> Result<(), String> {
        self._collision_environment = None;
        if environment_name.is_some() {
            self._collision_environment = Some(CollisionEnvironment::new(environment_name.unwrap())?);
        }
        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_robot_set_mut_ref(&mut self) -> &mut RobotSet {
        return &mut self._robot_set;
    }

    pub fn get_robot_set_ref(&self) -> &RobotSet {
        return &self._robot_set;
    }

    pub fn get_collision_environment_option_mut_ref(&mut self) -> &mut Option<CollisionEnvironment> {
        return &mut self._collision_environment;
    }

    pub fn get_collision_environment_option_ref(&self) -> &Option<CollisionEnvironment> {
        return &self._collision_environment;
    }
}
