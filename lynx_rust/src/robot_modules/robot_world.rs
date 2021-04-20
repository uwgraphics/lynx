use crate::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use crate::utils::utils_collisions::prelude::*;
use crate::robot_modules::robot_fk_module::RobotFKResult;
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;

#[derive(Clone, Debug)]
pub struct RobotWorld {
    _robot_module_toolbox: RobotModuleToolbox,
    _collision_environment: Option<CollisionEnvironment>
}

impl RobotWorld {
    pub fn new(robot_name: &str, configuration_name: Option<&str>, mobile_base_bounds_filename: Option<&str>, environment_name: Option<&str>) -> Result<Self, String> {
        let _robot_module_toolbox = RobotModuleToolbox::new_lite(robot_name, configuration_name, mobile_base_bounds_filename)?;
        let mut _collision_environment = None;
        if environment_name.is_some() {
            _collision_environment = Some(CollisionEnvironment::new(environment_name.unwrap())?);
        }

        return Ok(Self { _robot_module_toolbox, _collision_environment } );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn environment_intersect_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<IntersectCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(IntersectCheckMultipleResult::NoIntersectionsFound(IntersectionCheckMultipleInfo::new(stop_at_first_detected)));
        }
        return self._robot_module_toolbox.get_core_collision_module_mut_ref().environment_intersect_check(fk_res, link_geometry_type, &self._collision_environment.as_ref().unwrap(), stop_at_first_detected);
    }

    pub fn environment_distance_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<DistanceCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(DistanceCheckMultipleResult::NoIntersectionsFound(DistanceCheckMultipleInfo::new(stop_at_first_detected)));
        }
        return self._robot_module_toolbox.get_core_collision_module_mut_ref().environment_distance_check(fk_res, link_geometry_type, &self._collision_environment.as_ref().unwrap(), stop_at_first_detected);
    }

    pub fn environment_contact_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<ContactCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(ContactCheckMultipleResult::NoIntersectionsFound(ContactCheckMultipleInfo::new(stop_at_first_detected, margin)));
        }
        return self._robot_module_toolbox.get_core_collision_module_mut_ref().environment_contact_check(fk_res, link_geometry_type, &self._collision_environment.as_ref().unwrap(), stop_at_first_detected, margin);
    }

    pub fn environment_contact_check_subset(&mut self, subset_check_idxs: &Vec<[[usize; 2]; 2]>, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<ContactCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(ContactCheckMultipleResult::NoIntersectionsFound(ContactCheckMultipleInfo::new(stop_at_first_detected, margin)));
        }
        return self._robot_module_toolbox.get_core_collision_module_mut_ref().environment_contact_check_subset(subset_check_idxs, fk_res, link_geometry_type, &self._collision_environment.as_ref().unwrap(), stop_at_first_detected, margin);
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
}