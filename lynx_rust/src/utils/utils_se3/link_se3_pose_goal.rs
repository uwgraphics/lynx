// PRE-DEPRECATION

/*

use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use crate::utils::utils_se3::transformation_utils::*;
use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use termion::{style, color};
use nalgebra::{Vector3};

#[derive(Debug, Clone)]
pub struct LinkSE3PoseGoal {
    _link_idx: usize,
    _link_name: String,
    _se3_pose_goal: ImplicitDualQuaternion
}

impl LinkSE3PoseGoal {
    pub fn new(robot_configuration_module: &RobotConfigurationModule, link_idx: usize, se3_pose_goal: ImplicitDualQuaternion) -> Result<Self, String> {
        if link_idx >= robot_configuration_module.robot_model_module.links.len() {
            return Err( format!("link idx {:?} is too high for robot (num links is {:?})", link_idx, robot_configuration_module.robot_model_module.links.len()) );
        }

        let link_name = robot_configuration_module.robot_model_module.links[link_idx].name.clone();

        let is_active = robot_configuration_module.is_link_active(&link_name);
        if !(is_active.unwrap()) {
            return Err(format!("link {:?} is not active, cannot be a link pose goal", link_name));
        }

        return Ok( Self { _link_idx: link_idx, _link_name: link_name,_se3_pose_goal: se3_pose_goal });
    }

    pub fn new_from_link_name(robot_configuration_module: &RobotConfigurationModule, link_name: &String, se3_pose_goal: ImplicitDualQuaternion) -> Result<Self, String> {
        let link_idx = robot_configuration_module.robot_model_module.get_link_idx_from_name(link_name);
        if link_idx.is_none() {
            return Err(format!("link name {:?} not found for robot {:?}", link_name, robot_configuration_module.robot_model_module.robot_name));
        }

        let is_active = robot_configuration_module.is_link_active(link_name);
        if !(is_active.unwrap()) {
            return Err(format!("link {:?} is not active, cannot be a link pose goal", link_name));
        }

        return Self::new(robot_configuration_module, link_idx.unwrap(), se3_pose_goal);
    }

    pub fn new_relative_to_fk_result(robot_configuration_module: &RobotConfigurationModule, fk_res: &Vec<Option<ImplicitDualQuaternion>>, link_idx: usize, world_space_se3_offset: Option<ImplicitDualQuaternion>) -> Result<Self, String> {
        if fk_res[link_idx].is_none() { return Err(format!("link {:?} is None in fk_res", link_idx)); }

        let mut se3_pose_goal = fk_res[link_idx].as_ref().unwrap().clone();
        if world_space_se3_offset.is_some() {
            se3_pose_goal = world_space_se3_offset.unwrap().multiply(&se3_pose_goal);
        }

        return Self::new(robot_configuration_module, link_idx, se3_pose_goal);
    }

    pub fn new_relative_to_fk_result_and_link_name(robot_configuration_module: &RobotConfigurationModule, fk_res: &Vec<Option<ImplicitDualQuaternion>>, link_name: &String, world_space_se3_offset: Option<ImplicitDualQuaternion>) -> Result<Self, String> {
        let link_idx = robot_configuration_module.robot_model_module.get_link_idx_from_name(link_name);
        if link_idx.is_none() {
            return Err(format!("link name {:?} not found for robot {:?}", link_name, robot_configuration_module.robot_model_module.robot_name));
        }

        if fk_res[link_idx.unwrap()].is_none() { return Err(format!("link {:?} is None in fk_res", link_idx)); }

        let mut se3_pose_goal = fk_res[link_idx.unwrap()].as_ref().unwrap().clone();
        if world_space_se3_offset.is_some() {
            se3_pose_goal = world_space_se3_offset.unwrap().multiply(&se3_pose_goal);
        }

        return Self::new(robot_configuration_module, link_idx.unwrap(), se3_pose_goal);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_translation_and_rotation_errors(&self, se3_pose: &ImplicitDualQuaternion) -> (f64, f64) {
        /* in meters and radians */
        let disp = self.get_disp(se3_pose);
        return (2.0*disp.1.norm(), 2.0*disp.0.norm());
    }

    pub fn get_disp(&self, se3_pose: &ImplicitDualQuaternion) -> (Vector3<f64>, Vector3<f64>) {
        return implicit_dual_quaternion_disp(&self._se3_pose_goal, se3_pose);
    }

    pub fn get_disp_l2_magnitude(&self, se3_pose: &ImplicitDualQuaternion) -> f64 {
        return implicit_dual_quaternion_disp_l2_magnitude(&self._se3_pose_goal, se3_pose);
    }

    pub fn print_summary(&self) {
        println!("{}{}link idx ---> {:?} {}", color::Fg(color::Blue), style::Bold, self._link_idx, style::Reset);
        println!("{}{}se3 pose goal ---> {:?} {}", color::Fg(color::Blue), style::Bold, self._se3_pose_goal, style::Reset);
    }
}

*/