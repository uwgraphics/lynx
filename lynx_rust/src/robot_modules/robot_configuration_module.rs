use crate::robot_modules::robot_model_module::RobotModelModule;
use crate::utils::utils_files_and_strings::robot_folder_utils::*;
use crate::utils::utils_files_and_strings::file_utils::*;
use crate::utils::utils_parsing::yaml_parsing_utils::*;
use serde::{Serialize, Deserialize};
use termion::{color, style};
use crate::robot_modules::{link::Link, joint::Joint};
use nalgebra::{Vector3};
use std::collections::HashMap;

/*
Notes:
Dead-end links: these links, and any links that are descendants of these links, will not be active
    and will not be included in any downstream computations (e.g., collision detection, etc)
Active joints: these joints are considered degrees-of-freedom and will be included in downstream computations.
    Any joints that are not active always default to values of 0.0, and thus, are not variables in any robot
    application.
Mobile base mode: string that specifies the mode of the base.  Options are:
    - static
    - floating
    - translation
    - rotation
    - planar_translation  (z assumed to be up)
    - planar_rotation    (z assumed to be up)
    - planar_translation_and_rotation    (z assumed to be up)
*/

#[derive(Clone, Serialize, Deserialize)]
pub struct  RobotConfigurationModule {
    pub configuration_name: String,
    pub robot_model_module: RobotModelModule,
    pub dead_end_link_names: Vec<String>,
    pub inactive_joint_names: Vec<String>,
    pub mobile_base_mode: String, // static, floating, planar_translation, planar_rotation, planar_translation_and_rotation
    pub all_inactive_links: Vec<String>
}

impl RobotConfigurationModule {
    pub fn new(robot_name: &str, configuration_name: Option<&str>) -> Result<Self, String> {
        let partial_fp0 = "configurations".to_string();

        let res = check_if_path_exists_relative_to_robot_directory(robot_name.to_string(), partial_fp0.clone());
        if !res {
            Self::_create_configuration_directory_with_example_if_need_be(robot_name.to_string());
        }

        if configuration_name.is_none() { return Ok(Self::new_base_configuration(robot_name)); }
        else { return Self::_new(robot_name, configuration_name.unwrap()); }
    }

    fn _new(robot_name: &str, configuration_name: &str) -> Result<Self, String> {
        let partial_fp1 = "configurations/".to_string() + configuration_name + ".yaml";
        let partial_fp2 = "autogenerated_metadata/configurations/".to_string() + configuration_name + ".json";

        let res = check_if_file1_modified_after_file2_relative_to_robot_directory(robot_name.to_string(), partial_fp1.clone(), partial_fp2.clone());
        if res.is_none() {
            let out_self = Self::_new_load_from_configuration_yaml(robot_name.clone(), configuration_name.clone())?;
            let exists = check_if_path_exists_relative_to_robot_directory(robot_name.to_string(), partial_fp1.clone());
            if exists {
                 out_self.save_robot_configuration_module();
            }
            return Ok(out_self);
        } else {
            if res.unwrap() {
                println!("{}{}I see that you've adjusted configuration {}.  Updating metadata for this configuration to reflect these changes.{}", color::Fg(color::Blue), style::Bold, configuration_name, style::Reset);
                let out_self = Self::_new_load_from_configuration_yaml(robot_name.clone(), configuration_name.clone())?;
                out_self.save_robot_configuration_module();
                return Ok(out_self);
            } else {
                let json_string = read_file_contents_relative_to_robot_directory(robot_name.to_string(), partial_fp2.clone());
                if json_string.is_none() {
                    let out_self = Self::_new_load_from_configuration_yaml(robot_name.clone(), configuration_name.clone())?;
                    out_self.save_robot_configuration_module();
                    return Ok(out_self);
                }

                let out_self: RobotConfigurationModule = serde_json::from_str(&json_string.unwrap()).unwrap();
                return Ok(out_self);
            }
        }
    }

    pub fn new_base_configuration(robot_name: &str) -> Self {
        let configuration_name = "baseconfig";
        let dead_end_link_names = Vec::new();
        let inactive_joint_names = Vec::new();
        Self::new_manual_inputs(robot_name, configuration_name, dead_end_link_names, inactive_joint_names, "static".to_string())
    }

    pub fn new_manual_inputs(robot_name: &str, configuration_name: &str, dead_end_link_names: Vec<String>, inactive_joint_names: Vec<String>, mobile_base_mode: String) -> Self {
        let robot_model_module = RobotModelModule::new(robot_name);
        let all_inactive_links = Vec::new();

        let mut out_self = Self { configuration_name: configuration_name.to_string(), robot_model_module,
            dead_end_link_names, inactive_joint_names, mobile_base_mode, all_inactive_links};

        out_self._adjust_model_module_based_on_mobile_base_mode();
        out_self._set_inactive_links();
        out_self._set_inactive_joints();
        out_self._set_link_tree_traversal_info();

        return out_self;
    }

    fn _new_load_from_configuration_yaml(robot_name: &str, configuration_name: &str) -> Result<Self, String> {
        let fp = get_path_to_particular_robot_directory(robot_name.to_string()) + "/configurations/" + configuration_name + ".yaml";

        let mut dead_end_link_names = Vec::new();
        let mut inactive_joint_names = Vec::new();
        let mut mobile_base_mode = "static".to_string();

        let y = get_yaml_obj(fp)?;
        if y.len() == 0 {
            println!("{}{}WARNING: {} is not a valid configuration name for robot {}  Defaulting to base configuration.{}", color::Fg(color::Yellow), style::Bold, configuration_name, robot_name, style::Reset);
            return Ok(Self::new_manual_inputs(robot_name, "baseconfig", dead_end_link_names, inactive_joint_names, mobile_base_mode));
        }

        let y1 = y[0].clone();
        let dead_end_link_names_ = y1["dead_end_links"].as_vec().unwrap();
        let l = dead_end_link_names_.len();
        for i in 0..l {
            dead_end_link_names.push( dead_end_link_names_[i].as_str().unwrap().to_string() );
        }

        let inactive_joint_names_ = y1["inactive_joints"].as_vec().unwrap();
        let l = inactive_joint_names_.len();
        for i in 0..l {
            inactive_joint_names.push( inactive_joint_names_[i].as_str().unwrap().to_string() );
        }

        mobile_base_mode = y1["mobile_base_mode"].as_str().unwrap().to_string();

        return Ok(Self::new_manual_inputs(robot_name, configuration_name, dead_end_link_names, inactive_joint_names, mobile_base_mode));
    }

    fn _create_configuration_directory_with_example_if_need_be(robot_name: String) {
        let mut out_string = "".to_string();

        out_string += "# this is a sample robot configuration file that would be called `sample_config' based on the filename.\n";
        out_string += "# this is a yaml file with the following format: \n\n";
        out_string += "# dead_end_links: [\"dead_end_link_1\"] \n";
        out_string += "# inactive_joints: [\"inactive_joint_1\", \"inactive_joint_2\"] \n";
        out_string += "# mobile_base_mode: \"static\" \n";
        out_string += "#    ^^(can be static, floating, translation, rotation, planar_translation, planar_rotation, planar_translation_and_rotation)\n\n";
        out_string += "dead_end_links: [] \n";
        out_string += "inactive_joints: [] \n";
        out_string += "mobile_base_mode: \"static\" ";

        let partial_fp = "configurations".to_string();
        write_string_to_file_relative_to_robot_directory(robot_name.clone(), partial_fp.clone(), "sample_config.yaml".to_string(), out_string, true);
    }

    fn _adjust_model_module_based_on_mobile_base_mode(&mut self) {
        if self.mobile_base_mode == "static".to_string() { return; }

        let num_links = self.robot_model_module.links.len();
        let num_joints = self.robot_model_module.joints.len();

        let new_link_idx = num_links;
        let new_joint_idx = num_joints;

        let world_link_idx = self.robot_model_module.world_link_idx;

        let mut new_link = Link::new_without_urdf_link( new_link_idx, None, vec![world_link_idx], None, vec![ new_joint_idx ] );
        new_link.name = "world_link_preceding_mobile_base_joint".to_string();
        let mut new_joint = Joint::new_without_urdf_joint( new_joint_idx, new_link_idx, world_link_idx );
        new_joint.name = "mobile_base_joint".to_string();

        if self.mobile_base_mode == "floating".to_string() {
            new_joint.dof_translation_axes.push( Vector3::new(1.0, 0.0, 0.0) );
            new_joint.dof_translation_axes.push( Vector3::new(0.0, 1.0, 0.0) );
            new_joint.dof_translation_axes.push( Vector3::new(0.0, 0.0, 1.0) );

            new_joint.dof_rotation_axes.push( Vector3::new(1.0, 0.0, 0.0) );
            new_joint.dof_rotation_axes.push( Vector3::new(0.0, 1.0, 0.0) );
            new_joint.dof_rotation_axes.push( Vector3::new(0.0, 0.0, 1.0) );
        } else if self.mobile_base_mode == "translation".to_string() {
            new_joint.dof_translation_axes.push( Vector3::new(1.0, 0.0, 0.0) );
            new_joint.dof_translation_axes.push( Vector3::new(0.0, 1.0, 0.0) );
            new_joint.dof_translation_axes.push( Vector3::new(0.0, 0.0, 1.0) );
        } else if self.mobile_base_mode == "rotation".to_string() {
            new_joint.dof_rotation_axes.push( Vector3::new(1.0, 0.0, 0.0) );
            new_joint.dof_rotation_axes.push( Vector3::new(0.0, 1.0, 0.0) );
            new_joint.dof_rotation_axes.push( Vector3::new(0.0, 0.0, 1.0) );
        } else if self.mobile_base_mode == "planar_translation".to_string() {
            new_joint.dof_translation_axes.push( Vector3::new(1.0, 0.0, 0.0) );
            new_joint.dof_translation_axes.push( Vector3::new(0.0, 1.0, 0.0) );
        } else if self.mobile_base_mode == "planar_rotation".to_string() {
            new_joint.dof_rotation_axes.push( Vector3::new(0.0, 0.0, 1.0) );
        } else if self.mobile_base_mode == "planar_translation_and_rotation".to_string() {
            new_joint.dof_translation_axes.push( Vector3::new(1.0, 0.0, 0.0) );
            new_joint.dof_translation_axes.push( Vector3::new(0.0, 1.0, 0.0) );
            new_joint.dof_rotation_axes.push( Vector3::new(0.0, 0.0, 1.0) );
        }

        new_joint.num_dofs = new_joint.dof_translation_axes.len() + new_joint.dof_rotation_axes.len();
        new_joint.set_dof_rotation_axes_as_units();

        self.robot_model_module.links.push( new_link );
        self.robot_model_module.joints.push( new_joint );
        self.robot_model_module.links[ world_link_idx ].preceding_link_idx = Some( new_link_idx );
        self.robot_model_module.links[ world_link_idx ].preceding_joint_idx = Some( new_joint_idx );
        self.robot_model_module.add_to_link_hashmap( &"world_link_preceding_mobile_base_joint".to_string(), new_link_idx );
        self.robot_model_module.add_to_joint_hashmap( &"mobile_base_joint".to_string(), new_joint_idx );
        self.robot_model_module.world_link_idx = new_link_idx;
    }

    fn _set_inactive_links(&mut self) {
        if self.dead_end_link_names.len() == 0 { return; }

        let l = self.robot_model_module.links.len();

        loop {
            let mut change_on_this_loop = false;

            for i in 0..l {
                if self.robot_model_module.links[i].active {
                    let preceding_link_idx = self.robot_model_module.links[i].preceding_link_idx.clone();

                    let has_preceding_link = preceding_link_idx.is_some();
                    let mut preceding_link_active = true;
                    if has_preceding_link {
                        preceding_link_active = self.robot_model_module.links[ preceding_link_idx.unwrap() ].active;
                    }
                    let is_dead_end_itself = self.dead_end_link_names.contains(&self.robot_model_module.links[i].name);

                    if (!preceding_link_active) || is_dead_end_itself {
                        self.robot_model_module.links[i].active = false;
                        self.all_inactive_links.push( self.robot_model_module.links[i].name.clone() );
                        change_on_this_loop = true;
                    }
                }
            }

            if !change_on_this_loop { return; }
        }

    }

    fn _set_inactive_joints(&mut self) {
        let l = self.robot_model_module.joints.len();

        for i in 0..l {
            let mut child_link_inactive = false;
            let child_link_idx = self.robot_model_module.joints[i].child_link_idx;
            if (!self.robot_model_module.links[child_link_idx].active) {
                child_link_inactive = true;
            }

            if child_link_inactive || self.inactive_joint_names.contains(&self.robot_model_module.joints[i].name) {
                self.robot_model_module.joints[i].active = false;
            } else {
                self.robot_model_module.joints[i].active = true;
            }
        }
    }

    fn _set_link_tree_traversal_info(&mut self) {
        self.robot_model_module.set_link_tree_traversal_info();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn is_link_active(&self, link_name: &String) -> Option<bool> {
        let idx_res = self.robot_model_module.get_link_idx_from_name(link_name);
        if idx_res.is_none() { return None; }

        if self.all_inactive_links.contains(link_name) {
            return Some(false);
        } else {
            return Some(true);
        }
    }

    pub fn save_robot_configuration_module(&self) {
        let serialized = serde_json::to_string(&self).unwrap();
        write_string_to_file_relative_to_robot_directory( self.robot_model_module.robot_name.clone(), "autogenerated_metadata/configurations".to_string(), format!("{}.json", self.configuration_name.clone()).to_string(), serialized, true );
    }
}
