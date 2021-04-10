extern crate lynx_lib;
use lynx_lib::robot_modules::robot_configuration_module::RobotConfigurationModule;
use lynx_lib::robot_modules::robot_fk_module::RobotFKModule;
use lynx_lib::robot_modules::robot_collision_detection_module::RobotCollisionDetectionModule;
use lynx_lib::robot_modules::robot_dof_module::RobotDOFModule;
use lynx_lib::utils::utils_files_and_strings::file_utils::*;
use lynx_lib::utils::utils_parsing::yaml_parsing_utils::*;
use std::time::Instant;
use lynx_lib::utils::utils_files_and_strings::robot_folder_utils::*;
use lynx_lib::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use lynx_lib::utils::utils_collisions::oriented_bounding_box::*;
use lynx_lib::utils::utils_math::implicit_dual_quaternion::ImplicitDualQuaternion;
use lynx_lib::robot_modules::robot_bounds_module::RobotBoundsModule;
use nalgebra::{Vector3};

fn main () {

    let mut r = RobotConfigurationModule::_new("sawyer".to_string(), "test".to_string());
    let mut d = RobotDOFModule::new(&r);
    let mut fk = RobotFKModule::new(&r, &d);
    let mut b = RobotBoundsModule::new(&r, &d, None);

    let mut c = RobotCollisionDetectionModule::new(&r, &d, &fk, &b);

    println!("{:?}", c._collision_node_idx_to_node_name);

    let leaf_layer = c.robot_collision_tree.get_leaf_layer_idx();
}