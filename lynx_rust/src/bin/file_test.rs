extern crate lynx_lib;

use lynx_lib::utils::utils_files_and_strings::file_utils::*;
use lynx_lib::utils::utils_files_and_strings::robot_folder_utils::*;
use lynx_lib::utils::utils_parsing::urdf_parsing_utils::*;
use lynx_lib::utils::utils_preprocessing::preprocessing_utils::*;
use lynx_lib::robot_modules::robot_model_module::RobotModelModule;
use std::time::Instant;

fn main() {

    // decompose_all_links_into_convex_subcomponents_and_save_mesh_files("ur5".to_string(), 1);

    let r = RobotModelModule::new("ur5".to_string());

    // r.save_robot_model_module();

    // println!("{:?}", r.world_link_idx);
    /*
    println!("{:?}", r.links[1].name);
    println!("{:?}", r.links[1].preceding_link_idx);
    println!("{:?}", r.links[1].children_link_idxs);
    println!("{:?}", r.world_link_idx);
    println!("{:?}", r.links[r.world_link_idx].name);
    println!("{:?}", r.links[1].preceding_joint_idx);
    println!("{:?}", r.joints[4].name);

    println!("{:?}", r.joints[5].dof_rotation_axes);

    // get_all_urdf_links_from_robot_name("hubo".to_string());

    println!("{:?}", r.joints[5].num_dofs);
    */



}