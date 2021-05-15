use urdf_rs::*;
use urdf_rs::utils::{rospack_find, expand_package_path};
use crate::utils::utils_files_and_strings::robot_folder_utils::*;
use termion::{color, style};
use std::fs;
use crate::utils::utils_files_and_strings::file_utils::*;
use crate::utils::utils_parsing::urdf_link::*;
use crate::utils::utils_parsing::urdf_joint::*;


pub fn get_all_urdf_links_from_robot_name(robot_name: String) -> Vec<URDFLink> {
    let mut out_vec = Vec::new();

    let urdf_robo = get_urdf_parser_robot(robot_name.clone());
    if urdf_robo.is_none() { return out_vec; }

    let urdf_robo_u = urdf_robo.unwrap();

    let l = urdf_robo_u.links.len();

    for i in 0..l {
        out_vec.push( URDFLink::new_from_urdf_link( &robot_name, &urdf_robo_u.links[i] ) );
    }

    out_vec
}

pub fn get_all_urdf_joints_from_robot_name(robot_name: String) -> Vec<URDFJoint> {
    let mut out_vec = Vec::new();

    let urdf_robo = get_urdf_parser_robot(robot_name.clone());
    if urdf_robo.is_none() { return out_vec; }

    let urdf_robo_u = urdf_robo.unwrap();

    let l = urdf_robo_u.joints.len();

    for i in 0..l {
        out_vec.push( URDFJoint::new_from_urdf_joint( &urdf_robo_u.joints[i] ) );
    }

    out_vec
}

pub fn get_urdf_parser_robot(robot_name: String) -> Option<Robot> {
    let urdf_fp = get_path_to_urdf(robot_name.clone());
    if urdf_fp.is_none() { return None; }

    let urdf_robo = urdf_rs::read_file(urdf_fp.unwrap().as_str()).unwrap();
    return Some(urdf_robo);
}

pub fn get_all_visual_mesh_filepaths(robot_name: String) -> Vec<String> {
    let mut out_vec = Vec::new();

    let urdf_fp = get_path_to_urdf(robot_name.clone());
    if urdf_fp.is_none() { return out_vec; }

    let urdf_robo = urdf_rs::read_file(urdf_fp.unwrap().as_str()).unwrap();
    let links = urdf_robo.links.clone();
    let l = links.len();
    for i in 0..l {
        let v = links[i].visual.clone();
        if v.len() == 0 { continue; }
        else {
            let mesh = v[0].geometry.clone();
            let mut out_fp = "".to_string();
            match mesh {
                urdf_rs::Geometry::Mesh {filename, scale} => {
                    out_fp = filename.clone();
                },
                _ => {  }
            }


            if out_fp.contains("file:///") {
                out_fp.replace_range((0..7), "");
            }

            let res = expand_package_path(&out_fp.as_str(), Option::None);

            if !(res == "".to_string()) {
                out_vec.push(res);
            }
        }
    }

    if out_vec.len() == 0 {
        println!("{}{}WARNING: It looks like there weren't any visual meshes found for robot {}. {}", color::Fg(color::Yellow), style::Bold, robot_name.clone(), style::Reset);
    }

    out_vec
}

pub fn get_all_collision_mesh_filepaths(robot_name: String) -> Vec<String> {
    let mut out_vec = Vec::new();

    let urdf_fp = get_path_to_urdf(robot_name.clone());
    if urdf_fp.is_none() { return out_vec; }

    let urdf_robo = urdf_rs::read_file(urdf_fp.unwrap().as_str()).unwrap();
    let links = urdf_robo.links.clone();
    let l = links.len();
    for i in 0..l {
        let v = links[i].collision.clone();
        if v.len() == 0 { continue; }
        else {
            let mesh = v[0].geometry.clone();
            let mut out_fp = "".to_string();
            match mesh {
                urdf_rs::Geometry::Mesh {filename, scale} => {
                    out_fp = filename.clone();
                },
                _ => {  }
            }


            if out_fp.contains("file:///") {
                out_fp.replace_range((0..7), "");
            }

            let res = expand_package_path(&out_fp.as_str(), Option::None);

            if !(res == "".to_string()) {
                out_vec.push(res);
            }
        }
    }

    if out_vec.len() == 0 {
        println!("{}{}WARNING: It looks like there weren't any collision meshes found for robot {}. {}", color::Fg(color::Yellow), style::Bold, robot_name.clone(), style::Reset);
    }

    out_vec
}

pub fn copy_all_visual_mesh_filepaths_to_robot_directory(robot_name: String) {
    let destination_directory = get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/visual/";

    let all_visual_mesh_filepaths = get_all_visual_mesh_filepaths(robot_name.clone());
    let l = all_visual_mesh_filepaths.len();
    if l == 0 { return; }

    create_directories_recursively_relative_to_robot_directory(robot_name.clone(), "base_meshes/visual".to_string());
    for i in 0..l {
        let filename_res = recover_filename_from_full_path(all_visual_mesh_filepaths[i].clone());
        if filename_res.is_none() { continue; }

        let filename = filename_res.unwrap();
        let fp = destination_directory.clone() + filename.as_str();

        fs::copy(all_visual_mesh_filepaths[i].as_str(), fp.as_str());
    }

}

pub fn copy_all_collision_mesh_filepaths_to_robot_directory(robot_name: String) {
    let destination_directory = get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/collision/";

    let all_collision_mesh_filepaths = get_all_collision_mesh_filepaths(robot_name.clone());
    let l = all_collision_mesh_filepaths.len();
    if l == 0 { return; }

    create_directories_recursively_relative_to_robot_directory(robot_name.clone(), "base_meshes/collision".to_string());
    for i in 0..l {
        let filename_res = recover_filename_from_full_path(all_collision_mesh_filepaths[i].clone());
        if filename_res.is_none() { continue; }

        let filename = filename_res.unwrap();
        let fp = destination_directory.clone() + filename.as_str();

        fs::copy(all_collision_mesh_filepaths[i].as_str(), fp.as_str());
    }

}

pub fn recover_real_mesh_file_path_from_ros_style_reference(fp: String) -> String {
    let mut out_fp = fp.clone();
    if out_fp.contains("file:///") {
        out_fp.replace_range((0..7), "");
    }

    return expand_package_path(&out_fp.as_str(), Option::None);
}




