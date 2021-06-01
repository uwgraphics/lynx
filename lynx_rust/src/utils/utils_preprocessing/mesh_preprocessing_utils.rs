use crate::utils::utils_files_and_strings::file_utils::*;
use crate::utils::utils_parsing::urdf_parsing_utils::*;
use crate::utils::utils_parsing::{urdf_joint::*, urdf_link::*};
use crate::utils::utils_files_and_strings::robot_folder_utils::*;
use crate::utils::utils_collisions::triangle_mesh_engine::*;
use crate::utils::utils_files_and_strings::string_utils::*;
use termion::{color, style};
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use nalgebra::{Vector3};

////////////////////////////////////////////////////////////////////////////////////////////////////

fn _decompose_trimesh_into_convex_subcomponents_and_save_mesh_files(link_name: String, trimesh: &TriMeshEngine, out_fp: String, max_compoenents_per_link: usize) {
    let mut res = vec![ trimesh.compute_convex_hull() ];

    if max_compoenents_per_link > 1 {
        res = trimesh.split_into_convex_components(0.3, 0);
        if res.len() > max_compoenents_per_link {
            res = trimesh.split_into_convex_components(1.0, 0);
        }
        if res.len() > max_compoenents_per_link {
            res = trimesh.split_into_convex_components(5.0, 0);
        }
        if res.len() > max_compoenents_per_link {
            res = trimesh.split_into_convex_components(40.0, 0);
        }
        if res.len() > max_compoenents_per_link {
            res = vec![trimesh.compute_convex_hull()];
        }
    }

    let l = res.len();

    println!("{}{}Decomposed link {}.  Found {} convex subcomponents.  Saving mesh files. {}", color::Fg(color::Green), style::Bold, link_name.clone(), l, style::Reset);

    for i in 0..l {
        let stl_fp = out_fp.clone() + link_name.as_str() + "_" + usize_to_string(i).as_str() + ".stl";
        let obj_fp = out_fp.clone() + link_name.as_str() + "_" + usize_to_string(i).as_str() + ".obj";

        res[i].output_to_stl(stl_fp.clone());
        res[i].output_to_obj(obj_fp.clone());
    }
}

pub fn decompose_all_links_into_convex_subcomponents_and_save_mesh_files(robot_name: String, max_components_per_link: usize) -> Result<(), String> {
    let includes_base_meshes = robot_directory_includes_base_meshes(robot_name.clone());
    if !(includes_base_meshes) {
        copy_all_visual_mesh_filepaths_to_robot_directory(robot_name.clone());
        copy_all_collision_mesh_filepaths_to_robot_directory(robot_name.clone());
    }

    let includes_base_meshes = robot_directory_includes_base_meshes(robot_name.clone());
    if !(includes_base_meshes) {
        println!("{}{}WARNING: It looks like there weren't any meshes found for robot {}. Cannot decompose links into convex subcomponents.  {}", color::Fg(color::Red), style::Bold, robot_name.clone(), style::Reset);
        return Err(format!("It looks like there weren't any meshes found for robot {}. Cannot convert links into convex shapes. ", robot_name.clone()));
    }

    delete_directory_all_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_subcomponents/".to_string());
    create_directories_recursively_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_subcomponents".to_string());

    let out_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_convex_subcomponents/";
    let visual_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/visual/";
    let collision_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/collision/";

    let links = get_all_urdf_links_from_robot_name(robot_name.clone());

    let l = links.len();

    for i in 0..l {
        println!("{}{}Decomposing link {}.  {} of {}. {}", color::Fg(color::Blue), style::Bold, links[i].name, i+1, l, style::Reset);

        let has_visual = links[i].visual.len() > 0;
        let has_collision = links[i].collision.len() > 0;

        let mut decomposed = false;
        if has_collision {
            let has_mesh_file = links[i].collision[0].filename.is_some();
            if has_mesh_file {
                let fp = collision_fp.clone() + links[i].collision[0].filename.as_ref().unwrap();

                let mut trimesh = TriMeshEngine::new_from_path(fp.clone())?;
                let origin_xyz = links[i].collision[0].origin_xyz.clone();
                let origin_rpy = links[i].collision[0].origin_rpy.clone();
                if !(origin_xyz[0] == 0.0) || !(origin_xyz[1] == 0.0) || !(origin_xyz[2] == 0.0) || !(origin_rpy[0] == 0.0) || !(origin_rpy[1] == 0.0) || !(origin_rpy[2] == 0.0) {
                    trimesh.transform_vertices( &ImplicitDualQuaternion::new_from_euler_angles( origin_rpy[0], origin_rpy[1], origin_rpy[2], Vector3::new(origin_xyz[0], origin_xyz[1], origin_xyz[2])) );
                }
                let scale_xyz = links[i].collision[0].scale.clone();
                if scale_xyz.is_some() {
                    let scale_xyz_unwrap = scale_xyz.as_ref().unwrap();
                    trimesh.scale_vertices(scale_xyz_unwrap[0], scale_xyz_unwrap[1], scale_xyz_unwrap[2]);
                }
                let mesh_offset = &links[i].link_collision_mesh_offset;
                let mesh_scaling = &links[i].link_collision_mesh_scaling;
                if mesh_offset.is_some() {
                    trimesh.transform_vertices(&mesh_offset.as_ref().unwrap());
                }
                if mesh_scaling.is_some() {
                    let s = mesh_scaling.as_ref().unwrap();
                    trimesh.scale_vertices(s[0], s[1], s[2]);
                }

                _decompose_trimesh_into_convex_subcomponents_and_save_mesh_files(links[i].name.clone(), &trimesh, out_fp.clone(), max_components_per_link);
                decomposed = true;
            }
        }

        if has_visual && !decomposed {
            let has_mesh_file = links[i].visual[0].filename.is_some();

            if has_mesh_file {
                let fp = visual_fp.clone() + links[i].visual[0].filename.as_ref().unwrap();

                let mut trimesh = TriMeshEngine::new_from_path(fp.clone())?;
                let origin_xyz = links[i].visual[0].origin_xyz.clone();
                let origin_rpy = links[i].visual[0].origin_rpy.clone();
                if !(origin_xyz[0] == 0.0) || !(origin_xyz[1] == 0.0) || !(origin_xyz[2] == 0.0) || !(origin_rpy[0] == 0.0) || !(origin_rpy[1] == 0.0) || !(origin_rpy[2] == 0.0)  {
                    trimesh.transform_vertices( &ImplicitDualQuaternion::new_from_euler_angles( origin_rpy[0], origin_rpy[1], origin_rpy[2], Vector3::new(origin_xyz[0], origin_xyz[1], origin_xyz[2])) );
                }
                let scale_xyz = links[i].visual[0].scale.clone();
                if scale_xyz.is_some() {
                    let scale_xyz_unwrap = scale_xyz.as_ref().unwrap();
                    trimesh.scale_vertices(scale_xyz_unwrap[0], scale_xyz_unwrap[1], scale_xyz_unwrap[2]);
                }
                let mesh_offset = &links[i].link_visual_mesh_offset;
                let mesh_scaling = &links[i].link_visual_mesh_scaling;
                if mesh_offset.is_some() {
                    trimesh.transform_vertices(&mesh_offset.as_ref().unwrap());
                }
                if mesh_scaling.is_some() {
                    let s = mesh_scaling.as_ref().unwrap();
                    trimesh.scale_vertices(s[0], s[1], s[2]);
                }

                _decompose_trimesh_into_convex_subcomponents_and_save_mesh_files(links[i].name.clone(), &trimesh, out_fp.clone(), 15);
                decomposed = true;
            }
        }
    }

    Ok(())
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn _convert_trimesh_into_convex_shapes_and_save_mesh_files(link_name: String, trimesh: &TriMeshEngine, out_fp: String) {
    let res = trimesh.compute_convex_hull();

    println!("{}{}Converted link {} into convex shape.  Saving mesh file. {}", color::Fg(color::Green), style::Bold, link_name.clone(), style::Reset);
    let stl_fp = out_fp.clone() + link_name.as_str() + ".stl";
    let obj_fp = out_fp.clone() + link_name.as_str() + ".obj";

    res.output_to_stl(stl_fp.clone());
    res.output_to_obj(obj_fp.clone());
}

pub fn convert_all_links_into_convex_shapes_and_save_mesh_files(robot_name: String) -> Result<(), String> {
    let includes_base_meshes = robot_directory_includes_base_meshes(robot_name.clone());
    if !(includes_base_meshes) {
        copy_all_visual_mesh_filepaths_to_robot_directory(robot_name.clone());
        copy_all_collision_mesh_filepaths_to_robot_directory(robot_name.clone());
    }

    let includes_base_meshes = robot_directory_includes_base_meshes(robot_name.clone());
    if !(includes_base_meshes) {
        println!("{}{}WARNING: It looks like there weren't any meshes found for robot {}. Cannot convert links into convex shapes.  {}", color::Fg(color::Red), style::Bold, robot_name.clone(), style::Reset);
        return Err(format!("It looks like there weren't any meshes found for robot {}. Cannot convert links into convex shapes. ", robot_name.clone()));
    }

    delete_directory_all_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_shapes/".to_string());
    create_directories_recursively_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_shapes".to_string());

    let out_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_convex_shapes/";
    let visual_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/visual/";
    let collision_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/collision/";

    let links = get_all_urdf_links_from_robot_name(robot_name.clone());

    let l = links.len();

    for i in 0..l {
        println!("{}{}Attempting to convert link {} into convex shape.  {} of {}. {}", color::Fg(color::Blue), style::Bold, links[i].name, i+1, l, style::Reset);

        let has_visual = links[i].visual.len() > 0;
        let has_collision = links[i].collision.len() > 0;

        let mut decomposed = false;
        if has_collision {
            let has_mesh_file = links[i].collision[0].filename.is_some();
            if has_mesh_file {
                let fp = collision_fp.clone() + links[i].collision[0].filename.as_ref().unwrap();

                let mut trimesh = TriMeshEngine::new_from_path(fp.clone())?;
                let origin_xyz = links[i].collision[0].origin_xyz.clone();
                let origin_rpy = links[i].collision[0].origin_rpy.clone();
                if !(origin_xyz[0] == 0.0) || !(origin_xyz[1] == 0.0) || !(origin_xyz[2] == 0.0) || !(origin_rpy[0] == 0.0) || !(origin_rpy[1] == 0.0) || !(origin_rpy[2] == 0.0) {
                    trimesh.transform_vertices( &ImplicitDualQuaternion::new_from_euler_angles( origin_rpy[0], origin_rpy[1], origin_rpy[2], Vector3::new(origin_xyz[0], origin_xyz[1], origin_xyz[2])) );
                }
                let scale_xyz = links[i].collision[0].scale.clone();
                if scale_xyz.is_some() {
                    let scale_xyz_unwrap = scale_xyz.as_ref().unwrap();
                    trimesh.scale_vertices(scale_xyz_unwrap[0], scale_xyz_unwrap[1], scale_xyz_unwrap[2]);
                }
                let mesh_offset = &links[i].link_collision_mesh_offset;
                let mesh_scaling = &links[i].link_collision_mesh_scaling;
                if mesh_offset.is_some() {
                    trimesh.transform_vertices(&mesh_offset.as_ref().unwrap());
                }
                if mesh_scaling.is_some() {
                    let s = mesh_scaling.as_ref().unwrap();
                    trimesh.scale_vertices(s[0], s[1], s[2]);
                }

                _convert_trimesh_into_convex_shapes_and_save_mesh_files(links[i].name.clone(), &trimesh, out_fp.clone());
                decomposed = true;
            }
        }

        if has_visual && !decomposed {
            let has_mesh_file = links[i].visual[0].filename.is_some();

            if has_mesh_file {
                let fp = visual_fp.clone() + links[i].visual[0].filename.as_ref().unwrap();

                let mut trimesh = TriMeshEngine::new_from_path(fp.clone())?;
                let origin_xyz = links[i].visual[0].origin_xyz.clone();
                let origin_rpy = links[i].visual[0].origin_rpy.clone();
                if !(origin_xyz[0] == 0.0) || !(origin_xyz[1] == 0.0) || !(origin_xyz[2] == 0.0) || !(origin_rpy[0] == 0.0) || !(origin_rpy[1] == 0.0) || !(origin_rpy[2] == 0.0)  {
                    trimesh.transform_vertices( &ImplicitDualQuaternion::new_from_euler_angles( origin_rpy[0], origin_rpy[1], origin_rpy[2], Vector3::new(origin_xyz[0], origin_xyz[1], origin_xyz[2])) );
                }
                let scale_xyz = links[i].visual[0].scale.clone();
                if scale_xyz.is_some() {
                    let scale_xyz_unwrap = scale_xyz.as_ref().unwrap();
                    trimesh.scale_vertices(scale_xyz_unwrap[0], scale_xyz_unwrap[1], scale_xyz_unwrap[2]);
                }
                let mesh_offset = &links[i].link_visual_mesh_offset;
                let mesh_scaling = &links[i].link_visual_mesh_scaling;
                if mesh_offset.is_some() {
                    trimesh.transform_vertices(&mesh_offset.as_ref().unwrap());
                }
                if mesh_scaling.is_some() {
                    let s = mesh_scaling.as_ref().unwrap();
                    trimesh.scale_vertices(s[0], s[1], s[2]);
                }

                _convert_trimesh_into_convex_shapes_and_save_mesh_files(links[i].name.clone(), &trimesh, out_fp.clone());
                decomposed = true;
            }
        }
    }

    Ok(())
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn save_all_links_as_triangle_meshes(robot_name: String) -> Result<(), String> {
    let includes_base_meshes = robot_directory_includes_base_meshes(robot_name.clone());
    if !(includes_base_meshes) {
        copy_all_visual_mesh_filepaths_to_robot_directory(robot_name.clone());
        copy_all_collision_mesh_filepaths_to_robot_directory(robot_name.clone());
    }

    let includes_base_meshes = robot_directory_includes_base_meshes(robot_name.clone());
    if !(includes_base_meshes) {
        println!("{}{}WARNING: It looks like there weren't any meshes found for robot {}. Cannot convert links into convex shapes.  {}", color::Fg(color::Red), style::Bold, robot_name.clone(), style::Reset);
        return Err(format!("It looks like there weren't any meshes found for robot {}. Cannot convert links into convex shapes. ", robot_name.clone()));
    }

    // let out_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_triangle_meshes/";
    let visual_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/visual/";
    let collision_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/collision/";
    let num_visual_files = get_all_files_in_directory(visual_fp.clone()).len();
    let num_collision_files = get_all_files_in_directory(collision_fp.clone()).len();

    if num_visual_files > 0 {
        delete_directory_all_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_triangle_meshes_visual/".to_string());
        create_directories_recursively_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_triangle_meshes_visual".to_string());
    }

    if num_collision_files > 0 {
        delete_directory_all_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_triangle_meshes_collision/".to_string());
        create_directories_recursively_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_triangle_meshes_collision".to_string());
    }


    let links = get_all_urdf_links_from_robot_name(robot_name.clone());

    let l = links.len();

    for i in 0..l {
        println!("{}{}Saving Triangle mesh file for link {}.  {} of {}. {}", color::Fg(color::Blue), style::Bold, links[i].name, i+1, l, style::Reset);

        let has_visual = links[i].visual.len() > 0;
        let has_collision = links[i].collision.len() > 0;

        if has_collision {
            let has_mesh_file = links[i].collision[0].filename.is_some();
            if has_mesh_file {
                let fp = collision_fp.clone() + links[i].collision[0].filename.as_ref().unwrap();

                let mut trimesh = TriMeshEngine::new_from_path(fp.clone())?;
                let origin_xyz = links[i].collision[0].origin_xyz.clone();
                let origin_rpy = links[i].collision[0].origin_rpy.clone();
                if !(origin_xyz[0] == 0.0) || !(origin_xyz[1] == 0.0) || !(origin_xyz[2] == 0.0) || !(origin_rpy[0] == 0.0) || !(origin_rpy[1] == 0.0) || !(origin_rpy[2] == 0.0) {
                    trimesh.transform_vertices( &ImplicitDualQuaternion::new_from_euler_angles( origin_rpy[0], origin_rpy[1], origin_rpy[2], Vector3::new(origin_xyz[0], origin_xyz[1], origin_xyz[2])) );
                }
                let scale_xyz = links[i].collision[0].scale.clone();
                if scale_xyz.is_some() {
                    let scale_xyz_unwrap = scale_xyz.as_ref().unwrap();
                    trimesh.scale_vertices(scale_xyz_unwrap[0], scale_xyz_unwrap[1], scale_xyz_unwrap[2]);
                }
                let mesh_offset = &links[i].link_collision_mesh_offset;
                let mesh_scaling = &links[i].link_collision_mesh_scaling;
                if mesh_offset.is_some() {
                    trimesh.transform_vertices(&mesh_offset.as_ref().unwrap());
                }
                if mesh_scaling.is_some() {
                    let s = mesh_scaling.as_ref().unwrap();
                    trimesh.scale_vertices(s[0], s[1], s[2]);
                }

                let out_stl_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_triangle_meshes_collision/" + links[i].name.as_str() + ".stl";
                trimesh.output_to_stl(out_stl_fp);
                let out_obj_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_triangle_meshes_collision/" + links[i].name.as_str() + ".obj";
                trimesh.output_to_obj(out_obj_fp);
            }
        }

        if has_visual {
            let has_mesh_file = links[i].visual[0].filename.is_some();

            if has_mesh_file {
                let fp = visual_fp.clone() + links[i].visual[0].filename.as_ref().unwrap();

                let mut trimesh = TriMeshEngine::new_from_path(fp.clone())?;
                let origin_xyz = links[i].visual[0].origin_xyz.clone();
                let origin_rpy = links[i].visual[0].origin_rpy.clone();
                if !(origin_xyz[0] == 0.0) || !(origin_xyz[1] == 0.0) || !(origin_xyz[2] == 0.0) || !(origin_rpy[0] == 0.0) || !(origin_rpy[1] == 0.0) || !(origin_rpy[2] == 0.0)  {
                    trimesh.transform_vertices( &ImplicitDualQuaternion::new_from_euler_angles( origin_rpy[0], origin_rpy[1], origin_rpy[2], Vector3::new(origin_xyz[0], origin_xyz[1], origin_xyz[2])) );
                }
                let scale_xyz = links[i].visual[0].scale.clone();
                if scale_xyz.is_some() {
                    let scale_xyz_unwrap = scale_xyz.as_ref().unwrap();
                    trimesh.scale_vertices(scale_xyz_unwrap[0], scale_xyz_unwrap[1], scale_xyz_unwrap[2]);
                }
                let mesh_offset = &links[i].link_visual_mesh_offset;
                let mesh_scaling = &links[i].link_visual_mesh_scaling;
                if mesh_offset.is_some() {
                    trimesh.transform_vertices(&mesh_offset.as_ref().unwrap());
                }
                if mesh_scaling.is_some() {
                    let s = mesh_scaling.as_ref().unwrap();
                    trimesh.scale_vertices(s[0], s[1], s[2]);
                }


                let out_stl_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_triangle_meshes_visual/" + links[i].name.as_str() + ".stl";
                trimesh.output_to_stl(out_stl_fp);
                let out_obj_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_triangle_meshes_visual/" + links[i].name.as_str() + ".obj";
                trimesh.output_to_obj(out_obj_fp);
            }
        }
    }

    Ok(())
}

////////////////////////////////////////////////////////////////////////////////////////////////////
