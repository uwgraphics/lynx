use crate::app::app_states::res_comps::{*};
use crate::robot_modules::prelude::{RobotWorld, RobotSet};
use crate::robot_modules::robot_fk_module::VecOfRobotFKResult;
use bevy::prelude::{ResMut, Res, Query, Transform, World, Vec3};
use crate::prelude::LynxVarsGeneric;
use crate::utils::utils_math::prelude::vec_to_dvec;
use crate::app::app_utils::math_utils::transform_utils::*;
use nalgebra::DVector;
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;

/*
pub fn update_robot_link_transforms(full_state_vec: &DVector<f64>,
                                    lynx_vars: &ResMut<LynxVarsGeneric<'static>>,
                                    q: &mut Query<(&mut Transform, &RobotLinkIdx, &RobotIdxInSet, &RobotLinkSpawnType, &RobotLinkMeshType)>,
                                    robot_link_spawn_type: &RobotLinkSpawnType) {
    let robot_world = get_lynx_var_ref_generic!(& **lynx_vars, RobotWorld, "robot_world").expect("error loading RobotWorld");
    let robot_set = robot_world.get_robot_set_ref();
    // let joint_values = &physical_robot_set_joint_values.0;
    // let vec_of_fk_res: VecOfRobotFKResult = robot_set.compute_fk(&vec_to_dvec(joint_values)).expect("error computing fk");
    let vec_of_fk_res_ = robot_set.compute_fk(full_state_vec);
    if vec_of_fk_res_.is_err() { return; }
    let vec_of_fk_res: VecOfRobotFKResult = vec_of_fk_res_.unwrap();

    for mut a in q.iter_mut() {
        let transform: &mut Transform = &mut a.0;
        let robot_link_idx: &RobotLinkIdx = &a.1;
        let robot_idx_in_set: &RobotIdxInSet = &a.2;
        let robot_link_spawn_type_from_query: &RobotLinkSpawnType = &a.3;
        let robot_link_mesh_type: &RobotLinkMeshType = &a.4;

        if robot_link_spawn_type_from_query == robot_link_spawn_type {
            if robot_idx_in_set.0 < robot_set.get_robots_ref().len() && robot_link_idx.0 < robot_set.get_robots_ref()[robot_idx_in_set.0].get_mesh_info_module_ref().get_link_mesh_visual_offsets().len() {
                let link_mesh_visual_offset = robot_set.get_robots_ref()[robot_idx_in_set.0].get_mesh_info_module_ref().get_link_mesh_visual_offsets()[robot_link_idx.0].as_ref().expect("error unwrapping link_mesh_visual_offset in robot_pose_manager.");
                let link_urdf_visual_offset = robot_set.get_robots_ref()[robot_idx_in_set.0].get_mesh_info_module_ref().get_link_urdf_visual_offsets()[robot_link_idx.0].as_ref().expect("error unwrapping link_urdf_visual_offset in robot_pose_manager.");
                let link_frame_ = vec_of_fk_res.get_robot_fk_results_ref()[robot_idx_in_set.0].get_link_frames_ref()[robot_link_idx.0].as_ref();
                if link_frame_.is_none() { continue; }
                let link_frame = link_frame_.unwrap();

                match robot_link_mesh_type {
                    RobotLinkMeshType::VislbleGlb => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_urdf_visual_offset) }
                    RobotLinkMeshType::StandardMaterial => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset) }
                    RobotLinkMeshType::InvisibleMaterial => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset) }
                }
            }
        }
    }
}
*/

/*
pub fn update_robot_link_transforms2(full_state_vec: &DVector<f64>,
                                    lynx_vars: &ResMut<LynxVarsGeneric<'static>>,
                                    q: &mut Query<(&mut Transform, &RobotLinkInfoContainer)>,
                                    // q: &mut Query<(&mut Transform, &RobotLinkIdx, &RobotIdxInSet, &RobotLinkSpawnType, &RobotLinkMeshType)>,
                                    robot_link_spawn_type: &RobotLinkSpawnType) {
    let robot_world = get_lynx_var_ref_generic!(& **lynx_vars, RobotWorld, "robot_world").expect("error loading RobotWorld");
    let robot_set = robot_world.get_robot_set_ref();
    // let joint_values = &physical_robot_set_joint_values.0;
    // let vec_of_fk_res: VecOfRobotFKResult = robot_set.compute_fk(&vec_to_dvec(joint_values)).expect("error computing fk");
    let vec_of_fk_res_ = robot_set.compute_fk(full_state_vec);
    if vec_of_fk_res_.is_err() { return; }
    let vec_of_fk_res: VecOfRobotFKResult = vec_of_fk_res_.unwrap();

    for mut a in q.iter_mut() {
        let transform: &mut Transform = &mut a.0;
        let robot_link_info_container: &RobotLinkInfoContainer = &a.1;

        if &robot_link_info_container.robot_link_spawn_type == robot_link_spawn_type {
            if robot_link_info_container.robot_set_idx < robot_set.get_robots_ref().len() && robot_link_info_container.robot_link_idx < robot_set.get_robots_ref()[robot_link_info_container.robot_set_idx].get_mesh_info_module_ref().get_link_mesh_visual_offsets().len() {
                let link_mesh_visual_offset = robot_set.get_robots_ref()[robot_link_info_container.robot_set_idx].get_mesh_info_module_ref().get_link_mesh_visual_offsets()[robot_link_info_container.robot_link_idx].as_ref().expect("error unwrapping link_mesh_visual_offset in robot_pose_manager.");
                let link_urdf_visual_offset = robot_set.get_robots_ref()[robot_link_info_container.robot_set_idx].get_mesh_info_module_ref().get_link_urdf_visual_offsets()[robot_link_info_container.robot_link_idx].as_ref().expect("error unwrapping link_urdf_visual_offset in robot_pose_manager.");
                let link_frame_ = vec_of_fk_res.get_robot_fk_results_ref()[robot_link_info_container.robot_set_idx].get_link_frames_ref()[robot_link_info_container.robot_link_idx].as_ref();
                if link_frame_.is_none() { continue; }
                let link_frame = link_frame_.unwrap();

                match robot_link_info_container.robot_link_mesh_type {
                    RobotLinkMeshType::VislbleGlb => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_urdf_visual_offset) }
                    RobotLinkMeshType::StandardMaterial => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset) }
                    RobotLinkMeshType::InvisibleMaterial => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset) }
                }
            }
        }
    }
}
*/

pub fn update_robot_link_transforms(full_state_vec: &DVector<f64>,
                                    robot_set: &RobotSet,
                                    robot_set_entity_and_info_server: &mut RobotSetEntityAndInfoServer,
                                    robot_server_vector_idx: usize,
                                    q: &mut Query<(&mut Transform)>) {
    if robot_set_entity_and_info_server.is_robot_hidden(robot_server_vector_idx) { return; }

    // let robot_world = get_lynx_var_ref_generic!(& **lynx_vars, RobotWorld, "robot_world").expect("error loading RobotWorld");
    // let robot_set = robot_world.get_robot_set_ref();
    let vec_of_fk_res_ = robot_set.compute_fk(full_state_vec);
    if vec_of_fk_res_.is_err() { return; }
    let vec_of_fk_res: VecOfRobotFKResult = vec_of_fk_res_.unwrap();

    let ind_res = robot_set_entity_and_info_server.get_individual_robot_set_entity_and_info_container_ref(robot_server_vector_idx);
    if ind_res.is_err() { return; }
    let ind = ind_res.unwrap();

    let visible_glb_scene_link_entities = ind.get_visible_glb_scene_link_entities_ref();
    let standard_material_link_entities = ind.get_standard_material_link_entites_ref();
    let invisible_material_link_entities = ind.get_invisible_material_link_entites_ref();
    let fk_res_per_robot = vec_of_fk_res.get_robot_fk_results_ref();

    let l = fk_res_per_robot.len();
    for i in 0..l {
        let fk_res = fk_res_per_robot[i].get_link_frames_ref();
        let robot_ref = &robot_set.get_robots_ref()[i];

        let l2 = fk_res.len();
        for j in 0..l2 {
            // let link_mesh_visual_offset = robot_ref.get_mesh_info_module_ref().get_link_mesh_visual_offsets()[j].as_ref().expect("error unwrapping link_mesh_visual_offset in robot_pose_manager.");
            // let link_urdf_visual_offset = robot_ref.get_mesh_info_module_ref().get_link_urdf_visual_offsets()[j].as_ref().expect("error unwrapping link_urdf_visual_offset in robot_pose_manager.");
            let link_urdf_visual_offset = robot_ref.get_configuration_module_ref().robot_model_module.links[j].urdf_link.link_visual_urdf_offset.as_ref().expect("error unwrapping link_urdf_visual_offset in robot_pose_manager");

            let link_frame_ = fk_res[j].as_ref();

            if link_frame_.is_none() { continue; }
            let link_frame = link_frame_.unwrap();

            let visible_glb_entity_option = visible_glb_scene_link_entities[i][j];
            let standard_material_link_entity_option = standard_material_link_entities[i][j];
            let invisible_material_link_entity_option = invisible_material_link_entities[i][j];

            if visible_glb_entity_option.is_some() {
                let query_res = q.get_mut(visible_glb_entity_option.unwrap().clone());
                if query_res.is_ok() {
                    let mut t_ = query_res.ok();
                    let mut t = t_.as_mut().unwrap();
                    **t = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_urdf_visual_offset);
                    let curr_material = ind.get_curr_material(i, j).unwrap();
                    if curr_material > LynxMaterialType::VisualizationDefault { t.scale = Vec3::new(0., 0., 0.); }
                }
            }

            if standard_material_link_entity_option.is_some() {
                let query_res = q.get_mut(standard_material_link_entity_option.unwrap().clone());
                if query_res.is_ok() {
                    let mut t_ = query_res.ok();
                    let mut t = t_.as_mut().unwrap();
                    **t = convert_z_up_idq_to_y_up_bevy_transform(&link_frame);
                    // let c = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset);
                    // t.translation = c.translation;
                    // t.rotation = c.rotation;
                    // **t = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset);
                }
            }

            if invisible_material_link_entity_option.is_some() {
                let query_res = q.get_mut(invisible_material_link_entity_option.unwrap().clone());
                if query_res.is_ok() {
                    let mut t_ = query_res.ok();
                    let mut t = t_.as_mut().unwrap();
                    **t = convert_z_up_idq_to_y_up_bevy_transform(&link_frame);

                    // let c = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset);
                    // t.translation = c.translation;
                    // t.rotation = c.rotation;
                    // **t = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset);
                }
            }

        }
    }

    /*
    for mut a in q.iter_mut() {
        let transform: &mut Transform = &mut a.0;
        let robot_link_info_container: &RobotLinkInfoContainer = &a.1;

        if &robot_link_info_container.robot_link_spawn_type == robot_link_spawn_type {
            if robot_link_info_container.robot_set_idx < robot_set.get_robots_ref().len() && robot_link_info_container.robot_link_idx < robot_set.get_robots_ref()[robot_link_info_container.robot_set_idx].get_mesh_info_module_ref().get_link_mesh_visual_offsets().len() {
                let link_mesh_visual_offset = robot_set.get_robots_ref()[robot_link_info_container.robot_set_idx].get_mesh_info_module_ref().get_link_mesh_visual_offsets()[robot_link_info_container.robot_link_idx].as_ref().expect("error unwrapping link_mesh_visual_offset in robot_pose_manager.");
                let link_urdf_visual_offset = robot_set.get_robots_ref()[robot_link_info_container.robot_set_idx].get_mesh_info_module_ref().get_link_urdf_visual_offsets()[robot_link_info_container.robot_link_idx].as_ref().expect("error unwrapping link_urdf_visual_offset in robot_pose_manager.");
                let link_frame_ = vec_of_fk_res.get_robot_fk_results_ref()[robot_link_info_container.robot_set_idx].get_link_frames_ref()[robot_link_info_container.robot_link_idx].as_ref();
                if link_frame_.is_none() { continue; }
                let link_frame = link_frame_.unwrap();

                match robot_link_info_container.robot_link_mesh_type {
                    RobotLinkMeshType::VislbleGlb => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_urdf_visual_offset) }
                    RobotLinkMeshType::StandardMaterial => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset) }
                    RobotLinkMeshType::InvisibleMaterial => { *transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_mesh_visual_offset) }
                }
            }
        }
    }
    */


}