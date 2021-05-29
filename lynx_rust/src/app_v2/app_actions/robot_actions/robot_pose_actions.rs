use crate::robot_modules::robot_set::RobotSet;
use crate::robot_modules::robot_fk_module::VecOfRobotFKResult;
use crate::app_v2::app_structs::{
    robot_structs::robot_link_spawn_structs::RobotLinkSpawnIdxInfo
};
use crate::app_v2::app_type_enums::enums::{
    RobotLinkSpawnType
};
use nalgebra::DVector;
use bevy::prelude::{Query, Transform, With, Or};
use crate::app_v2::app_utils::math_utils::transform_utils::*;

pub fn pose_robot_set_from_full_joint_state(robot_set: &RobotSet,
                                            full_joint_state: &DVector<f64>,
                                            robot_set_idx_in_scene: usize,
                                            query: &mut Query<(&mut Transform, &RobotLinkSpawnIdxInfo, &RobotLinkSpawnType)>) -> Result<(), String> {
    let vec_of_robot_fk_result: VecOfRobotFKResult = robot_set.compute_fk(full_joint_state)?;
    return pose_robot_set_from_vec_of_robot_fk_result(robot_set, &vec_of_robot_fk_result, robot_set_idx_in_scene, query);
}

pub fn pose_robot_set_from_vec_of_robot_fk_result(robot_set: &RobotSet,
                                                  vec_of_robot_fk_result: &VecOfRobotFKResult,
                                                  robot_set_idx_in_scene: usize,
                                                  query: &mut Query<(&mut Transform, &RobotLinkSpawnIdxInfo, &RobotLinkSpawnType)>) -> Result<(), String> {
    let fk_res = vec_of_robot_fk_result.get_robot_fk_results_ref();

    for mut q in query.iter_mut() {
        let mut transform: &mut Transform = &mut q.0;
        let robot_link_spawn_idx_info: &RobotLinkSpawnIdxInfo = &q.1;
        let robot_link_spawn_type: &RobotLinkSpawnType = &q.2;

        if robot_link_spawn_idx_info.robot_set_idx_in_scene == robot_set_idx_in_scene {
            let robot_idx_in_robot_set = robot_link_spawn_idx_info.robot_idx_in_robot_set;
            let link_idx_in_robot = robot_link_spawn_idx_info.link_idx_in_robot;

            if robot_idx_in_robot_set >= fk_res.len() { return Err(format!("robot_idx_in_robot_set {:?} too high for fk_res ({:?})", robot_idx_in_robot_set, fk_res.len())); }

            let robot_fk_result = &fk_res[robot_idx_in_robot_set];

            let link_frames_ref = robot_fk_result.get_link_frames_ref();

            if link_idx_in_robot >= link_frames_ref.len() { return Err(format!("link_idx_in_robot {:?} too high for link_frames_ref ({:?})", link_idx_in_robot, link_frames_ref.len())); }

            if link_frames_ref[link_idx_in_robot].is_none() { continue; }

            let link_frame = link_frames_ref[link_idx_in_robot].as_ref().unwrap();

            match robot_link_spawn_type  {
                RobotLinkSpawnType::StandardMaterial => {
                    let t = convert_z_up_idq_to_y_up_bevy_transform(link_frame);
                    transform.translation = t.translation.clone();
                    transform.rotation = t.rotation.clone();
                }
                RobotLinkSpawnType::SelectableInvisible => {
                    let t = convert_z_up_idq_to_y_up_bevy_transform(link_frame);
                    transform.translation = t.translation.clone();
                    transform.rotation = t.rotation.clone();
                }
                RobotLinkSpawnType::VisibleGlb => {
                    let link_urdf_visual_offset = robot_set
                        .get_robots_ref()[robot_idx_in_robot_set]
                        .get_configuration_module_ref()
                        .robot_model_module.links[link_idx_in_robot]
                        .urdf_link
                        .link_visual_urdf_offset
                        .as_ref()
                        .expect("error unwrapping link_visual_urdf_offset");
                    let t = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame, link_urdf_visual_offset);
                    transform.translation = t.translation.clone();
                    transform.rotation = t.rotation.clone();
                }
            }
        }
    }

    Ok(())
}