use bevy::prelude::{Res, Time, ResMut, Query, Transform};
use crate::app::app_states::res_comps::RobotSetEntityAndInfoServer;
use crate::app::app_states::path_planning::path_planning_res_comps::PathPlanningPlaybackPack;
use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::app::app_utils::robot_utils::robot_pose_utils::update_robot_link_transforms;
use crate::robot_modules::robot_set::RobotSet;
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;
use bevy::render::prelude::Color;


pub fn animate_robot_along_path_single_step(robot_set_entity_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                            robot_set: &RobotSet,
                                            time: &Res<Time>,
                                            robot_server_vector_idx: usize,
                                            path_and_arclength_value: &mut (&mut LinearSplinePath, &mut f64),
                                            transform_query: &mut Query<(&mut Transform)>) {
    let solution_path_ref = &mut path_and_arclength_value.0;
    let mut arclength_value = &mut path_and_arclength_value.1;

    let new_pose = solution_path_ref.get_arclength_interpolated_point(**arclength_value);
    if robot_server_vector_idx >= robot_set_entity_server.get_all_individual_robot_set_entity_and_info_containers_ref().len() { return; }
    *robot_set_entity_server.get_all_individual_robot_set_entity_and_info_containers_mut_ref()[robot_server_vector_idx].get_robot_set_joint_values_mut_ref() = new_pose.clone();
    update_robot_link_transforms(&new_pose, &robot_set, &mut *robot_set_entity_server, robot_server_vector_idx, transform_query);

    robot_set_entity_server.change_link_base_material_data_whole_robot_set(
        robot_server_vector_idx,
        LynxMaterialType::Interpolate {
            start_color_rgb: [0.85, 0.85, 0.85],
            end_color_rgb: [0.239, 0.96, 0.43],
            u: **arclength_value,
            alpha: 1.0,
        });

        **arclength_value += 0.15 * time.delta_seconds_f64();
    if **arclength_value >= 1.1 {
        **arclength_value = 0.0;
    }
}