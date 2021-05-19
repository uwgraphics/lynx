use bevy::prelude::{Plugin, AppBuilder, SystemSet, IntoSystem, ResMut, Commands, AssetServer, Assets, StandardMaterial, Res, Query, Transform};
use crate::app::app_states::app_states_enum::AppState;
use crate::app::app_states::res_comps::{RobotSetEntityAndInfoServer, RobotLinkSpawnType};
use crate::app::app_utils::robot_utils::full_robot_set_spawners::*;
use crate::utils::utils_vars::lynx_vars_generic::LynxVarsGeneric;
use crate::robot_modules::prelude::*;
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;
use crate::app::app_utils::asset_utils::robot_set_asset_loader::RobotSetAssetLoader;
use crate::app::app_states::path_planning::path_planning_gui::path_planning_gui_plugin::PathPlanningGUIPlugin;
use crate::app::app_states::path_planning::path_planning_on_update_systems::path_planning_on_update_systems_plugin::PathPlanningOnUpdateSystemsPlugin;
use crate::app::app_states::path_planning::path_planning_gui::path_planning_gui_values::PathPlanningGUIValues;
use crate::app::app_states::path_planning::path_planning_res_comps::{PathPlanningPlaybackPack, PathPlanningStartAndGoalStatePack};
use crate::app::app_utils::robot_utils::robot_pose_utils::update_robot_link_transforms;

pub struct PathPlanningStatePlugin;

impl Plugin for PathPlanningStatePlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_system_set(
            SystemSet::on_enter(AppState::PathPlanning)
                .with_system(path_planning_enter.system())
            )

            .add_system_set(
                SystemSet::on_exit(AppState::PathPlanning)
                    .with_system(path_planning_exit.system())
            )

            .add_plugin(PathPlanningOnUpdateSystemsPlugin)
            .add_plugin(PathPlanningGUIPlugin);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn path_planning_enter(mut robot_set_entity_server: ResMut<RobotSetEntityAndInfoServer>,
                       mut robot_set_asset_loader: ResMut<RobotSetAssetLoader>,
                       mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                       mut commands: Commands,
                       asset_server: Res<AssetServer>,
                       mut materials: ResMut<Assets<StandardMaterial>>,
                       mut path_planning_start_and_goal_state_pack: ResMut<PathPlanningStartAndGoalStatePack>,
                       mut transform_query: Query<&mut Transform>) {

    path_planning_enter_generic(&mut robot_set_entity_server,
                                &mut robot_set_asset_loader,
                                &mut lynx_vars,
                                &mut commands,
                                &asset_server,
                                &mut materials,
                                &mut path_planning_start_and_goal_state_pack,
                                &mut transform_query);
}

pub fn path_planning_enter_generic(robot_set_entity_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                   robot_set_asset_loader: &mut ResMut<RobotSetAssetLoader>,
                                   lynx_vars: &mut ResMut<LynxVarsGeneric<'static>>,
                                   commands: &mut Commands,
                                   asset_server: &Res<AssetServer>,
                                   materials: &mut ResMut<Assets<StandardMaterial>>,
                                   path_planning_start_and_goal_state_pack: &mut ResMut<PathPlanningStartAndGoalStatePack>,
                                   transform_query: &mut Query<&mut Transform>) {
    commands.insert_resource(PathPlanningGUIValues::new());
    commands.insert_resource(PathPlanningPlaybackPack::new());

    let robot_world = get_lynx_var_mut_ref_generic!(&mut **lynx_vars, RobotWorld, "robot_world").expect("error loading robot_world");
    let robot_set = robot_world.get_robot_set_mut_ref();

    robot_set_entity_server.hide_robot(0);

    while robot_set_entity_server.get_num_individual_robot_set_entity_and_info_containers() < 3 {
        spawn_robot_set(robot_set, commands, &asset_server, materials, robot_set_entity_server, robot_set_asset_loader, RobotLinkSpawnType::Visualization, Some(LynxMaterialType::Default));
    }

    robot_set_entity_server.unhide_robot(1);
    robot_set_entity_server.unhide_robot(2);
    // robot_set_entity_server.hide_robot(3);
    robot_set_entity_server.change_link_base_material_data_whole_robot_set(1, LynxMaterialType::PathPlanningStart);
    robot_set_entity_server.change_link_base_material_data_whole_robot_set(2, LynxMaterialType::PathPlanningGoal);
    // robot_set_entity_server.change_link_base_material_data_whole_robot_set(3, LynxMaterialType::Default);

    if path_planning_start_and_goal_state_pack.start_state.is_some() && path_planning_start_and_goal_state_pack.goal_state.is_some() {
        robot_set_entity_server.get_all_individual_robot_set_entity_and_info_containers_mut_ref()[1].set_robot_set_joint_values(&path_planning_start_and_goal_state_pack.start_state.as_ref().unwrap());
        robot_set_entity_server.get_all_individual_robot_set_entity_and_info_containers_mut_ref()[2].set_robot_set_joint_values(&path_planning_start_and_goal_state_pack.goal_state.as_ref().unwrap());
        path_planning_start_and_goal_state_pack.start_state = None;
        path_planning_start_and_goal_state_pack.goal_state = None;
    }

    let start_state = robot_set_entity_server.get_all_individual_robot_set_entity_and_info_containers_ref()[1].get_robot_set_joint_values_ref().clone();
    let goal_state = robot_set_entity_server.get_all_individual_robot_set_entity_and_info_containers_ref()[2].get_robot_set_joint_values_ref().clone();
    update_robot_link_transforms(&start_state, robot_set, robot_set_entity_server, 1, transform_query);
    update_robot_link_transforms(&goal_state, robot_set, robot_set_entity_server, 2, transform_query);

    let mut robot_worlds = get_lynx_var_all_mut_refs_generic!(&mut **lynx_vars, RobotWorld, "robot_world");
    for r in robot_worlds {
        if r.get_collision_environment_option_mut_ref().is_some() {
            r.get_collision_environment_option_mut_ref().as_mut().unwrap().update_bounding_volumes_on_all_environment_obbs();
        }
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn path_planning_exit(mut robot_set_entity_server: ResMut<RobotSetEntityAndInfoServer>) {
    robot_set_entity_server.hide_robot(1);
    robot_set_entity_server.hide_robot(2);
    robot_set_entity_server.hide_robot(3);
    robot_set_entity_server.unhide_robot(0);
    robot_set_entity_server.purge_robot_server_vectors(1);
    robot_set_entity_server.purge_robot_server_vectors(2);
    robot_set_entity_server.purge_robot_server_vectors(3);
}

////////////////////////////////////////////////////////////////////////////////////////////////////



