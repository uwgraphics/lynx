use bevy::prelude::{Plugin, AppBuilder, SystemSet, IntoSystem, ResMut, Commands, AssetServer, Assets, StandardMaterial, Res};
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
use crate::app::app_states::path_planning::path_planning_res_comps::PathPlanningPlaybackPack;

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
                       mut materials: ResMut<Assets<StandardMaterial>>) {

    path_planning_enter_generic(&mut robot_set_entity_server, &mut robot_set_asset_loader, &mut lynx_vars, &mut commands, &asset_server, &mut materials);
}

pub fn path_planning_enter_generic(robot_set_entity_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                   robot_set_asset_loader: &mut ResMut<RobotSetAssetLoader>,
                                   lynx_vars: &mut ResMut<LynxVarsGeneric<'static>>,
                                   commands: &mut Commands,
                                   asset_server: &Res<AssetServer>,
                                   materials: &mut ResMut<Assets<StandardMaterial>>) {
    commands.insert_resource(PathPlanningGUIValues::new());
    commands.insert_resource(PathPlanningPlaybackPack::new());

    let robot_world = get_lynx_var_mut_ref_generic!(&mut **lynx_vars, RobotWorld, "robot_world").expect("error loading robot_world");
    let robot_set = robot_world.get_robot_set_mut_ref();

    robot_set_entity_server.hide_robot(0);

    while robot_set_entity_server.get_num_individual_robot_set_entity_and_info_containers() < 4 {
        spawn_robot_set(robot_set, commands, &asset_server, materials, robot_set_entity_server, robot_set_asset_loader, RobotLinkSpawnType::Visualization, Some(LynxMaterialType::Default));
    }

    robot_set_entity_server.unhide_robot(1);
    robot_set_entity_server.hide_robot(2);
    robot_set_entity_server.hide_robot(3);
    robot_set_entity_server.change_link_base_material_data_whole_robot_set(1, LynxMaterialType::PathPlanningStart);
    robot_set_entity_server.change_link_base_material_data_whole_robot_set(2, LynxMaterialType::PathPlanningGoal);
    robot_set_entity_server.change_link_base_material_data_whole_robot_set(3, LynxMaterialType::Default);
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



