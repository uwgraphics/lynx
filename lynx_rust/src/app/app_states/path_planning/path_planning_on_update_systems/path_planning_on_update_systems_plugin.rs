use bevy::prelude::{Plugin, SystemSet, ResMut, Commands, Res, Assets, StandardMaterial, AssetServer, IntoSystem, Time, Query, Transform};
use bevy::app::AppBuilder;
use crate::app::app_states::app_states_enum::*;
use crate::app::app_states::res_comps::{RobotSetEntityAndInfoServer, InstantContainer};
use crate::app::app_utils::asset_utils::robot_set_asset_loader::RobotSetAssetLoader;
use crate::utils::utils_vars::prelude::LynxVarsGeneric;
use std::time::{Instant, Duration};
use crate::app::app_states::path_planning::path_planning_state_plugin::path_planning_enter_generic;
use crate::app::app_states::path_planning::path_planning_res_comps::{PathPlanningPlaybackPack, PathPlanningStartAndGoalStatePack};
use crate::app::app_utils::robot_utils::animate_along_path::animate_robot_along_path_single_step;
use crate::robot_modules::robot_world::RobotWorld;

pub struct PathPlanningOnUpdateSystemsPlugin;

impl Plugin for PathPlanningOnUpdateSystemsPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_system_set(
                SystemSet::on_update(AppState::PathPlanning)
                    .with_system(reset_on_load_new_robot.system())
                    .with_system(solution_playback_animation.system())
            );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn reset_on_load_new_robot(mut robot_set_entity_server: ResMut<RobotSetEntityAndInfoServer>,
                           mut robot_set_asset_loader: ResMut<RobotSetAssetLoader>,
                           mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                           mut commands: Commands,
                           asset_server: Res<AssetServer>,
                           mut materials: ResMut<Assets<StandardMaterial>>,
                           mut instant_container: ResMut<InstantContainer>,
                           mut path_planning_start_and_goal_state_pack: ResMut<PathPlanningStartAndGoalStatePack>,
                           mut transform_query: Query<&mut Transform>) {
    if instant_container.robot_reset.elapsed() <= Duration::from_millis(200) {
        path_planning_enter_generic(&mut robot_set_entity_server,
                                    &mut robot_set_asset_loader,
                                    &mut lynx_vars,
                                    &mut commands,
                                    &asset_server,
                                    &mut materials,
                                    &mut path_planning_start_and_goal_state_pack,
                                    &mut transform_query);
    }
}

fn solution_playback_animation(mut robot_set_entity_server: ResMut<RobotSetEntityAndInfoServer>,
                               lynx_vars: Res<LynxVarsGeneric<'static>>,
                               mut path_planning_playback_pack: ResMut<PathPlanningPlaybackPack>,
                               time: Res<Time>,
                               mut transform_query: Query<(&mut Transform)>) {

    if path_planning_playback_pack.display_playback_path && path_planning_playback_pack.curr_solution.is_some() {
        let curr_solution_ref = path_planning_playback_pack.curr_solution.as_ref().unwrap();

        let robot_world = get_lynx_var_ref_generic!(&*lynx_vars, RobotWorld, "robot_world").expect("error loading robot world");
        let robot_set = robot_world.get_robot_set_ref();

        animate_robot_along_path_single_step(&mut robot_set_entity_server,
                                             robot_set,
                                             &time,
                                             3,
                                             path_planning_playback_pack.get_path_mut_ref_and_arclength_curr_value_mut_ref().as_mut().unwrap(),
                                             &mut transform_query);

    }
}

