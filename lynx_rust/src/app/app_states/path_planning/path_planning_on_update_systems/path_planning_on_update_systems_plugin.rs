use bevy::prelude::{Plugin, SystemSet, ResMut, Commands, Res, Assets, StandardMaterial, AssetServer, IntoSystem};
use bevy::app::AppBuilder;
use crate::app::app_states::app_states_enum::*;
use crate::app::app_states::res_comps::{RobotSetEntityAndInfoServer, InstantContainer};
use crate::app::app_utils::asset_utils::robot_set_asset_loader::RobotSetAssetLoader;
use crate::utils::utils_vars::prelude::LynxVarsGeneric;
use std::time::{Instant, Duration};
use crate::app::app_states::path_planning::path_planning_state_plugin::path_planning_enter_generic;

pub struct PathPlanningOnUpdateSystemsPlugin;

impl Plugin for PathPlanningOnUpdateSystemsPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_system_set(
                SystemSet::on_update(AppState::PathPlanning)
                    .with_system(reset_on_load_new_robot.system())
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
                           mut instant_container: ResMut<InstantContainer>) {
    if instant_container.robot_reset.elapsed() <= Duration::from_millis(200) {
        path_planning_enter_generic(&mut robot_set_entity_server, &mut robot_set_asset_loader, &mut lynx_vars, &mut commands, &asset_server, &mut materials);
    }
}



