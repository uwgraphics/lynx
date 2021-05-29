use crate::app_v2::app_state_plugins::state0_perpetual_systems::{
    viewport_visuals_systems::*,
    camera_management_systems::*,
    game_engine_status_systems::*
};
use bevy::prelude::{Plugin, AppBuilder, IntoSystem};
use bevy::app::CoreStage;

pub struct PerpetualPlugin;
impl Plugin for PerpetualPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_startup_system(grid_management_system.system())
            .add_system(main_camera_management_system.system())
            .add_system_to_stage(CoreStage::PostUpdate, frame_counter.system());
    }
}