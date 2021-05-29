use bevy::prelude::*;
use crate::app_v2::app_state_plugins::state1_startup_systems::state1_startup_systems::*;

pub struct StartupPlugin;
impl Plugin for StartupPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_startup_system(insert_init_resources.system())
            .add_startup_system(spawn_init_lights.system())
            .add_startup_system(spawn_init_camera.system());
    }
}