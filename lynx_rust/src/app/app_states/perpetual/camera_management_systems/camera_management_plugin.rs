use crate::app::app_states::perpetual::camera_management_systems::camera_manager::*;
use bevy::prelude::Plugin;
use bevy::app::AppBuilder;
use bevy::ecs::prelude::IntoSystem;

pub struct CameraManagementPlugin;

impl Plugin for CameraManagementPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_system(camera_manager.system());
    }
}