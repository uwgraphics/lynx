use bevy::prelude::Plugin;
use bevy::app::AppBuilder;
use crate::app::app_states::perpetual::{camera_management_systems::camera_management_plugin::CameraManagementPlugin,
                                        robot_management_systems::robot_management_plugin::RobotManagementPlugin,
                                        gui_management_systems::gui_management_plugin::GUIManagementPlugin,
                                        viewport_visuals_systems::viewport_visuals_plugin::ViewportVisualsPlugin,
                                        environment_management_systems::environment_management_plugin::*};
use bevy::ecs::prelude::IntoSystem;

pub struct PerpetualPlugin;

impl Plugin for PerpetualPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_plugin(ViewportVisualsPlugin)
            .add_plugin(CameraManagementPlugin)
            .add_plugin(GUIManagementPlugin)
            .add_plugin(RobotManagementPlugin)
            .add_plugin(EnvironmentManagementPlugin);
    }
}