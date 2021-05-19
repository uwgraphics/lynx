use bevy::prelude::*;
use crate::app::app_states::perpetual::viewport_visuals_systems::{grid_lines::*,
                                                                  robot_mesh_hover_highlighting::*,
                                                                  robot_mesh_focus_highlighting::*,
                                                                  environment_mesh_hover_highlighting::*,
                                                                  environment_mesh_focus_highlighting::*
};
use crate::app::app_states::app_states_enum::AppState;

pub struct ViewportVisualsPlugin;

impl Plugin for ViewportVisualsPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_startup_system(draw_grid_lines.system())
            .add_system(grid_lines_manager_system.system())
            .add_system(robot_mesh_hover_highlighting.system())
            .add_system(environment_mesh_hover_highlighting.system())
            .add_system(robot_mesh_focus_highlighting.system())
            .add_system(environment_mesh_focus_highlighting.system());
    }
}