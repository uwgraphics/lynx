use bevy::prelude::*;
use crate::app::app_states::perpetual::environment_management_systems::{
    environment_spawn_manager::*,
    environment_material_manager::*,
    environment_focus_selector::*
};


pub struct EnvironmentManagementPlugin;

impl Plugin for EnvironmentManagementPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_system(environment_spawn_manager_system.system())
            .add_system_to_stage(CoreStage::PreUpdate, environment_material_manager_system.system())
            .add_system(environment_focus_selector.system());
    }
}
