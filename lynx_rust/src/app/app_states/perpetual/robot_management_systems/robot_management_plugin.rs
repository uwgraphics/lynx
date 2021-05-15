use bevy::prelude::*;
use bevy::app::AppBuilder;
use crate::app::app_states::perpetual::robot_management_systems::{robot_spawn_manager::robot_spawn_manager_system};
use crate::app::app_states::perpetual::robot_management_systems::robot_link_material_manager::robot_link_material_manager_system;
use crate::app::app_states::perpetual::robot_management_systems::robot_link_focus_selector::robot_link_focus_selector;
use crate::app::app_states::perpetual::robot_management_systems::robot_hidden_manager::*;

pub struct RobotManagementPlugin;

impl Plugin for RobotManagementPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_system(robot_spawn_manager_system.system().label("robot_spawn_manager"))
            .add_system_to_stage(CoreStage::PreUpdate, robot_link_material_manager_system.system())
            .add_system_to_stage(CoreStage::PreUpdate, robot_hidden_manager_system.system())
            .add_system(robot_link_focus_selector.system());
    }
}