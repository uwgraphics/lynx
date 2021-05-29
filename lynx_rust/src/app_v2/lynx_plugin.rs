use crate::app_v2::dependency_plugins::*;
use crate::app_v2::debug_plugin::*;
use crate::app_v2::app_state_plugins::{
    state0_perpetual_plugin::PerpetualPlugin,
    state1_startup_plugin::StartupPlugin
};


use crate::app_v2::app_state_plugins::app_states_enum::*;
use bevy::app::{Plugin, AppBuilder};
use bevy::prelude::{SystemSet};



pub struct LynxPlugin;
impl Plugin for LynxPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_state(AppState::NullState)
            .add_plugins(DependencyPlugins)
            .add_plugin(DebugPlugin)
            .add_plugin(PerpetualPlugin)
            .add_plugin(StartupPlugin);
            // .add_plugin(StartupPlugin)
            // .add_plugin(PerpetualPlugin)
            // .add_plugin(JointValueSlidersStatePlugin)
            // .add_plugin(PathPlanningStatePlugin);
    }
}

