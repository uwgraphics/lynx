use crate::app::dependency_plugins::*;
use crate::app::app_states::{startup::startup_plugin::*,
                             perpetual::perpetual_plugin::*,
                             joint_value_sliders::joint_value_sliders_state_plugin::*,
                             path_planning::path_planning_state_plugin::*};
use crate::app::app_states::app_states_enum::*;
use bevy::app::{Plugin, AppBuilder};
use bevy::prelude::{SystemSet};


pub struct LynxPlugin;
impl Plugin for LynxPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_state(AppState::NullState)

            .add_plugins(DependencyPlugins)
            .add_plugin(StartupPlugin)
            .add_plugin(PerpetualPlugin)
            .add_plugin(JointValueSlidersStatePlugin)
            .add_plugin(PathPlanningStatePlugin);

    }
}




