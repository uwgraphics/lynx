use bevy::prelude::{Plugin, AppBuilder};
use crate::app::app_states::joint_value_sliders::joint_value_sliders_gui::joint_value_sliders_gui_plugin::JointValueSlidersGUIPlugin;

pub struct JointValueSlidersStatePlugin;

impl Plugin for JointValueSlidersStatePlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_plugin(JointValueSlidersGUIPlugin);
    }
}