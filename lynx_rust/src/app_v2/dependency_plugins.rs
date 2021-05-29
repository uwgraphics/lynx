use bevy_mod_picking::*;
use bevy::prelude::*;
use bevy_stl;
use bevy::app::PluginGroupBuilder;
use bevy_egui::EguiPlugin;
use bevy_prototype_debug_lines::DebugLinesPlugin;
use bevy::diagnostic::{LogDiagnosticsPlugin, FrameTimeDiagnosticsPlugin};

pub struct DependencyPlugins;
impl PluginGroup for DependencyPlugins {
    fn build(&mut self, group: &mut PluginGroupBuilder) {
        group
            .add(bevy_stl::StlPlugin)
            .add(PickingPlugin)
            .add(InteractablePickingPlugin)
            // .add(HighlightablePickingPlugin) // optional
            // .add(DebugEventsPickingPlugin) // optional
            .add(EguiPlugin)
            // .add(DebugLinesPlugin);
            .add(LogDiagnosticsPlugin::default())
            .add(FrameTimeDiagnosticsPlugin::default());
    }
}