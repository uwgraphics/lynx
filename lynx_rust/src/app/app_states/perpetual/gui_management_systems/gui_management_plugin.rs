use bevy::prelude::*;
use bevy_egui::{EguiContext, egui};
use crate::utils::utils_vars::prelude::*;
use crate::app::app_states::perpetual::perpetual_res_comps::*;
use crate::app::app_utils::robot_utils::full_robot_set_spawners::spawn_robot_set;
use crate::robot_modules::prelude::*;
use crate::app::app_states::res_comps::*;
use crate::app::app_utils::gui_utils::robot_selection_choices::*;
use crate::app::app_states::perpetual::gui_management_systems::robot_selectors::*;
use bevy_egui::egui::{Ui, Pos2};
use crate::app::app_states::perpetual::gui_management_systems::{
    app_state_selector::*,
    robot_link_focus_windows::*,
    collision_information_window::*,
    environment_selector::*,
    environment_focus_windows::*
};
use crate::app::app_states::app_states_enum::AppState;
use crate::app::app_utils::drawing_utils::line_drawing::LineType;
use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;

pub struct GUIManagementPlugin;

impl Plugin for GUIManagementPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_system(main_gui_system.system().label("main_gui"))
            .add_system(robot_link_focus_windows_system.system())
            .add_system(environment_focus_windows_system.system());
    }
}

fn main_gui_system(egui_context: Res<EguiContext>,
                   mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                   mut spawn_robot: ResMut<SpawnNewPhysicalRobot>,
                   mut spawn_environment: ResMut<SpawnNewEnvironment>,
                   mut current_main_gui_values: ResMut<CurrentMainGUIValues>,
                   mut collision_window_variables: ResMut<CollisionWindowVariables>,
                   mut state: ResMut<State<AppState>>,
                   mut robot_set_entity_and_info_server: ResMut<RobotSetEntityAndInfoServer>,
                   mut env_entity_and_info_server: ResMut<EnvironmentEntityAndInfoServer>,
                   key: Res<Input<KeyCode>>,
                   mut commands: Commands,
                   mut meshes: ResMut<Assets<Mesh>>,
                   mut materials: ResMut<Assets<StandardMaterial>>,
                   mut line_query: Query<(Entity, &LineType)>) {

    if key.just_pressed(KeyCode::Tab) { current_main_gui_values.hide_main_gui = !current_main_gui_values.hide_main_gui; }

    if !current_main_gui_values.hide_main_gui {
        egui::SidePanel::left("side_panel", 220.0).show(egui_context.ctx(), |ui| {
            egui::ScrollArea::auto_sized().show(ui, |ui| {
                gui_robot_selector(ui, &mut spawn_robot, &mut current_main_gui_values, &mut lynx_vars);
                ui.separator();
                gui_robot_set_selector(ui, &mut spawn_robot, &mut current_main_gui_values, &mut lynx_vars);
                ui.separator();
                gui_environment_selector(ui, &egui_context, &mut spawn_environment, &mut current_main_gui_values, &mut lynx_vars, &mut env_entity_and_info_server, &mut commands);
                ui.separator();
                gui_app_state_selector(ui, &mut current_main_gui_values, &mut state);
                ui.separator();
                let robot_world = get_lynx_var_mut_ref_generic!(&mut *lynx_vars, RobotWorld, "robot_world").expect("error loading robot world");
                collision_information_selector(ui, &egui_context, &mut current_main_gui_values, &mut collision_window_variables, &mut robot_set_entity_and_info_server, &mut env_entity_and_info_server, robot_world);
            });
        });
    } else {
        // let fixed_pos = if current_main_gui_values.hide_application_gui { Pos2::new(5., 5.) } else { Pos2::new(5., 700.) };
        // egui::Area::new("text").show(egui_context.ctx(), |ui| {
            // ui.label("`Tab' to open side panel.");
        // });
    }
}


