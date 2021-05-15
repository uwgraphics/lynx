use bevy::prelude::*;
use crate::app::app_states::app_states_enum::AppState;
use bevy_egui::{EguiContext, egui};
use crate::utils::utils_vars::prelude::LynxVarsGeneric;
use crate::app::app_states::res_comps::{CurrentMainGUIValues, RobotSetEntityAndInfoServer};
use crate::app::app_states::joint_value_sliders::joint_value_sliders_gui::joint_value_sliders_gui_plugin::joint_value_sliders_gui_system_generic;
use crate::app::app_states::path_planning::path_planning_gui::path_planning_gui_values::PathPlanningGUIValues;
use crate::app::app_states::path_planning::path_planners_enum::PathPlannerSelection;

pub struct PathPlanningGUIPlugin;

impl Plugin for PathPlanningGUIPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_system_set(
            SystemSet::on_update(AppState::PathPlanning)
                .with_system(path_planning_sliders_gui_system.system()).after("main_gui")
        );
    }
}


fn path_planning_sliders_gui_system(egui_context: Res<EguiContext>,
                                    mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                                    mut current_main_gui_values: ResMut<CurrentMainGUIValues>,
                                    mut robot_set_entity_and_info_server: ResMut<RobotSetEntityAndInfoServer>,
                                    mut transform_query: Query<(&mut Transform)>,
                                    key: Res<Input<KeyCode>>,
                                    mut path_planning_gui_values: ResMut<PathPlanningGUIValues>) {
    if key.just_pressed(KeyCode::T) { current_main_gui_values.hide_application_gui = !current_main_gui_values.hide_application_gui; }

    if !current_main_gui_values.hide_application_gui {
        egui::SidePanel::left("path_planning_sliders_panel", 220.).show(egui_context.ctx(), |ui| {
            ui.heading("Path Planning");
            ui.separator();

            egui::ScrollArea::auto_sized().id_source("path_planning_gui_scroll_area").show(ui, |ui| {
                egui::CollapsingHeader::new("Start Joint Value Sliders").default_open(false).show(ui, |ui| {
                    joint_value_sliders_gui_system_generic(ui, &mut lynx_vars, &key, &mut transform_query, "Start Joint Value Sliders", &mut robot_set_entity_and_info_server, 1, false);
                });
                egui::CollapsingHeader::new("Goal Joint Value Sliders").default_open(false).show(ui, |ui| {
                    joint_value_sliders_gui_system_generic(ui, &mut lynx_vars, &key, &mut transform_query, "Goal Joint Value Sliders", &mut robot_set_entity_and_info_server, 2, false);
                });
            });

            ui.group(|ui| {
                if ui.checkbox(&mut path_planning_gui_values.start_visible, "Start Visible").changed() {
                    if !path_planning_gui_values.start_visible { robot_set_entity_and_info_server.hide_robot(1); }
                    else {
                        robot_set_entity_and_info_server.unhide_robot(1);
                        robot_set_entity_and_info_server.reset_link_material_data_whole_robot_set(1);
                    }
                }

                if ui.checkbox(&mut path_planning_gui_values.goal_visible, "Goal Visible").changed() {
                    if !path_planning_gui_values.goal_visible { robot_set_entity_and_info_server.hide_robot(2); }
                    else {
                        robot_set_entity_and_info_server.unhide_robot(2);
                        robot_set_entity_and_info_server.reset_link_material_data_whole_robot_set(2);
                    }
                }
            });

            ui.separator();

            ui.horizontal(|ui| {
                let curr_selection = path_planning_gui_values.curr_path_planner.clone();
                egui::ComboBox::from_id_source("path_planning_combo_box").selected_text(curr_selection.to_string()).show_ui(ui, |ui| {
                    ui.selectable_value(&mut path_planning_gui_values.curr_path_planner, PathPlannerSelection::SPRINT, "SPRINT");
                    ui.selectable_value(&mut path_planning_gui_values.curr_path_planner, PathPlannerSelection::RRTConnect, "RRTConnect");
                    ui.selectable_value(&mut path_planning_gui_values.curr_path_planner, PathPlannerSelection::RRT, "RRT");
                });

                if ui.button("Plan").clicked() {

                }
            });

        });
    }
}
