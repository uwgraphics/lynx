use bevy::prelude::*;
use crate::app::app_states::app_states_enum::AppState;
use bevy_egui::{EguiContext, egui};
use crate::utils::utils_vars::prelude::LynxVarsGeneric;
use crate::app::app_states::res_comps::{CurrentMainGUIValues, RobotSetEntityAndInfoServer};
use crate::app::app_states::joint_value_sliders::joint_value_sliders_gui::joint_value_sliders_gui_plugin::joint_value_sliders_gui_system_generic;
use crate::app::app_states::path_planning::path_planning_gui::path_planning_gui_values::PathPlanningGUIValues;
use crate::app::app_states::path_planning::path_planners_enum::PathPlannerSelection;
use crate::path_planning::prelude::*;
use crate::utils::utils_collisions::prelude::*;
use crate::utils::utils_sampling::prelude::*;
use crate::robot_modules::prelude::*;
use crate::utils::utils_path_planning::prelude::*;
use crate::prelude::RecorderArcMutexOption;
use crate::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use crate::app::app_states::path_planning::path_planning_res_comps::PathPlanningPlaybackPack;

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
                                    mut path_planning_gui_values: ResMut<PathPlanningGUIValues>,
                                    mut path_planning_playback_pack: ResMut<PathPlanningPlaybackPack>) {

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

                ui.set_enabled(path_planning_playback_pack.curr_solution.is_some());
                if ui.checkbox(&mut path_planning_playback_pack.display_playback_path, "Solution Visible").clicked() {
                    if !path_planning_playback_pack.display_playback_path {
                        robot_set_entity_and_info_server.hide_robot(3);
                    }
                    else {
                        path_planning_playback_pack.arclength_curr_value = 0.0;
                        robot_set_entity_and_info_server.unhide_robot(3);
                        robot_set_entity_and_info_server.reset_link_material_data_whole_robot_set(3);
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
                    let q_init = robot_set_entity_and_info_server
                        .get_individual_robot_set_entity_and_info_container_ref(1)
                        .expect("error")
                        .get_robot_set_joint_values_ref();

                    let q_goal = robot_set_entity_and_info_server
                        .get_individual_robot_set_entity_and_info_container_ref(2)
                        .expect("error")
                        .get_robot_set_joint_values_ref();

                    let collision = RobotWorldCollisionChecker.to_collision_checker_box();
                    let a = collision.in_collision(q_init, &mut *lynx_vars).expect("error on collision check");
                    let b = collision.in_collision(q_goal, &mut *lynx_vars).expect("error on collision check");

                    let robot_world = get_lynx_var_ref_generic!(&mut *lynx_vars, RobotWorld, "robot_world").expect("error loading robot_world");
                    let base_sampler = robot_world.get_robot_set_ref().to_lynx_float_vec_sampler_box();

                    if a.is_in_collision() || b.is_in_collision() {
                        println!("yes!");
                    } else {
                        match path_planning_gui_values.curr_path_planner {
                            PathPlannerSelection::SPRINT => {
                                let freespace_sampler = FreeSpaceSampler::new(base_sampler.clone(), collision.clone()).to_lynx_multi_float_vec_sampler_box();
                                let sprint_global = SprintGlobal::new(freespace_sampler.clone(), true, 50, SurgeParallelMode::Independent);
                                let sprint_local = SprintLocal::new(collision.clone(), 0.08, false, false).to_local_search_box();
                                let solution = sprint_global.solve_global(q_init, q_goal, &sprint_local, &mut *lynx_vars, &RecorderArcMutexOption::new_none(), &mut TerminationUtilOption::new_none()).expect("error on path plan");
                                match &solution {
                                    PathPlannerResult::SolutionFound(s) => {
                                        println!("SOLUTION FOUND!");
                                        // solution.output_to_file("sprint", "ur5", "test");
                                        path_planning_playback_pack.curr_solution = Some(s.clone());
                                        path_planning_playback_pack.arclength_curr_value = 0.0;
                                        path_planning_playback_pack.display_playback_path = true;
                                        robot_set_entity_and_info_server.unhide_robot(3);
                                    }
                                    PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(s) => {}
                                    PathPlannerResult::SolutionNotFound(_) => {}
                                }
                            }
                            PathPlannerSelection::RRTConnect => {}
                            PathPlannerSelection::RRT => {}
                        }
                    }
                }
            });
        });
    }
}

