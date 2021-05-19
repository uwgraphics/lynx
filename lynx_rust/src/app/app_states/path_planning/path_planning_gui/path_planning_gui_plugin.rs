use bevy::prelude::*;
use crate::app::app_states::app_states_enum::AppState;
use bevy_egui::{EguiContext, egui};
use crate::utils::utils_vars::prelude::LynxVarsGeneric;
use crate::app::app_states::res_comps::{CurrentMainGUIValues, RobotSetEntityAndInfoServer, SpawnNewPhysicalRobot, SpawnNewEnvironment};
use crate::app::app_states::joint_value_sliders::joint_value_sliders_gui::joint_value_sliders_gui_plugin::joint_value_sliders_gui_system_generic;
use crate::app::app_states::path_planning::path_planning_gui::path_planning_gui_values::PathPlanningGUIValues;
use crate::app::app_states::path_planning::path_planners_enum::PathPlannerSelection;
use crate::path_planning::prelude::*;
use crate::utils::utils_collisions::prelude::*;
use crate::utils::utils_sampling::prelude::*;
use crate::robot_modules::prelude::*;
use crate::utils::utils_path_planning::prelude::*;
use crate::prelude::{RecorderArcMutexOption};
use crate::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use crate::app::app_states::path_planning::path_planning_res_comps::{PathPlanningPlaybackPack, PathPlanningStartAndGoalStatePack};
use bevy_egui::egui::Color32;
use crate::utils::utils_path_planning::path_planning_query::PathPlanningQuery;
use crate::utils::utils_files_and_strings::prelude::*;
use bevy_egui::egui::style::Widgets;


pub struct PathPlanningGUIPlugin;

impl Plugin for PathPlanningGUIPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_system_set(
            SystemSet::on_update(AppState::PathPlanning)
                .with_system(path_planning_gui_system.system()).after("main_gui")
        );
    }
}

fn path_planning_gui_system(egui_context: Res<EguiContext>,
                            mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                            mut current_main_gui_values: ResMut<CurrentMainGUIValues>,
                            mut robot_set_entity_and_info_server: ResMut<RobotSetEntityAndInfoServer>,
                            mut transform_query: Query<(&mut Transform)>,
                            key: Res<Input<KeyCode>>,
                            mut path_planning_gui_values: ResMut<PathPlanningGUIValues>,
                            mut path_planning_playback_pack: ResMut<PathPlanningPlaybackPack>,
                            mut spawn_new_robot: ResMut<SpawnNewPhysicalRobot>,
                            mut spawn_new_environment: ResMut<SpawnNewEnvironment>,
                            mut path_planning_start_and_goal_state_pack: ResMut<PathPlanningStartAndGoalStatePack>) {
    if key.just_pressed(KeyCode::T) && (key.pressed(KeyCode::LShift) || key.pressed(KeyCode::RShift))
        && !(path_planning_gui_values.path_planning_query_load_window_open)
        && !(path_planning_gui_values.path_planning_query_save_window_open) {
        current_main_gui_values.hide_application_gui = !current_main_gui_values.hide_application_gui;
    }

    if !current_main_gui_values.hide_application_gui {
        egui::SidePanel::left("path_planning_sliders_panel", 220.).show(egui_context.ctx(), |ui| {
            ui.heading("Path Planning");
            ui.separator();

            egui::ScrollArea::auto_sized().id_source("path_planning_gui_scroll_area").show(ui, |ui| {
                egui::CollapsingHeader::new("Start Joint Value Sliders").default_open(false).show(ui, |ui| {
                    joint_value_sliders_gui_system_generic(&egui_context, ui, &mut lynx_vars, &key, &mut transform_query, "Start Joint Value Sliders", &mut robot_set_entity_and_info_server, 1, false, &mut current_main_gui_values);
                });
                egui::CollapsingHeader::new("Goal Joint Value Sliders").default_open(false).show(ui, |ui| {
                    joint_value_sliders_gui_system_generic(&egui_context, ui, &mut lynx_vars, &key, &mut transform_query, "Goal Joint Value Sliders", &mut robot_set_entity_and_info_server, 2, false, &mut current_main_gui_values);
                });

                ui.group(|ui| {
                    if ui.checkbox(&mut path_planning_gui_values.start_visible, "Start Visible").changed() {
                        if !path_planning_gui_values.start_visible { robot_set_entity_and_info_server.hide_robot(1); } else {
                            robot_set_entity_and_info_server.unhide_robot(1);
                            robot_set_entity_and_info_server.reset_link_material_data_whole_robot_set(1);
                        }
                    }

                    if ui.checkbox(&mut path_planning_gui_values.goal_visible, "Goal Visible").changed() {
                        if !path_planning_gui_values.goal_visible { robot_set_entity_and_info_server.hide_robot(2); } else {
                            robot_set_entity_and_info_server.unhide_robot(2);
                            robot_set_entity_and_info_server.reset_link_material_data_whole_robot_set(2);
                        }
                    }

                    ui.set_enabled(path_planning_playback_pack.curr_solution.is_some());
                    if ui.checkbox(&mut path_planning_playback_pack.display_playback_path, "Solution Visible").clicked() {
                        if !path_planning_playback_pack.display_playback_path {
                            robot_set_entity_and_info_server.hide_robot(0);
                        } else {
                            path_planning_playback_pack.arclength_curr_value = 0.0;
                            robot_set_entity_and_info_server.unhide_robot(0);
                            robot_set_entity_and_info_server.reset_link_material_data_whole_robot_set(0);
                        }
                    }

                    ui.set_enabled(path_planning_playback_pack.display_playback_path);
                    ui.horizontal(|ui| {
                        ui.label("play position");
                        if ui.add(
                            // egui::Slider::new(&mut path_planning_playback_pack.arclength_curr_value, 0.0..=1.0).show_value(false)
                            egui::DragValue::new(&mut path_planning_playback_pack.arclength_curr_value).speed(0.001).fixed_decimals(6).clamp_range(0.0..=1.10001)
                        ).changed() {
                            path_planning_playback_pack.playing = false;
                        }
                    });

                    ui.horizontal(|ui| {
                        ui.label("play speed");
                        ui.add(
                            egui::Slider::new(&mut path_planning_playback_pack.playback_speed_slider_value, 0.0..=1.0).show_value(false)
                        );
                    });

                    ui.horizontal(|ui| {
                        if path_planning_playback_pack.playing {
                            if ui.button("⏸").clicked() {
                                path_planning_playback_pack.playing = false;
                            }
                        } else {
                            if ui.button("⏵").clicked() {
                                path_planning_playback_pack.playing = true;
                            }
                        }

                        if ui.button("⏮").clicked() {
                            path_planning_playback_pack.arclength_curr_value -= 0.01;
                            path_planning_playback_pack.arclength_curr_value = path_planning_playback_pack.arclength_curr_value.max(0.0);
                            path_planning_playback_pack.playing = false;
                        }

                        if ui.button("⏭").clicked() {
                            path_planning_playback_pack.arclength_curr_value += 0.01;
                            path_planning_playback_pack.arclength_curr_value = path_planning_playback_pack.arclength_curr_value.min(1.0);
                            path_planning_playback_pack.playing = false;
                        }
                    });
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

                        if a.is_in_collision() || b.is_in_collision() || (q_init == q_goal) {
                            println!("yes!");
                        } else {
                            match path_planning_gui_values.curr_path_planner {
                                PathPlannerSelection::SPRINT => {
                                    let freespace_sampler = FreeSpaceSampler::new(base_sampler.clone(), collision.clone()).to_lynx_multi_float_vec_sampler_box();
                                    let sprint_global = SprintGlobal::new(freespace_sampler.clone(), true, 50, SurgeParallelMode::Independent);
                                    let sprint_local = SprintLocal::new(collision.clone(), 0.085, false, false).to_local_search_box();
                                    let solution = sprint_global.solve_global(q_init, q_goal, &sprint_local, &mut *lynx_vars, &RecorderArcMutexOption::new_none(), &mut TerminationUtilOption::new_none()).expect("error on path plan");
                                    match &solution {
                                        PathPlannerResult::SolutionFound(s) => {
                                            println!("SOLUTION FOUND!");
                                            // solution.output_to_file("sprint", "ur5", "test");
                                            path_planning_playback_pack.curr_solution = Some(s.clone());
                                            path_planning_playback_pack.arclength_curr_value = 0.0;
                                            path_planning_playback_pack.display_playback_path = true;
                                            path_planning_playback_pack.playing = true;
                                            robot_set_entity_and_info_server.unhide_robot(0);
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

                ui.separator();
                ui.group(|ui| {
                    if ui.button("Save Path Planning Query").clicked() {
                        path_planning_gui_values.path_planning_query_save_window_open = true;
                        path_planning_gui_values.path_planning_query_load_window_open = false;
                    }

                    if ui.button("Load Path Planning Query").clicked() {
                        path_planning_gui_values.path_planning_query_load_window_open = true;
                        path_planning_gui_values.path_planning_query_save_window_open = false;
                    }
                });

                if path_planning_gui_values.path_planning_query_save_window_open {
                    let mut path_planning_query_save_window_open = path_planning_gui_values.path_planning_query_save_window_open.clone();
                    let mut path_planning_query_save_string = path_planning_gui_values.path_planning_query_save_string.clone();
                    egui::Window::new("Save Path Planning Query")
                        .open(&mut path_planning_gui_values.path_planning_query_save_window_open)
                        .collapsible(false)
                        .show(egui_context.ctx(), |ui| {
                            ui.horizontal(|ui| {
                                let all_options = _get_all_path_planning_query_options(false);
                                if all_options.contains(&mut path_planning_query_save_string) {
                                    ui.style_mut().visuals.override_text_color = Some(Color32::RED);
                                }
                                ui.text_edit_singleline(&mut path_planning_query_save_string);
                                ui.style_mut().visuals.override_text_color = None;
                                if ui.button("Save").clicked() {
                                    let robot_world = lynx_vars.get_robot_world_ref(None).expect("error loading robot world").clone();
                                    let start_state = robot_set_entity_and_info_server.get_all_individual_robot_set_entity_and_info_containers_ref()[1].get_robot_set_joint_values_ref().clone();
                                    let goal_state = robot_set_entity_and_info_server.get_all_individual_robot_set_entity_and_info_containers_ref()[2].get_robot_set_joint_values_ref().clone();
                                    let ppq = PathPlanningQuery::new(robot_world, start_state, goal_state);
                                    let json_string = ppq.get_json_string();

                                    let fp_to_dir = get_path_to_src() + "/autogenerated_metadata/path_planning_queries/";
                                    let file_name = path_planning_query_save_string.clone() + ".json";
                                    write_string_to_file(fp_to_dir, file_name, json_string, true);

                                    path_planning_query_save_window_open = false;
                                }
                            });
                        });
                    if !path_planning_query_save_window_open { path_planning_gui_values.path_planning_query_save_window_open = false; }
                    path_planning_gui_values.path_planning_query_save_string = path_planning_query_save_string.clone();
                }

                if path_planning_gui_values.path_planning_query_load_window_open {
                    let mut path_planning_query_load_window_open = path_planning_gui_values.path_planning_query_load_window_open.clone();
                    let mut path_planning_query_load_string = path_planning_gui_values.path_planning_query_load_string.clone();
                    let all_options = _get_all_path_planning_query_options(true);
                    egui::Window::new("Load Path Planning Query")
                        .open(&mut path_planning_gui_values.path_planning_query_load_window_open)
                        .collapsible(false)
                        .default_width(30.0)
                        .show(egui_context.ctx(), |ui| {
                            egui::ScrollArea::auto_sized().show(ui, |ui| {
                                for s in &all_options {
                                    let mut checked = false;
                                    if s == &path_planning_query_load_string { checked = true; }
                                    if ui.selectable_label(checked, s).clicked() {
                                        path_planning_query_load_string = s.clone();
                                    }
                                }
                            });
                            ui.separator();
                            if ui.button("Load").clicked() {
                                if !(path_planning_query_load_string == "".to_string()) {
                                    let fp = get_path_to_src() + "/autogenerated_metadata/path_planning_queries/" + path_planning_query_load_string.as_str() + ".json";
                                    let json_string = read_file_contents(fp.clone()).expect(format!("trouble reading file path {}", fp).as_str());
                                    let path_planning_query = PathPlanningQuery::new_from_json_string(json_string).expect("error");

                                    let mut robot_worlds = get_lynx_var_all_mut_refs_generic!(&mut *lynx_vars, RobotWorld, "robot_world");
                                    for r in robot_worlds {
                                        *r = path_planning_query.robot_world.clone();
                                    }

                                    spawn_new_environment.0 = true;
                                    spawn_new_robot.0 = true;
                                    path_planning_start_and_goal_state_pack.start_state = Some(path_planning_query.start_state.clone());
                                    path_planning_start_and_goal_state_pack.goal_state = Some(path_planning_query.goal_state.clone());

                                    path_planning_query_load_window_open = false;
                                }
                            }
                        });
                    if !path_planning_query_load_window_open { path_planning_gui_values.path_planning_query_load_window_open = false; }
                    path_planning_gui_values.path_planning_query_load_string = path_planning_query_load_string.clone();
                }
            });
        });
    }
}

fn _get_all_path_planning_query_options(sort: bool) -> Vec<String> {
    let fp = get_path_to_src() + "/autogenerated_metadata/path_planning_queries";
    let all_files = get_all_files_in_directory(fp);

    let mut out_vec = Vec::new();
    for a in all_files {
        out_vec.push(get_filename_without_extension(a.clone()));
    }

    if sort { out_vec.sort(); }

    out_vec
}
