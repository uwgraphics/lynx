use bevy_egui::egui::{Ui, Color32};
use bevy_egui::{EguiContext, egui};
use bevy::prelude::*;
use crate::app::app_states::res_comps::*;
use crate::utils::utils_vars::prelude::*;
use crate::app::app_utils::gui_utils::environment_selection_choices::*;
use crate::robot_modules::prelude::*;
use crate::prelude::{CollisionEnvironment, robot_directory_includes_base_meshes};
use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;


pub fn gui_environment_selector(ui: &mut Ui,
                                egui_context: &Res<EguiContext>,
                                spawn: &mut ResMut<SpawnNewEnvironment>,
                                current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                                lynx_vars: &mut ResMut<LynxVarsGeneric<'static>>,
                                env_entity_server: &mut ResMut<EnvironmentEntityAndInfoServer>,
                                commands: &mut Commands) {
    let environment_choices = get_all_environment_choices();

    ui.group(|ui| {
        egui::CollapsingHeader::new("Environment Selection").show(ui, |ui| {
            for r in &environment_choices {
                let mut selected = false;
                if current_main_gui_values.curr_selected_environment.is_some() {
                    if current_main_gui_values.curr_selected_environment.as_ref().unwrap().to_string() == r.to_string() {
                        selected = true;
                    }
                }

                if ui.radio(selected, r.clone()).clicked() {
                    current_main_gui_values.curr_selected_environment = Some(r.clone());
                }
            }
        });

        ui.horizontal(|ui| {
            if ui.button("Load Env.").clicked() {
                if current_main_gui_values.curr_selected_environment.is_some() {
                    let curr_selected_environment = current_main_gui_values.curr_selected_environment.as_ref().unwrap().clone();
                    let collision_environment = CollisionEnvironment::new(curr_selected_environment.as_str()).expect("error loading collision environment");

                    let mut robot_world = get_lynx_var_all_mut_refs_generic!(&mut **lynx_vars, RobotWorld, "robot_world");
                    for r in robot_world {
                        r.set_collision_environment(collision_environment.clone());
                    }
                    // robot_world.set_collision_environment(collision_environment);

                    // let robot_world_ = robot_world.clone();
                    // set_or_add_lynx_var_generic!(&mut **lynx_vars, RobotWorld, "robot_world", robot_world_);

                    spawn.0 = true;
                }
            }

            if ui.button("Append Env.").clicked() {
                if current_main_gui_values.curr_selected_environment.is_some() {
                    let curr_selected_environment = current_main_gui_values.curr_selected_environment.as_ref().unwrap().clone();
                    let collision_environment = CollisionEnvironment::new(curr_selected_environment.as_str()).expect("error loading collision environment");

                    let mut robot_world = get_lynx_var_all_mut_refs_generic!(&mut **lynx_vars, RobotWorld, "robot_world");
                    for r in robot_world {
                        r.absorb_collision_environment(collision_environment.clone());
                    }

                    spawn.0 = true;
                }
            }
        });

        ui.horizontal(|ui| {
            if ui.button("Remove Env.").clicked() {
                let mut robot_world = get_lynx_var_all_mut_refs_generic!(&mut **lynx_vars, RobotWorld, "robot_world");
                for r in robot_world {
                    r.remove_collision_environment();
                }

                // let robot_world_ = robot_world.clone();
                // set_or_add_lynx_var_generic!(&mut **lynx_vars, RobotWorld, "robot_world", robot_world_);

                env_entity_server.despawn_all(commands);
            }

            if ui.button("Save Env.").clicked() {
                current_main_gui_values.save_environment_window_open = true;
            }
        });

        if current_main_gui_values.save_environment_window_open {
            let mut current_text = current_main_gui_values.save_environment_text.clone();
            let mut window_open = true;

            egui::Window::new("Save Environment").open(&mut current_main_gui_values.save_environment_window_open).collapsible(false).show(egui_context.ctx(), |ui| {
                let mut m = false;
                m = environment_choices.contains(&current_text);

                ui.horizontal(|ui| {
                    ui.label("name: ");
                    if m { ui.visuals_mut().override_text_color = Some(Color32::RED); }
                    ui.text_edit_singleline(&mut current_text);
                    ui.visuals_mut().override_text_color = None;

                    if ui.button("Save").clicked() {
                        let mut robot_world = get_lynx_var_mut_ref_generic!(&mut **lynx_vars, RobotWorld, "robot_world").expect("error");
                        let mut collision_environment_ = robot_world.get_collision_environment_option_mut_ref();
                        if collision_environment_.is_some() {
                            let mut collision_environment = collision_environment_.as_mut().unwrap();
                            collision_environment.print_summary();
                            collision_environment.save_environment_obbs_metadata(current_text.as_str());
                        }
                        window_open = false;
                    }
                });
            });

            current_main_gui_values.save_environment_text = current_text;
            if current_main_gui_values.save_environment_window_open {
                current_main_gui_values.save_environment_window_open = window_open;
            }
        }

        if ui.checkbox(&mut current_main_gui_values.environment_selectable, "Selectable Env.").clicked() {
            if !current_main_gui_values.environment_selectable {
                env_entity_server.set_all_env_objects_without_focus();
            }
        };

    });
    /*
    ui.horizontal(|ui| {
        ui.text_edit_singleline(&mut current_main_gui_values.save_environment_text);
        if ui.button("Save Env.").clicked() {

        }
    });
    */
}