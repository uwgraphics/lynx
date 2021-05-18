use bevy::prelude::*;
use bevy_egui::{EguiContext, egui};
use crate::utils::utils_vars::prelude::*;
use crate::app::app_states::perpetual::perpetual_res_comps::*;
use crate::app::app_utils::robot_utils::full_robot_set_spawners::spawn_robot_set;
use crate::robot_modules::prelude::*;
use crate::app::app_states::res_comps::*;
use crate::app::app_utils::gui_utils::robot_selection_choices::*;
use bevy_egui::egui::Ui;


pub fn gui_robot_selector(ui: &mut Ui,
                      spawn: &mut ResMut<SpawnNewPhysicalRobot>,
                      current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                      lynx_vars: &mut ResMut<LynxVarsGeneric<'static>>) {
    let robot_choices = get_all_robot_and_configuration_choices();
    ui.group(|ui| {
        egui::CollapsingHeader::new("Robot Selection").show(ui, |ui| {
            for r in robot_choices {
                let selection_string = if r.1.is_some() { r.0.clone() + " -- " + r.1.as_ref().unwrap().as_str() } else { r.0.clone() };
                let mut selected = false;
                if current_main_gui_values.curr_selected_robot_and_configuration.is_some() {
                    current_main_gui_values.curr_selected_robot_set = None;
                    let curr_loaded_robot_and_configuration = current_main_gui_values.curr_selected_robot_and_configuration.as_ref().unwrap();
                    if &curr_loaded_robot_and_configuration.0 == &r.0 && &curr_loaded_robot_and_configuration.1 == &r.1 {
                        selected = true;
                    }
                }

                if ui.radio(selected, selection_string).clicked() {
                    current_main_gui_values.curr_selected_robot_and_configuration = Some(r.clone());
                    current_main_gui_values.curr_selected_robot_set = None;
                }
            }
        });

        if ui.button("Load Robot").clicked() {
            if current_main_gui_values.curr_selected_robot_and_configuration.is_some() {
                let r = current_main_gui_values.curr_selected_robot_and_configuration.as_ref().unwrap();

                let configuration_names = if r.1.is_some() { vec![Some(r.1.as_ref().unwrap().as_str())] } else { vec![None] };
                // let mut lynx_vars_new = LynxVarsGeneric::new_parallel_packaged_with_robot_world(None, vec![r.0.as_str()], configuration_names, None).expect("error");

                // let mut new_robot_world = get_lynx_var_mut_ref_generic!(&mut lynx_vars_new, RobotWorld, "robot_world").expect("error loading robot_world from lynx_vars in robot_spawn_manager_system");
                // let mut robot_world = get_lynx_var_mut_ref_generic!(&mut **lynx_vars, RobotWorld, "robot_world").expect("error loading robot_world from lynx_vars in robot_spawn_manager_system");
                // robot_world.set_robot_set(new_robot_world.get_robot_set_ref().clone());
                // **lynx_vars = lynx_vars_new;

                let robot_set = RobotSet::new(vec![r.0.as_str()], configuration_names.clone()).expect("error loading robot set");
                let mut robot_worlds = get_lynx_var_all_mut_refs_generic!(&mut **lynx_vars, RobotWorld, "robot_world");
                for ro in &mut robot_worlds {
                    ro.update_robot_set_with_given_set(robot_set.clone());
                }

                println!("loaded");

                spawn.0 = true;
            }
        }
    });
}

pub fn gui_robot_set_selector(ui: &mut Ui,
                          spawn: &mut ResMut<SpawnNewPhysicalRobot>,
                          current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                          lynx_vars: &mut ResMut<LynxVarsGeneric<'static>>) {

    let robot_set_choices = get_all_robot_set_choices();

    ui.group(|ui| {
        egui::CollapsingHeader::new("Robot Set Selection").show(ui, |ui| {
            for r in robot_set_choices {
                let selection_string = r.clone();
                let mut selected = false;
                if current_main_gui_values.curr_selected_robot_set.is_some() {
                    current_main_gui_values.curr_selected_robot_and_configuration = None;
                    let curr_selected_robot_set = current_main_gui_values.curr_selected_robot_set.as_ref().unwrap();
                    if &r == curr_selected_robot_set { selected = true; }
                }

                if ui.radio(selected, selection_string).clicked() {
                    current_main_gui_values.curr_selected_robot_set = Some(r.clone());
                    current_main_gui_values.curr_selected_robot_and_configuration = None;
                }
            }
            // ui.separator();
        });

        if ui.button("Load Robot Set").clicked() {
            if current_main_gui_values.curr_selected_robot_set.is_some() {
                let r = current_main_gui_values.curr_selected_robot_set.as_ref().unwrap();
                // let mut lynx_vars_new = LynxVarsGeneric::new_parallel_packaged_with_robot_world_from_set_name(None, r.as_str(), None).expect("error");

                // let mut new_robot_world = get_lynx_var_mut_ref_generic!(&mut lynx_vars_new, RobotWorld, "robot_world").expect("error loading robot_world from lynx_vars in robot_spawn_manager_system");
                // let mut robot_world = get_lynx_var_mut_ref_generic!(&mut **lynx_vars, RobotWorld, "robot_world").expect("error loading robot_world from lynx_vars in robot_spawn_manager_system");

                // robot_world.set_robot_set(new_robot_world.get_robot_set_ref().clone());
                let robot_set = RobotSet::new_from_set_name(r.as_str()).expect("error loading robot set");
                let mut robot_worlds = get_lynx_var_all_mut_refs_generic!(&mut **lynx_vars, RobotWorld, "robot_world");
                for ro in &mut robot_worlds {
                    ro.update_robot_set_with_given_set(robot_set.clone());
                }

                println!("loaded");
                spawn.0 = true;
            }
        }
    });
}