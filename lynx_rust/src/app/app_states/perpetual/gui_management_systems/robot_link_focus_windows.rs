use crate::app::app_states::res_comps::{RobotSetEntityAndInfoServer, SpawnNewPhysicalRobot, CurrentMainGUIValues, MultipleRobotLinkFocusWindowOpenManager};
use bevy::prelude::{Res, ResMut, Rect, Vec2, Input, MouseButton, EventReader, CursorMoved, Transform, Query};
use bevy_egui::{EguiContext, egui};
use crate::prelude::LynxVarsGeneric;
use std::sync::Arc;
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;
use bevy_egui::egui::{Pos2, Ui};
use rand::Rng;
use termion::input::Events;
use bevy::input::mouse::{MouseButtonInput, MouseMotion};
use crate::robot_modules::prelude::*;
use crate::robot_modules::robot_fk_module::VecOfRobotFKResult;
use crate::app::app_utils::robot_utils::robot_pose_utils::update_robot_link_transforms;
use nalgebra::{Vector3};
use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;

pub fn robot_link_focus_windows_system(mut robot_set_entity_and_info_server: ResMut<RobotSetEntityAndInfoServer>,
                                       mut multiple_focus_window_open_manager: ResMut<MultipleRobotLinkFocusWindowOpenManager>,
                                       egui_context: Res<EguiContext>,
                                       mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                                       mut current_main_gui_values: ResMut<CurrentMainGUIValues>,
                                       mut mouse_motion_events: EventReader<MouseMotion>,
                                       mut transform_query: Query<(&mut Transform)>) {
    let mut robot_world = get_lynx_var_mut_ref_generic!(&mut *lynx_vars, RobotWorld, "robot_world").expect("error loading robot_world");
    let mut robot_set = robot_world.get_robot_set_mut_ref();

    let links_with_focus = robot_set_entity_and_info_server.get_links_with_focus_ref().clone();
    let l = links_with_focus.len();

    if multiple_focus_window_open_manager.window_open_on_last_update.len() == l {
        for i in 0..l {
            if !multiple_focus_window_open_manager.window_open_on_last_update[i] {
                robot_set_entity_and_info_server.set_link_without_focus(links_with_focus[i].0, links_with_focus[i].1, links_with_focus[i].2);
            }
        }
    }

    multiple_focus_window_open_manager.window_open_on_last_update = Vec::new();
    let links_with_focus = robot_set_entity_and_info_server.get_links_with_focus_ref().clone();
    let l = links_with_focus.len();
    for i in 0..l {
        multiple_focus_window_open_manager.window_open_on_last_update.push(true);
        egui::Window::new(format!("Focus Window {}", i + 1))
            .open(&mut multiple_focus_window_open_manager.window_open_on_last_update[i])
            .resizable(true).resizable(true)
            .collapsible(false).scroll(true)
            .default_width(50.0)
            .show(egui_context.ctx(), |ui| {
                if ui.rect_contains_pointer(ui.max_rect()) {
                    robot_set_entity_and_info_server.change_link_material_data(links_with_focus[i].0, links_with_focus[i].1, links_with_focus[i].2, LynxMaterialType::FocusWindowHover);
                } else {
                    robot_set_entity_and_info_server.change_link_material_data_force_away_from_particular_materials(links_with_focus[i].0, links_with_focus[i].1, links_with_focus[i].2, LynxMaterialType::Focus, vec![LynxMaterialType::FocusWindowHover]);
                }

                let robot_configuration_module = robot_set.get_robots_ref()[links_with_focus[i].1].get_configuration_module_ref();
                let joint_values = robot_set_entity_and_info_server.get_robot_set_joint_values_ref(links_with_focus[i].0).unwrap();
                let vec_of_fk_results_ = robot_set.compute_fk(&joint_values);
                if vec_of_fk_results_.is_err() { return; }
                let vec_of_fk_results: VecOfRobotFKResult = vec_of_fk_results_.unwrap();

                ui.heading(format!("Robot {}: {}", links_with_focus[i].1, robot_configuration_module.robot_model_module.robot_name));
                display_robot_link_info(ui, &robot_configuration_module, links_with_focus[i]);
                display_robot_link_position_and_orientation(ui, &vec_of_fk_results, links_with_focus[i]);
                display_preceding_joint_sliders(ui, robot_set, &mut robot_set_entity_and_info_server, &mut transform_query, links_with_focus[i]);
            });
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn display_robot_link_info(ui: &mut Ui, robot_configuration_module: &RobotConfigurationModule, link_idxs: (usize, usize, usize)) {
    let link_name = robot_configuration_module.robot_model_module.links[link_idxs.2].name.clone();
    egui::CollapsingHeader::new("Link Name").default_open(true).show(ui, |ui| {
        ui.label(format!("{}", link_name));
    });
    egui::CollapsingHeader::new("Link Index").default_open(true).show(ui, |ui| {
        ui.label(format!("{}", link_idxs.2));
    });
}

pub fn display_robot_link_position_and_orientation(ui: &mut Ui, vec_of_fk_results: &VecOfRobotFKResult, link_idxs: (usize, usize, usize)) {
    let link_transform_ = vec_of_fk_results.get_robot_fk_results_ref()[link_idxs.1].get_link_frames_ref()[link_idxs.2].as_ref();
    if link_transform_.is_none() { return; }
    let link_transform = link_transform_.unwrap();
    let position = &link_transform.translation;
    let orientation = &link_transform.quat;

    egui::CollapsingHeader::new("Position").default_open(true).show(ui, |ui| {
        egui::CollapsingHeader::new("x").default_open(true).show(ui, |ui| {
            ui.label(format!("{:.6}", position[0]));
        });
        egui::CollapsingHeader::new("y").default_open(true).show(ui, |ui| {
            ui.label(format!("{:.6}", position[1]));
        });
        egui::CollapsingHeader::new("z").default_open(true).show(ui, |ui| {
            ui.label(format!("{:.6}", position[2]));
        });
    });

    egui::CollapsingHeader::new("Orientation").default_open(true).show(ui, |ui| {
        egui::CollapsingHeader::new("w").default_open(true).show(ui, |ui| {
            ui.label(format!("{:.6}", orientation.w));
        });
        egui::CollapsingHeader::new("i").default_open(true).show(ui, |ui| {
            ui.label(format!("{:.6}", orientation.i));
        });
        egui::CollapsingHeader::new("j").default_open(true).show(ui, |ui| {
            ui.label(format!("{:.6}", orientation.j));
        });
        egui::CollapsingHeader::new("k").default_open(true).show(ui, |ui| {
            ui.label(format!("{:.6}", orientation.k));
        });
    });
}

pub fn display_preceding_joint_sliders(ui: &mut Ui,
                                       robot_set: &RobotSet,
                                       robot_set_entity_and_info_server: &mut RobotSetEntityAndInfoServer,
                                       transform_query: &mut Query<(&mut Transform)>,
                                       link_idxs: (usize, usize, usize)) {
    let robot_configuration_module = robot_set.get_robots_ref()[link_idxs.1].get_configuration_module_ref();

    let link_idx = link_idxs.2;
    // let preceding_joint_idx_ = robot_configuration_module.robot_model_module.links[link_idx].preceding_joint_idx;
    let preceding_joint_idx_ = _get_preceding_actuated_joint_idx(robot_configuration_module, link_idx);
    if preceding_joint_idx_.is_some() {
        let preceding_joint_idx = preceding_joint_idx_.unwrap();
        let dof_module = robot_set.get_robots_ref()[link_idxs.1].get_dof_module_ref();
        let starting_idx = dof_module.get_input_x_starting_idx_from_joint_idx(preceding_joint_idx);
        let bounds_module = robot_set.get_robots_ref()[link_idxs.1].get_bounds_module_ref();
        let bounds = bounds_module.get_bounds();
        if starting_idx != usize::MAX {
            let joints_copy_ref = dof_module.get_joints_copy_ref();
            let num_joint_dof = joints_copy_ref[preceding_joint_idx].num_dofs;

            let mut robot_set_accumulated_idx = 0 as usize;
            let robot_idx = link_idxs.1;
            for i in 0..robot_idx {
                robot_set_accumulated_idx += robot_set.get_robots_ref()[i].get_dof_module_ref().get_num_dofs();
            }

            if num_joint_dof > 0 {
                egui::CollapsingHeader::new("Preceding Actuated Joint Sliders").default_open(true).show(ui, |ui| {
                    for j in 0..num_joint_dof {
                        let res = dof_module.get_joint_idx_type_and_subidx_from_input_x_idx(starting_idx + j);

                        let mut axis = Vector3::zeros();
                        if res.1 == "rotation".to_string() {
                            axis = dof_module.get_joints_copy_ref()[res.0].dof_rotation_axes[res.2].clone();
                        } else {
                            axis = dof_module.get_joints_copy_ref()[res.0].dof_translation_axes[res.2].clone();
                        }

                        let subjoint_type_str = if res.1 == "rotation" { "R" } else { "P" };
                        egui::CollapsingHeader::new(format!("{}. {}: [{},{},{}] ", dof_module.get_joints_copy_ref()[res.0].name, subjoint_type_str, axis[0], axis[1], axis[2])).default_open(true).show(ui, |ui| {
                            let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(link_idxs.0);
                            let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                            if ui.add(
                                egui::Slider::new(&mut robot_set_joint_values[robot_set_accumulated_idx + starting_idx + j], bounds[starting_idx + j].0.max(-10.0)..=bounds[starting_idx + j].1.min(10.0))
                            ).changed() {
                                let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(link_idxs.0);
                                let robot_set_joint_values = robot_set_joint_values_res.unwrap().clone();

                                update_robot_link_transforms(&robot_set_joint_values, &robot_set, &mut *robot_set_entity_and_info_server, link_idxs.0, transform_query);
                            }
                            ui.horizontal(|ui| {
                                if ui.button("0.0").clicked() {
                                    let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(link_idxs.0);
                                    let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                    robot_set_joint_values[robot_set_accumulated_idx + starting_idx + j] = 0.0;

                                    update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, link_idxs.0, transform_query);
                                }

                                if ui.button("+.1").clicked() {
                                    let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(link_idxs.0);
                                    let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                    robot_set_joint_values[robot_set_accumulated_idx + starting_idx + j] += 0.1;

                                    update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, link_idxs.0, transform_query);
                                }

                                if ui.button("-.1").clicked() {
                                    let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(link_idxs.0);
                                    let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                    robot_set_joint_values[robot_set_accumulated_idx + starting_idx + j] -= 0.1;

                                    update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, link_idxs.0, transform_query);
                                }

                                if ui.button("+.01").clicked() {
                                    let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(link_idxs.0);
                                    let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                    robot_set_joint_values[robot_set_accumulated_idx + starting_idx + j] += 0.01;

                                    update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, link_idxs.0, transform_query);
                                }

                                if ui.button("-.01").clicked() {
                                    let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(link_idxs.0);
                                    let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                    robot_set_joint_values[robot_set_accumulated_idx + starting_idx + j] -= 0.01;

                                    update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, link_idxs.0, transform_query);
                                }
                            });
                        });
                    }
                });
            }
        }
    }
}

fn _get_preceding_actuated_joint_idx(robot_configuration_module: &RobotConfigurationModule, link_idx: usize) -> Option<usize> {
    let links = &robot_configuration_module.robot_model_module.links;
    let joints = &robot_configuration_module.robot_model_module.joints;

    let mut curr_link_idx = link_idx;

    loop {
        let mut joint_idx = links[curr_link_idx].preceding_joint_idx;
        if joint_idx.is_none() { return None; }

        let joint_idx_unwrap = joint_idx.unwrap();
        let num_dofs = joints[joint_idx_unwrap].num_dofs;
        if num_dofs > 0 { return joint_idx; }

        let preceding_link_idx = joints[joint_idx_unwrap].preceding_link_idx;
        if preceding_link_idx == usize::MAX { return None; }

        curr_link_idx = preceding_link_idx;
    }
}