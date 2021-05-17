use crate::app::app_states::res_comps::{CurrentMainGUIValues, MultipleEnvironmentFocusWindowOpenManager, SpawnNewEnvironment};
use bevy::prelude::{Res, ResMut, Rect, Vec2, Input, MouseButton, EventReader, CursorMoved, Transform, Query, Entity};
use bevy_egui::{EguiContext, egui};
use crate::prelude::{LynxVarsGeneric, CollisionEnvironment};
use std::sync::Arc;
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;
use bevy_egui::egui::{Pos2, Ui};
use rand::Rng;
use termion::input::Events;
use bevy::input::mouse::{MouseButtonInput, MouseMotion};
use crate::robot_modules::prelude::*;
use crate::robot_modules::robot_fk_module::VecOfRobotFKResult;
use crate::app::app_utils::robot_utils::robot_pose_utils::update_robot_link_transforms;
use nalgebra::{Vector3, UnitQuaternion};
use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::{EnvironmentEntityAndInfoServer, EnvObjectEntityPack};
use crate::app::app_utils::math_utils::transform_utils::convert_z_up_idq_to_y_up_bevy_transform;
use crate::utils::utils_se3::prelude::ImplicitDualQuaternion;

pub fn environment_focus_windows_system(mut env_entity_and_info_server: ResMut<EnvironmentEntityAndInfoServer>,
                                        mut multiple_focus_window_open_manager: ResMut<MultipleEnvironmentFocusWindowOpenManager>,
                                        egui_context: Res<EguiContext>,
                                        mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                                        mut current_main_gui_values: ResMut<CurrentMainGUIValues>,
                                        mut mouse_motion_events: EventReader<MouseMotion>,
                                        mut transform_query: Query<(&mut Transform)>,
                                        mut spawn_new_environment: ResMut<SpawnNewEnvironment>) {
    if current_main_gui_values.environment_selectable {
        let mut robot_worlds = get_lynx_var_all_mut_refs_generic!(&mut *lynx_vars, RobotWorld, "robot_world");
        if robot_worlds[0].get_collision_environment_option_mut_ref().is_none() { return; }

        let env_objects_with_focus = env_entity_and_info_server.get_env_objects_with_focus().clone();
        let l = env_objects_with_focus.len();

        if multiple_focus_window_open_manager.window_open_on_last_update.len() == l {
            for i in 0..l {
                if !multiple_focus_window_open_manager.window_open_on_last_update[i] {
                    env_entity_and_info_server.set_env_object_without_focus(env_objects_with_focus[i]);
                }
            }
        }

        multiple_focus_window_open_manager.window_open_on_last_update = Vec::new();
        let env_objects_with_focus = env_entity_and_info_server.get_env_objects_with_focus().clone();
        let l = env_objects_with_focus.len();
        for i in 0..l {
            let env_object_idx = env_objects_with_focus[i];
            let entity_pack_ = env_entity_and_info_server.get_env_object_entity_pack(env_object_idx);
            if entity_pack_.is_err() { return; }

            let entity_pack = entity_pack_.as_ref().unwrap();

            multiple_focus_window_open_manager.window_open_on_last_update.push(true);
            egui::Window::new(format!("Env. Focus Window {}", i + 1))
                .default_height(600.0)
                .open(&mut multiple_focus_window_open_manager.window_open_on_last_update[i])
                .resizable(true)
                .collapsible(false)
                .scroll(true)
                .default_width(50.0)
                .show(egui_context.ctx(), |ui| {
                    if ui.rect_contains_pointer(ui.max_rect()) {
                        env_entity_and_info_server.change_env_object_material_data(env_object_idx, LynxMaterialType::FocusWindowHover);
                    } else {
                        env_entity_and_info_server.change_env_object_material_data_force_away_from_particular_materials(env_object_idx, LynxMaterialType::Focus, vec![LynxMaterialType::FocusWindowHover]);
                    }

                    egui::CollapsingHeader::new("Object Name").default_open(true).show(ui, |ui| {
                        ui.label(robot_worlds[0].get_collision_environment_option_ref().as_ref().unwrap().object_names[env_object_idx].clone());
                    });

                    let translation_labels = vec!["Position X", "Position Y", "Position Z"];
                    let mut translation = robot_worlds[0].get_collision_environment_option_ref().as_ref().unwrap().transforms[env_object_idx].translation.clone();

                    for i in 0..3 {
                        egui::CollapsingHeader::new(translation_labels[i]).default_open(true).show(ui, |ui| {
                            ui.add(egui::Slider::new(&mut translation[i], -5.0..=5.0));
                            for r in &mut robot_worlds {
                                r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].translation[i] = translation[i];
                            }
                            _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);

                            ui.horizontal(|ui| {
                                if ui.button("0.0").clicked() {
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].translation[i] = 0.0;
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }

                                if ui.button("+.1").clicked() {
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].translation[i] += 0.1;
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }

                                if ui.button("-.1").clicked() {
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].translation[i] -= 0.1;
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }

                                if ui.button("+.01").clicked() {
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].translation[i] += 0.01;
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }

                                if ui.button("-.01").clicked() {
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].translation[i] -= 0.01;
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }
                            });
                        });
                    }

                    let quat = robot_worlds[0].get_collision_environment_option_ref().as_ref().unwrap().transforms[env_object_idx].quat.clone();
                    let mut euler_angles_tuple = quat.euler_angles();
                    let mut euler_angles = vec![euler_angles_tuple.0, euler_angles_tuple.1, euler_angles_tuple.2];
                    let rotation_labels = vec!["Rotation X", "Rotation Y", "Rotation Z"];

                    for i in 0..3 {
                        egui::CollapsingHeader::new(rotation_labels[i]).default_open(true).show(ui, |ui| {
                            ui.add(egui::Slider::new(&mut euler_angles[i], -2.0 * std::f64::consts::PI..=2.0 * std::f64::consts::PI));
                            let new_quat = UnitQuaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                            for r in &mut robot_worlds {
                                r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].quat = new_quat.clone();
                            }
                            _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);

                            ui.horizontal(|ui| {
                                if ui.button("0.0").clicked() {
                                    euler_angles[i] = 0.0;
                                    let new_quat = UnitQuaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].quat = new_quat.clone();
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }

                                if ui.button("+pi/2").clicked() {
                                    euler_angles[i] += std::f64::consts::FRAC_PI_2;
                                    let new_quat = UnitQuaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].quat = new_quat.clone();
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }

                                if ui.button("-pi/2").clicked() {
                                    euler_angles[i] -= std::f64::consts::FRAC_PI_2;
                                    let new_quat = UnitQuaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].quat = new_quat.clone();
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }

                                if ui.button("+.1").clicked() {
                                    euler_angles[i] += 0.1;
                                    let new_quat = UnitQuaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].quat = new_quat.clone();
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }

                                if ui.button("-.1").clicked() {
                                    euler_angles[i] -= 0.1;
                                    let new_quat = UnitQuaternion::from_euler_angles(euler_angles[0], euler_angles[1], euler_angles[2]);
                                    for r in &mut robot_worlds {
                                        r.get_collision_environment_option_mut_ref().as_mut().unwrap().transforms[env_object_idx].quat = new_quat.clone();
                                    }
                                    _update_environment_object_transform(entity_pack, &mut robot_worlds, env_object_idx, &mut transform_query);
                                }
                            });
                        });
                    }

                    egui::CollapsingHeader::new("Object Tools").default_open(true).show(ui, |ui| {
                        ui.horizontal(|ui| {
                            if ui.button("Delete").clicked() {
                                // collision_environment.delete_object_by_idx(env_object_idx);
                                for r in &mut robot_worlds {
                                    r.get_collision_environment_option_mut_ref().as_mut().unwrap().delete_object_by_idx(env_object_idx);
                                }
                                spawn_new_environment.0 = true;
                            }

                            if ui.button("Duplicate").clicked() {
                                // collision_environment.duplicate_object_by_idx(env_object_idx);
                                for r in &mut robot_worlds {
                                    r.get_collision_environment_option_mut_ref().as_mut().unwrap().duplicate_object_by_idx(env_object_idx);
                                }
                                spawn_new_environment.0 = true;
                            }
                        });
                    });
                });
        }
    }
    /*
    for i in 0..l {
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
    */
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn _update_environment_object_transform(entity_pack: &EnvObjectEntityPack,
                                        robot_worlds: &mut Vec<&mut RobotWorld>,
                                        env_object_idx: usize,
                                        transform_query: &mut Query<(&mut Transform)>) {
    for r in robot_worlds {
        let mut collision_environment_ = r.get_collision_environment_option_mut_ref();
        if collision_environment_.is_none() { return; } else {
            let mut collision_environment = collision_environment_.as_mut().unwrap();
            collision_environment.update_object_transform_by_idx(env_object_idx, &collision_environment.transforms[env_object_idx].clone());
            if entity_pack.visible_glb_scene_env_object_entity.is_some() {
                let e = entity_pack.visible_glb_scene_env_object_entity.as_ref().unwrap();
                let mut q = transform_query.get_mut(e.clone()).expect("error");
                let new_t = convert_z_up_idq_to_y_up_bevy_transform(&collision_environment.transforms[env_object_idx].clone());
                q.translation = new_t.translation;
                q.rotation = new_t.rotation;
            }

            if entity_pack.standard_material_env_object_entity.is_some() {
                let e = entity_pack.standard_material_env_object_entity.as_ref().unwrap();
                let mut q = transform_query.get_mut(e.clone()).expect("error");
                let new_t = convert_z_up_idq_to_y_up_bevy_transform(&collision_environment.transforms[env_object_idx].clone());
                q.translation = new_t.translation;
                q.rotation = new_t.rotation;
            }

            if entity_pack.invisible_material_env_object_entity.is_some() {
                let e = entity_pack.invisible_material_env_object_entity.as_ref().unwrap();
                let mut q = transform_query.get_mut(e.clone()).expect("error");
                let new_t = convert_z_up_idq_to_y_up_bevy_transform(&collision_environment.transforms[env_object_idx].clone());
                q.translation = new_t.translation;
                q.rotation = new_t.rotation;
            }
        }
    }
}