use bevy::prelude::*;
use bevy::app::AppBuilder;
use bevy_egui::{EguiContext, egui};
use crate::prelude::LynxVarsGeneric;
use crate::app::app_states::res_comps::{CurrentMainGUIValues, RobotLinkSpawnType, RobotLinkMeshType, RobotSetJointValues, RobotSetType, RobotLinkInfoContainer, RobotSetEntityAndInfoServer};
use crate::app::app_states::app_states_enum::AppState;
use crate::robot_modules::prelude::*;
use crate::utils::utils_math::prelude::vec_to_dvec;
use crate::app::app_utils::robot_utils::robot_pose_utils::*;
use nalgebra::Vector3;
use bevy_egui::egui::Ui;
use crate::utils::utils_files_and_strings::prelude::usize_to_string;

pub struct JointValueSlidersGUIPlugin;

impl Plugin for JointValueSlidersGUIPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_system_set(
            SystemSet::on_update(AppState::JointValueSliders)
                .with_system(joint_value_sliders_gui_system.system().after("main_gui"))
        );
    }
}

pub fn joint_value_sliders_gui_system(egui_context: Res<EguiContext>,
                                      mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                                      mut current_main_gui_values: ResMut<CurrentMainGUIValues>,
                                      mut robot_set_entity_and_info_server: ResMut<RobotSetEntityAndInfoServer>,
                                      mut transform_query: Query<(&mut Transform)>,
                                      key: Res<Input<KeyCode>>,
                                      ) {
    if key.just_pressed(KeyCode::T) { current_main_gui_values.hide_application_gui = !current_main_gui_values.hide_application_gui; }

    if !current_main_gui_values.hide_application_gui {
        egui::SidePanel::left("joint_value_sliders_panel", 220.).show(egui_context.ctx(), |ui| {
            joint_value_sliders_gui_system_generic(ui, &mut lynx_vars, &key, &mut transform_query, "Joint Value Sliders", &mut robot_set_entity_and_info_server, 0, true);
        });
    }
}

pub fn joint_value_sliders_gui_system_generic(ui: &mut Ui,
                                              lynx_vars: &mut ResMut<LynxVarsGeneric<'static>>,
                                              key: &Res<Input<KeyCode>>,
                                              transform_query: &mut Query<(&mut Transform)>,
                                              heading: &str,
                                              robot_set_entity_and_info_server: &mut RobotSetEntityAndInfoServer,
                                              robot_server_vector_idx: usize,
                                              show_heading: bool) {

    let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
    if robot_set_joint_values_res.is_err() { return; }
    let robot_set_joint_values = robot_set_joint_values_res.unwrap();

    egui::ScrollArea::auto_sized().id_source(heading.to_string() + "a").show(ui, |ui| {
        if show_heading {
            ui.heading(heading);
            ui.separator();
        }

        let robot_world = get_lynx_var_ref_generic!(& **lynx_vars, RobotWorld, "robot_world").expect("error loading RobotWorld");
        let robot_set = robot_world.get_robot_set_ref();

        let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
        let robot_set_joint_values = robot_set_joint_values_res.unwrap().clone();

        // let robot_full_state = &vec_to_dvec(&(*physical_robot_set_joint_values.0).to_vec());
        let robot_split_state_res = robot_set.split_full_state_vector_into_robot_state_vectors(&robot_set_joint_values);
        if robot_split_state_res.is_err() { return; }
        let mut robot_split_state = robot_split_state_res.as_ref().unwrap();

        let mut dof_running_total = 1;
        let num_robots = robot_set.get_num_robots();
        for i in 0..num_robots {
            let robot = &robot_set.get_robots_ref()[i];
            let robot_state = &robot_split_state[i];
            if i > 0 { dof_running_total += robot_set.get_robots_ref()[i - 1].get_dof_module_ref().get_num_dofs() }
            egui::CollapsingHeader::new(format!("Robot {:?}: {}", i + 1, robot.get_robot_name_ref())).id_source(heading.to_string() + "b" + usize_to_string(i).as_str()).default_open(true).show(ui, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("Save State").clicked() {

                    }
                    if ui.button("Load State").clicked() {

                    }
                });
                ui.separator();

                let bounds_module = robot.get_bounds_module_ref();
                let bounds = bounds_module.get_bounds();
                let dof_module = robot.get_dof_module_ref();
                let num_dofs = dof_module.get_num_dofs();
                for j in 0..num_dofs {
                    let res = dof_module.get_joint_idx_type_and_subidx_from_input_x_idx(j);
                    let mut axis = Vector3::zeros();
                    if res.1 == "rotation".to_string() {
                        axis = dof_module.get_joints_copy_ref()[res.0].dof_rotation_axes[res.2].clone();
                    } else {
                        axis = dof_module.get_joints_copy_ref()[res.0].dof_translation_axes[res.2].clone();
                    }

                    let subjoint_type_str = if res.1 == "rotation" { "R" } else { "P" };
                    egui::CollapsingHeader::new(format!("{}: {}. {}: [{},{},{}] ", dof_running_total + j, dof_module.get_joints_copy_ref()[res.0].name, subjoint_type_str, axis[0], axis[1], axis[2])).default_open(true).show(ui, |ui| {
                        let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
                        let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                        if ui.add(
                            egui::Slider::new(&mut robot_set_joint_values[dof_running_total + j - 1], bounds[j].0.max(-10.0)..=bounds[j].1.min(10.0))
                        ).changed() {
                            let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
                            let robot_set_joint_values = robot_set_joint_values_res.unwrap().clone();
                            // let full_joint_state = vec_to_dvec(&physical_robot_set_joint_values.0);
                            update_robot_link_transforms(&robot_set_joint_values, &robot_set, robot_set_entity_and_info_server, robot_server_vector_idx, transform_query);
                        }

                        ui.horizontal(|ui| {
                            if ui.button("0.0").clicked() {
                                let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
                                let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                robot_set_joint_values[dof_running_total + j - 1] = 0.0;

                                update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, robot_server_vector_idx, transform_query);
                            }

                            if ui.button("+.1").clicked() {
                                let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
                                let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                robot_set_joint_values[dof_running_total + j - 1] += 0.1;

                                update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, robot_server_vector_idx, transform_query);
                            }

                            if ui.button("-.1").clicked() {
                                let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
                                let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                robot_set_joint_values[dof_running_total + j - 1] -= 0.1;

                                update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, robot_server_vector_idx, transform_query);
                            }

                            if ui.button("+.01").clicked() {
                                let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
                                let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                robot_set_joint_values[dof_running_total + j - 1] += 0.01;

                                update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, robot_server_vector_idx, transform_query);
                            }

                            if ui.button("-.01").clicked() {
                                let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
                                let robot_set_joint_values = robot_set_joint_values_res.unwrap();
                                robot_set_joint_values[dof_running_total + j - 1] -= 0.01;

                                update_robot_link_transforms(&robot_set_joint_values.clone(), &robot_set, &mut *robot_set_entity_and_info_server, robot_server_vector_idx, transform_query);
                            }
                        });
                    });
                }
            });
        }
        ui.separator();
        if ui.button("Reset All").clicked() || key.just_pressed(KeyCode::R) {
            let robot_set_joint_values_res = robot_set_entity_and_info_server.get_robot_set_joint_values_mut_ref(robot_server_vector_idx);
            let robot_set_joint_values = robot_set_joint_values_res.unwrap();

            for j in &mut robot_set_joint_values.iter_mut() { *j = 0.0; }
            // let full_joint_state = vec_to_dvec(&physical_robot_set_joint_values.0);
            let robot_set_joint_values = robot_set_joint_values.clone();

            update_robot_link_transforms(&robot_set_joint_values, &robot_set, robot_set_entity_and_info_server, robot_server_vector_idx, transform_query);
        }
    });

}




/*
pub fn joint_value_sliders_gui_system(egui_context: Res<EguiContext>,
                                      mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                                      mut current_main_gui_values: ResMut<CurrentMainGUIValues>,
                                      mut state: ResMut<State<AppState>>,
                                      mut physical_robot_set_joint_values: ResMut<PhysicalRobotSetJointValues>,
                                      key: Res<Input<KeyCode>>,
                                      mut q: Query<(&mut Transform, &RobotLinkIdx, &RobotIdxInSet, &RobotLinkSpawnType, &RobotLinkMeshType)>) {
    if key.just_pressed(KeyCode::T) { current_main_gui_values.hide_application_gui = !current_main_gui_values.hide_application_gui; }

    if !current_main_gui_values.hide_application_gui {
        egui::SidePanel::left("joint_value_sliders_panel", 220.).show(egui_context.ctx(), |ui| {
            egui::ScrollArea::auto_sized().show(ui, |ui| {
                ui.heading("Joint Value Sliders");
                ui.separator();

                let robot_world = get_lynx_var_ref_generic!(& *lynx_vars, RobotWorld, "robot_world").expect("error loading RobotWorld");
                let robot_set = robot_world.get_robot_set_ref();

                let robot_full_state = &vec_to_dvec(&(*physical_robot_set_joint_values.0).to_vec());
                let robot_split_state_res = robot_set.split_full_state_vector_into_robot_state_vectors(robot_full_state);
                if robot_split_state_res.is_err() { return; }
                let mut robot_split_state = robot_split_state_res.as_ref().unwrap();

                let mut dof_running_total = 1;
                let num_robots = robot_set.get_num_robots();
                for i in 0..num_robots {
                    let robot = &robot_set.get_robots_ref()[i];
                    let robot_state = &robot_split_state[i];
                    if i > 0 { dof_running_total += robot_set.get_robots_ref()[i - 1].get_dof_module_ref().get_num_dofs() }
                    egui::CollapsingHeader::new(format!("Robot {:?}: {}", i + 1, robot.get_robot_name_ref())).default_open(true).show(ui, |ui| {
                        let bounds_module = robot.get_bounds_module_ref();
                        let bounds = bounds_module.get_bounds();
                        let dof_module = robot.get_dof_module_ref();
                        let num_dofs = dof_module.get_num_dofs();
                        for j in 0..num_dofs {
                            let res = dof_module.get_joint_idx_type_and_subidx_from_input_x_idx(j);
                            let mut axis = Vector3::zeros();
                            if res.1 == "rotation".to_string() {
                                axis = dof_module.get_joints_copy_ref()[res.0].dof_rotation_axes[res.2].clone();
                            } else {
                                axis = dof_module.get_joints_copy_ref()[res.0].dof_translation_axes[res.2].clone();
                            }

                            let subjoint_type_str = if res.1 == "rotation" { "R" } else { "P" };
                            egui::CollapsingHeader::new(format!("{}: {}. {}: [{},{},{}] ", dof_running_total + j, dof_module.get_joints_copy_ref()[res.0].name, subjoint_type_str, axis[0], axis[1], axis[2])).default_open(true).show(ui, |ui| {
                                if ui.add(
                                    egui::Slider::new(&mut physical_robot_set_joint_values.0[dof_running_total + j - 1], bounds[j].0.max(-10.0)..=bounds[j].1.min(10.0))
                                ).changed() {
                                    let full_joint_state = vec_to_dvec(&physical_robot_set_joint_values.0);
                                    update_robot_link_transforms(&full_joint_state, &lynx_vars, &mut q, &RobotLinkSpawnType::Physical);
                                }
                            });
                        }
                    });
                }
                ui.separator();
                if ui.button("Reset All").clicked() || key.just_pressed(KeyCode::R) {
                    for j in &mut physical_robot_set_joint_values.0.iter_mut() { *j = 0.0; }
                    let full_joint_state = vec_to_dvec(&physical_robot_set_joint_values.0);
                    update_robot_link_transforms(&full_joint_state, &lynx_vars, &mut q, &RobotLinkSpawnType::Physical);
                }
            });
        });
    }
}
*/

/*
pub fn joint_value_sliders_gui_system_generic(ui: &mut Ui,
                                              lynx_vars: &mut ResMut<LynxVarsGeneric<'static>>,
                                              key: &Res<Input<KeyCode>>,
                                              robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                              robot_set_joint_values_query: &mut Query<(&mut RobotSetJointValues)>,
                                              robot_link_query: &mut Query<(&mut Transform, &RobotLinkInfoContainer)>,
                                              heading: &str,
                                              robot_set_type: RobotSetType,
                                              robot_server_vector_idx: usize) {

    for mut r in &mut robot_set_joint_values_query.iter_mut() {
        if &robot_set_type == &r.robot_set_type {
            let robot_set_joint_values = &mut r.joint_values;

            egui::ScrollArea::auto_sized().show(ui, |ui| {
                ui.heading(heading);
                ui.separator();

                let robot_world = get_lynx_var_ref_generic!(& **lynx_vars, RobotWorld, "robot_world").expect("error loading RobotWorld");
                let robot_set = robot_world.get_robot_set_ref();

                // let robot_full_state = &vec_to_dvec(&(*physical_robot_set_joint_values.0).to_vec());
                let robot_split_state_res = robot_set.split_full_state_vector_into_robot_state_vectors(robot_set_joint_values);
                if robot_split_state_res.is_err() { return; }
                let mut robot_split_state = robot_split_state_res.as_ref().unwrap();

                let mut dof_running_total = 1;
                let num_robots = robot_set.get_num_robots();
                for i in 0..num_robots {
                    let robot = &robot_set.get_robots_ref()[i];
                    let robot_state = &robot_split_state[i];
                    if i > 0 { dof_running_total += robot_set.get_robots_ref()[i - 1].get_dof_module_ref().get_num_dofs() }
                    egui::CollapsingHeader::new(format!("Robot {:?}: {}", i + 1, robot.get_robot_name_ref())).default_open(true).show(ui, |ui| {
                        let bounds_module = robot.get_bounds_module_ref();
                        let bounds = bounds_module.get_bounds();
                        let dof_module = robot.get_dof_module_ref();
                        let num_dofs = dof_module.get_num_dofs();
                        for j in 0..num_dofs {
                            let res = dof_module.get_joint_idx_type_and_subidx_from_input_x_idx(j);
                            let mut axis = Vector3::zeros();
                            if res.1 == "rotation".to_string() {
                                axis = dof_module.get_joints_copy_ref()[res.0].dof_rotation_axes[res.2].clone();
                            } else {
                                axis = dof_module.get_joints_copy_ref()[res.0].dof_translation_axes[res.2].clone();
                            }

                            let subjoint_type_str = if res.1 == "rotation" { "R" } else { "P" };
                            egui::CollapsingHeader::new(format!("{}: {}. {}: [{},{},{}] ", dof_running_total + j, dof_module.get_joints_copy_ref()[res.0].name, subjoint_type_str, axis[0], axis[1], axis[2])).default_open(true).show(ui, |ui| {
                                if ui.add(
                                    egui::Slider::new(&mut robot_set_joint_values[dof_running_total + j - 1], bounds[j].0.max(-10.0)..=bounds[j].1.min(10.0))
                                ).changed() {
                                    // let full_joint_state = vec_to_dvec(&physical_robot_set_joint_values.0);
                                    update_robot_link_transforms2(robot_set_joint_values, &lynx_vars, robot_link_query, &RobotLinkSpawnType::Physical);
                                }
                            });
                        }
                    });
                }
                ui.separator();
                if ui.button("Reset All").clicked() || key.just_pressed(KeyCode::R) {
                    for j in &mut robot_set_joint_values.iter_mut() { *j = 0.0; }
                    // let full_joint_state = vec_to_dvec(&physical_robot_set_joint_values.0);
                    update_robot_link_transforms2(robot_set_joint_values, &lynx_vars, robot_link_query, &RobotLinkSpawnType::Physical);
                }
            });
        }
    }
}
*/