use bevy_egui::egui::{Ui, Color32};
use bevy_egui::{egui, EguiContext};
use bevy::prelude::{ResMut, Res, Commands, Assets, Mesh, StandardMaterial, Vec3, Color, Query, Entity};
use crate::app::app_states::res_comps::{CurrentMainGUIValues, RobotSetEntityAndInfoServer, CollisionWindowVariables};
use crate::robot_modules::prelude::RobotWorld;
use crate::robot_modules::robot_fk_module::VecOfRobotFKResult;
use crate::prelude::{LinkGeometryType, VecOfContactCheckMultipleResult};
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;
use crate::app::app_utils::drawing_utils::line_drawing::{draw_line_lynx_space, LineType};
use termion::{style, color};
use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;
use crate::utils::utils_files_and_strings::string_utils::usize_to_string;

pub fn collision_information_selector(ui: &mut Ui,
                                      egui_context: &Res<EguiContext>,
                                      current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                                      collision_window_variables: &mut ResMut<CollisionWindowVariables>,
                                      robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                      env_entity_and_info_server: &mut ResMut<EnvironmentEntityAndInfoServer>,
                                      robot_world: &mut RobotWorld) {
    let l = robot_set_entity_and_info_server.get_num_individual_robot_set_entity_and_info_containers();
    for i in 0..l {
        robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(i, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
        robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(i, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
        robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(i, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
    }
    let l = env_entity_and_info_server.get_num_objects();
    for i in 0..l {
        env_entity_and_info_server.reset_env_object_material_data_force_away_from_particular_materials(i, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
        env_entity_and_info_server.reset_env_object_material_data_force_away_from_particular_materials(i, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
        env_entity_and_info_server.reset_env_object_material_data_force_away_from_particular_materials(i, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
    }

    // env_entity_and_info_server.reset_env_object_material_data_force_away_from_particular_materials()

    egui::CollapsingHeader::new("Collision Info").show(ui, |ui| {
        ui.checkbox(&mut current_main_gui_values.display_self_collision_information, "Self Collision Info");
        if current_main_gui_values.display_self_collision_information {
            _self_collision_information_window(egui_context,
                                               current_main_gui_values,
                                               collision_window_variables,
                                               robot_set_entity_and_info_server,
                                               robot_world);
        }
        ui.checkbox(&mut current_main_gui_values.display_multi_robot_collision_information, "Multi-Robot Collision Info");
        if current_main_gui_values.display_multi_robot_collision_information {
            _multi_robot_collision_information_window(egui_context,
                                                      current_main_gui_values,
                                                      collision_window_variables,
                                                      robot_set_entity_and_info_server,
                                                      robot_world);
        }
        ui.checkbox(&mut current_main_gui_values.display_environment_collision_information, "Environment Collision Info");
        if current_main_gui_values.display_environment_collision_information {
            _environment_collision_information_window(egui_context,
                                                      current_main_gui_values,
                                                      collision_window_variables,
                                                      robot_set_entity_and_info_server,
                                                      env_entity_and_info_server,
                                                      robot_world);
        }
        /*
        ui.checkbox(&mut current_main_gui_values.display_collision_information, "Display Collision Info");
        if current_main_gui_values.display_collision_information {
            _unified_collision_information_window(egui_context,
                                                  current_main_gui_values,
                                                  collision_window_variables,
                                                  robot_set_entity_and_info_server,
                                                  env_entity_and_info_server,
                                                  robot_world);
        }
        */
    });

    /*
    if !current_main_gui_values.display_self_collision_information {
        let l = robot_set_entity_and_info_server.get_num_individual_robot_set_entity_and_info_containers();
        for i in 0..l {
            robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(i, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
        }
    }
    */
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn _self_collision_information_window(egui_context: &Res<EguiContext>,
                                      current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                                      collision_window_variables: &mut ResMut<CollisionWindowVariables>,
                                      robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                      robot_world: &mut RobotWorld) {

    egui::Window::new("Self Collision Info").collapsible(false).open(&mut current_main_gui_values.display_self_collision_information).show(egui_context.ctx(), |ui| {
        egui::ScrollArea::auto_sized().show(ui, |ui| {
            let num_robots_on_server = robot_set_entity_and_info_server.get_num_individual_robot_set_entity_and_info_containers();
            for i in 0..num_robots_on_server {
                if robot_set_entity_and_info_server.is_robot_hidden(i) { continue; }
                let ind = robot_set_entity_and_info_server.get_individual_robot_set_entity_and_info_container_ref(i).unwrap();
                let joint_values = ind.get_robot_set_joint_values_ref();
                let vec_of_fk_results_ = robot_world.get_robot_set_ref().compute_fk(joint_values);
                if vec_of_fk_results_.is_err() { return; }
                let vec_of_fk_results: VecOfRobotFKResult = vec_of_fk_results_.unwrap();

                /*
                egui::ComboBox::from_id_source("geometry type")
                    .selected_text(current_main_gui_values.collision_panel_link_geometry_type.to_string())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut current_main_gui_values.collision_panel_link_geometry_type, LinkGeometryType::OBBs, "OBBs");
                        ui.selectable_value(&mut current_main_gui_values.collision_panel_link_geometry_type, LinkGeometryType::ConvexShapes, "ConvexShapes");
                        ui.selectable_value(&mut current_main_gui_values.collision_panel_link_geometry_type, LinkGeometryType::OBBSubcomponents, "OBBSubs");
                        ui.selectable_value(&mut current_main_gui_values.collision_panel_link_geometry_type, LinkGeometryType::ConvexShapeSubcomponents, "ConvexShapeSubs");
                    });
                   */
                _self_collision_info_display_for_one_robot_set(ui, robot_world, &vec_of_fk_results, i, robot_set_entity_and_info_server, collision_window_variables);
            }
        });
    });
}

fn _self_collision_info_display_for_one_robot_set(ui: &mut Ui,
                                                  robot_world: &mut RobotWorld,
                                                  vec_of_fk_results: &VecOfRobotFKResult,
                                                  robot_server_vector_idx: usize,
                                                  robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                                  collision_window_variables: &mut ResMut<CollisionWindowVariables>) {
    /*
    for l in line_query.iter() {
        let e: Entity = l.0;
        let line_type: &LineType = l.1;
        if line_type == &LineType::Visualization {
            commands.entity(e).despawn();
        }
    }
    */
    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);

    let vec_of_contact_check_multiple_result = robot_world.get_robot_set_mut_ref().self_contact_check(&vec_of_fk_results, collision_window_variables.self_collision_link_geometry_type.clone(), false, Some(0.5)).expect("error with self contact info");

    let in_collision = vec_of_contact_check_multiple_result.in_collision();
    ui.horizontal(|ui| {
        if in_collision {
            ui.visuals_mut().override_text_color = Some(Color32::RED);
            ui.heading("In Self Collision!");
            ui.visuals_mut().override_text_color = None;
        } else {
            ui.visuals_mut().override_text_color = Some(Color32::GREEN);
            ui.heading("No Self Collision");
            ui.visuals_mut().override_text_color = None;
        }

        egui::ComboBox::from_id_source("geometry type1".to_string() + usize_to_string(robot_server_vector_idx).as_str())
            .selected_text(collision_window_variables.self_collision_link_geometry_type.to_string())
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut collision_window_variables.self_collision_link_geometry_type, LinkGeometryType::OBBs, "OBBs");
                ui.selectable_value(&mut collision_window_variables.self_collision_link_geometry_type, LinkGeometryType::ConvexShapes, "ConvexShapes");
                ui.selectable_value(&mut collision_window_variables.self_collision_link_geometry_type, LinkGeometryType::OBBSubcomponents, "OBBSubs");
                ui.selectable_value(&mut collision_window_variables.self_collision_link_geometry_type, LinkGeometryType::ConvexShapeSubcomponents, "ConvexShapeSubs");
            });
    });
    ui.separator();
    ui.horizontal(|ui| {
        ui.checkbox(&mut collision_window_variables.reset_collision_tensor_checkbox, "Reset Collision Tensor");
        if ui.button("Enter").clicked() && collision_window_variables.reset_collision_tensor_checkbox {
            let l = robot_world.get_robot_set_ref().get_num_robots();
            for i in 0..l {
                let res = robot_world.get_robot_set_mut_ref().get_robots_mut_ref()[i].get_core_collision_module_mut_ref().revert_skip_collision_check_tensor(collision_window_variables.self_collision_link_geometry_type.clone());
                if res.is_err() {
                    println!("{}{}WARNING: failed to revert_skip_collision_check_tensor{}", style::Bold, color::Fg(color::Yellow), style::Reset);
                } else {
                    robot_world.get_robot_set_mut_ref().get_robots_mut_ref()[i].get_core_collision_module_mut_ref().load_link_skip_collision_check_tensor(collision_window_variables.self_collision_link_geometry_type.clone());
                }
            }
        }
    });
    ui.separator();

    let vec = vec_of_contact_check_multiple_result.get_contact_check_multiple_results_ref();
    let l1 = vec.len();
    for i in 0..l1 {
        ui.label(format!("Robot {}: {}", i, robot_world.get_robot_set_ref().get_robots_ref()[i].get_robot_name_ref()));
        egui::Grid::new("grid1".to_string() + usize_to_string(robot_server_vector_idx).as_str()).striped(true).show(ui, |ui| {
            let v = &vec[i];
            let res = v.get_contact_check_multiple_info_ref();
            let contacts = res.get_contact_check_contacts();
            let idxs = res.get_contact_check_idxs();
            let l2 = contacts.len();
            for j in 0..l2 {
                if contacts[j].depth > 0.0 {
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, i, idxs[j][0][0], LynxMaterialType::Collision);
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, i, idxs[j][1][0], LynxMaterialType::Collision);
                }
                let link_idx1: usize = idxs[j][0][0];
                let link_idx2: usize = idxs[j][1][0];
                let link_name1 = robot_world.get_robot_set_ref().get_robots_ref()[i].get_configuration_module_ref().robot_model_module.links[link_idx1].name.clone();
                let link_name2 = robot_world.get_robot_set_ref().get_robots_ref()[i].get_configuration_module_ref().robot_model_module.links[link_idx2].name.clone();
                let in_collision = contacts[j].depth > 0.0;

                if in_collision {
                    ui.visuals_mut().override_text_color = Some(Color32::RED);
                }
                if ui.radio(false, "").is_pointer_button_down_on() {
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, i, link_idx1, LynxMaterialType::CollisionHighlight1);
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, i, link_idx2, LynxMaterialType::CollisionHighlight2);
                    let p1 = contacts[j].world1.clone();
                    let p2 = contacts[j].world2.clone();
                    let color = if in_collision { Color::rgb(1.0, 0.0, 0.0) } else { Color::rgb(0.0, 0.0, 0.0) };
                    // draw_line_lynx_space(commands, meshes, materials, Vec3::new(p1[0] as f32, p1[1] as f32, p1[2] as f32), Vec3::new(p2[0] as f32, p2[1] as f32, p2[2] as f32), color, 6.0, LineType::Visualization);
                }
                ui.label(format!("{}: {}, {}      distance: {:.3} m", j, link_name1, link_name2, -contacts[j].depth));
                ui.visuals_mut().override_text_color = None;
                if in_collision {
                    let button = ui.button("not collision");
                    if button.clicked() {
                        robot_world.get_robot_set_mut_ref().get_robots_mut_ref()[i].get_core_collision_module_mut_ref().add_manual_collision_check_skip_between_links(collision_window_variables.self_collision_link_geometry_type.clone(), idxs[j].clone());
                        robot_world.get_robot_set_mut_ref().get_robots_mut_ref()[i].get_core_collision_module_mut_ref().load_link_skip_collision_check_tensor(collision_window_variables.self_collision_link_geometry_type.clone());
                    }
                    if button.hovered() || button.clicked() {
                        robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, i, link_idx1, LynxMaterialType::CollisionHighlight1);
                        robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, i, link_idx2, LynxMaterialType::CollisionHighlight2);
                        let p1 = contacts[j].world1.clone();
                        let p2 = contacts[j].world2.clone();
                        let color = if in_collision { Color::rgb(1.0, 0.0, 0.0) } else { Color::rgb(0.0, 0.0, 0.0) };
                        // draw_line_lynx_space(commands, meshes, materials, Vec3::new(p1[0] as f32, p1[1] as f32, p1[2] as f32), Vec3::new(p2[0] as f32, p2[1] as f32, p2[2] as f32), color, 6.0, LineType::Visualization);
                    }
                }
                ui.end_row();
            }
        });
        ui.separator();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn _multi_robot_collision_information_window(egui_context: &Res<EguiContext>,
                                             current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                                             collision_window_variables: &mut ResMut<CollisionWindowVariables>,
                                             robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                             robot_world: &mut RobotWorld) {
    egui::Window::new("Multi Robot Collision Info").collapsible(false).open(&mut current_main_gui_values.display_multi_robot_collision_information).show(egui_context.ctx(), |ui| {
        egui::ScrollArea::auto_sized().show(ui, |ui| {
            let num_robots_on_server = robot_set_entity_and_info_server.get_num_individual_robot_set_entity_and_info_containers();
            for i in 0..num_robots_on_server {
                if robot_set_entity_and_info_server.is_robot_hidden(i) { continue; }
                let ind = robot_set_entity_and_info_server.get_individual_robot_set_entity_and_info_container_ref(i).unwrap();
                let joint_values = ind.get_robot_set_joint_values_ref();
                let vec_of_fk_results_ = robot_world.get_robot_set_ref().compute_fk(joint_values);
                if vec_of_fk_results_.is_err() { return; }
                let vec_of_fk_results: VecOfRobotFKResult = vec_of_fk_results_.unwrap();

                _multi_robot_collision_info_display_for_one_robot_set(ui, robot_world, &vec_of_fk_results, i, robot_set_entity_and_info_server, collision_window_variables);
            }
        });
    });
}

fn _multi_robot_collision_info_display_for_one_robot_set(ui: &mut Ui,
                                                  robot_world: &mut RobotWorld,
                                                  vec_of_fk_results: &VecOfRobotFKResult,
                                                  robot_server_vector_idx: usize,
                                                  robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                                  collision_window_variables: &mut ResMut<CollisionWindowVariables>) {

    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight2, LynxMaterialType::CollisionHighlight1]);
    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);

    let vec_of_contact_check_multiple_result = robot_world.get_robot_set_mut_ref().multi_robot_contact_check(&vec_of_fk_results, collision_window_variables.multi_robot_collision_link_geometry_type.clone(), false, Some(0.5)).expect("error with multi robot contact info");
    let in_collision = vec_of_contact_check_multiple_result.in_collision();
    ui.horizontal(|ui| {
        if in_collision {
            ui.visuals_mut().override_text_color = Some(Color32::RED);
            ui.heading("Multi-robot Collision!");
            ui.visuals_mut().override_text_color = None;
        } else {
            ui.visuals_mut().override_text_color = Some(Color32::GREEN);
            ui.heading("No Multi-robot Collision");
            ui.visuals_mut().override_text_color = None;
        }

        egui::ComboBox::from_id_source("geometry type2".to_string() + usize_to_string(robot_server_vector_idx).as_str())
            .selected_text(collision_window_variables.multi_robot_collision_link_geometry_type.to_string())
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut collision_window_variables.multi_robot_collision_link_geometry_type, LinkGeometryType::OBBs, "OBBs");
                ui.selectable_value(&mut collision_window_variables.multi_robot_collision_link_geometry_type, LinkGeometryType::ConvexShapes, "ConvexShapes");
                ui.selectable_value(&mut collision_window_variables.multi_robot_collision_link_geometry_type, LinkGeometryType::OBBSubcomponents, "OBBSubs");
                ui.selectable_value(&mut collision_window_variables.multi_robot_collision_link_geometry_type, LinkGeometryType::ConvexShapeSubcomponents, "ConvexShapeSubs");
            });
    });
    ui.separator();

    let vec = vec_of_contact_check_multiple_result.get_contact_check_multiple_results_ref();
    let order = robot_world.get_robot_set_mut_ref().get_robot_set_result_vector_idxs_to_robot_idxs_ref().clone();
    let l1 = vec.len();
    for i in 0..l1 {
        let o = &order[i];
        ui.label(format!("Between Robot {}: {} and Robot {}: {}", o.0, robot_world.get_robot_set_ref().get_robots_ref()[o.0].get_robot_name_ref().clone(), o.1, robot_world.get_robot_set_ref().get_robots_ref()[o.1].get_robot_name_ref().clone() ));
        egui::Grid::new("grid2".to_string() + usize_to_string(robot_server_vector_idx).as_str()).striped(true).show(ui, |ui| {
            let v = &vec[i];
            let res = v.get_contact_check_multiple_info_ref();
            let contacts = res.get_contact_check_contacts();
            let idxs = res.get_contact_check_idxs();
            let l2 = contacts.len();
            for j in 0..l2 {
                if contacts[j].depth > 0.0 {
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, o.0, idxs[j][0][0], LynxMaterialType::Collision);
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, o.1, idxs[j][1][0], LynxMaterialType::Collision);
                }
                let link_idx1: usize = idxs[j][0][0];
                let link_idx2: usize = idxs[j][1][0];
                let link_name1 = robot_world.get_robot_set_ref().get_robots_ref()[o.0].get_configuration_module_ref().robot_model_module.links[link_idx1].name.clone();
                let link_name2 = robot_world.get_robot_set_ref().get_robots_ref()[o.1].get_configuration_module_ref().robot_model_module.links[link_idx2].name.clone();
                let in_collision = contacts[j].depth > 0.0;

                if in_collision {
                    ui.visuals_mut().override_text_color = Some(Color32::RED);
                }
                if ui.radio(false, "").is_pointer_button_down_on() {
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, o.0, link_idx1, LynxMaterialType::CollisionHighlight1);
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, o.1, link_idx2, LynxMaterialType::CollisionHighlight2);
                    let p1 = contacts[j].world1.clone();
                    let p2 = contacts[j].world2.clone();
                    let color = if in_collision { Color::rgb(1.0, 0.0, 0.0) } else { Color::rgb(0.0, 0.0, 0.0) };
                    // draw_line_lynx_space(commands, meshes, materials, Vec3::new(p1[0] as f32, p1[1] as f32, p1[2] as f32), Vec3::new(p2[0] as f32, p2[1] as f32, p2[2] as f32), color, 6.0, LineType::Visualization);
                }
                ui.label(format!("{}: {}, {}      distance: {:.3} m", j, link_name1, link_name2, -contacts[j].depth));
                ui.visuals_mut().override_text_color = None;
                ui.end_row();
            }
        });
        ui.separator();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn _environment_collision_information_window(egui_context: &Res<EguiContext>,
                                             current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                                             collision_window_variables: &mut ResMut<CollisionWindowVariables>,
                                             robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                             env_entity_and_info_server: &mut ResMut<EnvironmentEntityAndInfoServer>,
                                             robot_world: &mut RobotWorld) {
    egui::Window::new("Environment Collision Info").collapsible(false).open(&mut current_main_gui_values.display_environment_collision_information).show(egui_context.ctx(), |ui| {
        egui::ScrollArea::auto_sized().show(ui, |ui| {
            let num_robots_on_server = robot_set_entity_and_info_server.get_num_individual_robot_set_entity_and_info_containers();
            for i in 0..num_robots_on_server {
                if robot_set_entity_and_info_server.is_robot_hidden(i) { continue; }
                let ind = robot_set_entity_and_info_server.get_individual_robot_set_entity_and_info_container_ref(i).unwrap();
                let joint_values = ind.get_robot_set_joint_values_ref();
                let vec_of_fk_results_ = robot_world.get_robot_set_ref().compute_fk(joint_values);
                if vec_of_fk_results_.is_err() { return; }
                let vec_of_fk_results: VecOfRobotFKResult = vec_of_fk_results_.unwrap();

                _environment_collision_info_display_for_one_robot_set(ui, robot_world, &vec_of_fk_results, i, robot_set_entity_and_info_server, env_entity_and_info_server, collision_window_variables);
            }
        });
    });
}

fn _environment_collision_info_display_for_one_robot_set(ui: &mut Ui,
                                                         robot_world: &mut RobotWorld,
                                                         vec_of_fk_results: &VecOfRobotFKResult,
                                                         robot_server_vector_idx: usize,
                                                         robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                                         env_entity_and_info_server: &mut ResMut<EnvironmentEntityAndInfoServer>,
                                                         collision_window_variables: &mut ResMut<CollisionWindowVariables>) {

    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight2, LynxMaterialType::CollisionHighlight1]);
    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);
    // robot_set_entity_and_info_server.reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(robot_server_vector_idx, vec![LynxMaterialType::Collision, LynxMaterialType::CollisionHighlight1, LynxMaterialType::CollisionHighlight2]);

    let vec_of_contact_check_multiple_result = robot_world.environment_contact_check(&vec_of_fk_results, collision_window_variables.environment_collision_link_geometry_type.clone(), false, Some(0.5)).expect("error with multi robot contact info");
    let in_collision = vec_of_contact_check_multiple_result.in_collision();
    ui.horizontal(|ui| {
        if in_collision {
            ui.visuals_mut().override_text_color = Some(Color32::RED);
            ui.heading("Environment Collision!");
            ui.visuals_mut().override_text_color = None;
        } else {
            ui.visuals_mut().override_text_color = Some(Color32::GREEN);
            ui.heading("No Environment Collision");
            ui.visuals_mut().override_text_color = None;
        }

        egui::ComboBox::from_id_source("geometry type3".to_string() + usize_to_string(robot_server_vector_idx).as_str())
            .selected_text(collision_window_variables.environment_collision_link_geometry_type.to_string())
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut collision_window_variables.environment_collision_link_geometry_type, LinkGeometryType::OBBs, "OBBs");
                ui.selectable_value(&mut collision_window_variables.environment_collision_link_geometry_type, LinkGeometryType::ConvexShapes, "ConvexShapes");
                ui.selectable_value(&mut collision_window_variables.environment_collision_link_geometry_type, LinkGeometryType::OBBSubcomponents, "OBBSubs");
                ui.selectable_value(&mut collision_window_variables.environment_collision_link_geometry_type, LinkGeometryType::ConvexShapeSubcomponents, "ConvexShapeSubs");
            });
    });
    ui.separator();

    if robot_world.get_collision_environment_option_ref().is_none() { return; }

    let vec = vec_of_contact_check_multiple_result.get_contact_check_multiple_results_ref();
    let l1 = vec.len();
    for i in 0..l1 {
        ui.label(format!("Robot {}: {}", i, robot_world.get_robot_set_ref().get_robots_ref()[i].get_robot_name_ref()));
        egui::Grid::new("grid3".to_string() + usize_to_string(robot_server_vector_idx).as_str()).striped(true).show(ui, |ui| {
            let v = &vec[i];
            let res = v.get_contact_check_multiple_info_ref();
            let contacts = res.get_contact_check_contacts();
            let idxs = res.get_contact_check_idxs();
            let l2 = contacts.len();
            for j in 0..l2 {
                let in_collision = contacts[j].depth > 0.0;
                let link_idx: usize = idxs[j][1][0];
                let env_object_idx: usize = idxs[j][0][0];
                let link_name = robot_world.get_robot_set_ref().get_robots_ref()[i].get_configuration_module_ref().robot_model_module.links[link_idx].name.clone();
                let env_obj_name = robot_world.get_collision_environment_option_ref().as_ref().unwrap().object_names[env_object_idx].clone();

                if in_collision {
                    ui.visuals_mut().override_text_color = Some(Color32::RED);
                }
                if ui.radio(false, "").is_pointer_button_down_on() {
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, i, link_idx, LynxMaterialType::CollisionHighlight1);
                    env_entity_and_info_server.change_env_object_material_data(env_object_idx, LynxMaterialType::CollisionHighlight2);
                }
                ui.label(format!("{}: {}, {}      distance: {:.3} m", j, link_name, env_obj_name, -contacts[j].depth));
                if contacts[j].depth > 0.0 {
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, i, link_idx, LynxMaterialType::Collision);
                    env_entity_and_info_server.change_env_object_material_data(env_object_idx, LynxMaterialType::Collision);
                }
                ui.visuals_mut().override_text_color = None;

                ui.end_row();
            }
        });
    }

    /*
    let vec_of_contact_check_multiple_result = robot_world.get_robot_set_mut_ref().multi_robot_contact_check(&vec_of_fk_results, collision_window_variables.self_collision_link_geometry_type.clone(), false, Some(0.5)).expect("error with multi robot contact info");
    let in_collision = vec_of_contact_check_multiple_result.in_collision();
    ui.horizontal(|ui| {
        if in_collision {
            ui.visuals_mut().override_text_color = Some(Color32::RED);
            ui.heading("Multi-robot Collision!");
            ui.visuals_mut().override_text_color = None;
        } else {
            ui.visuals_mut().override_text_color = Some(Color32::GREEN);
            ui.heading("No Multi-robot Collision");
            ui.visuals_mut().override_text_color = None;
        }

        egui::ComboBox::from_id_source("geometry type")
            .selected_text(collision_window_variables.multi_robot_collision_link_geometry_type.to_string())
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut collision_window_variables.multi_robot_collision_link_geometry_type, LinkGeometryType::OBBs, "OBBs");
                ui.selectable_value(&mut collision_window_variables.multi_robot_collision_link_geometry_type, LinkGeometryType::ConvexShapes, "ConvexShapes");
                ui.selectable_value(&mut collision_window_variables.multi_robot_collision_link_geometry_type, LinkGeometryType::OBBSubcomponents, "OBBSubs");
                ui.selectable_value(&mut collision_window_variables.multi_robot_collision_link_geometry_type, LinkGeometryType::ConvexShapeSubcomponents, "ConvexShapeSubs");
            });
    });
    ui.separator();

    let vec = vec_of_contact_check_multiple_result.get_contact_check_multiple_results_ref();
    let order = robot_world.get_robot_set_mut_ref().get_robot_set_result_vector_idxs_to_robot_idxs_ref().clone();
    let l1 = vec.len();
    for i in 0..l1 {
        let o = &order[i];
        ui.label(format!("Between Robot {}: {} and Robot {}: {}", o.0, robot_world.get_robot_set_ref().get_robots_ref()[o.0].get_robot_name_ref().clone(), o.1, robot_world.get_robot_set_ref().get_robots_ref()[o.1].get_robot_name_ref().clone() ));
        egui::Grid::new("grid2").striped(true).show(ui, |ui| {
            let v = &vec[i];
            let res = v.get_contact_check_multiple_info_ref();
            let contacts = res.get_contact_check_contacts();
            let idxs = res.get_contact_check_idxs();
            let l2 = contacts.len();
            for j in 0..l2 {
                if contacts[j].depth > 0.0 {
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, o.0, idxs[j][0][0], LynxMaterialType::Collision);
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, o.1, idxs[j][1][0], LynxMaterialType::Collision);
                }
                let link_idx1: usize = idxs[j][0][0];
                let link_idx2: usize = idxs[j][1][0];
                let link_name1 = robot_world.get_robot_set_ref().get_robots_ref()[o.0].get_configuration_module_ref().robot_model_module.links[link_idx1].name.clone();
                let link_name2 = robot_world.get_robot_set_ref().get_robots_ref()[o.1].get_configuration_module_ref().robot_model_module.links[link_idx2].name.clone();
                let in_collision = contacts[j].depth > 0.0;

                if in_collision {
                    ui.visuals_mut().override_text_color = Some(Color32::RED);
                }
                if ui.label(format!("{}: {}, {}      distance: {:.3} m", j, link_name1, link_name2, -contacts[j].depth)).hovered() {
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, o.0, link_idx1, LynxMaterialType::CollisionHighlight1);
                    robot_set_entity_and_info_server.change_link_material_data(robot_server_vector_idx, o.1, link_idx2, LynxMaterialType::CollisionHighlight2);
                    let p1 = contacts[j].world1.clone();
                    let p2 = contacts[j].world2.clone();
                    let color = if in_collision { Color::rgb(1.0, 0.0, 0.0) } else { Color::rgb(0.0, 0.0, 0.0) };
                    // draw_line_lynx_space(commands, meshes, materials, Vec3::new(p1[0] as f32, p1[1] as f32, p1[2] as f32), Vec3::new(p2[0] as f32, p2[1] as f32, p2[2] as f32), color, 6.0, LineType::Visualization);
                }
                ui.visuals_mut().override_text_color = None;
                ui.end_row();
            }
        });
        ui.separator();
    }
    */
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Improve this
fn _unified_collision_information_window(egui_context: &Res<EguiContext>,
                                         current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                                         collision_window_variables: &mut ResMut<CollisionWindowVariables>,
                                         robot_set_entity_and_info_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                         env_entity_and_info_server: &mut ResMut<EnvironmentEntityAndInfoServer>,
                                         robot_world: &mut RobotWorld) {
    egui::Window::new("Collision Information").open(&mut current_main_gui_values.display_collision_information).show(egui_context.ctx(), |ui| {
        egui::ScrollArea::from_max_height(600.0).id_source("d").show(ui, |ui| {
            egui::CollapsingHeader::new("Self Collision Information").id_source("aa").default_open(false).show(ui, |ui| {
                egui::ScrollArea::auto_sized().id_source("a").show(ui, |ui| {
                    let num_robots_on_server = robot_set_entity_and_info_server.get_num_individual_robot_set_entity_and_info_containers();
                    for i in 0..num_robots_on_server {
                        let ind = robot_set_entity_and_info_server.get_individual_robot_set_entity_and_info_container_ref(i).unwrap();
                        let joint_values = ind.get_robot_set_joint_values_ref();
                        let vec_of_fk_results_ = robot_world.get_robot_set_ref().compute_fk(joint_values);
                        if vec_of_fk_results_.is_err() { return; }
                        let vec_of_fk_results: VecOfRobotFKResult = vec_of_fk_results_.unwrap();

                        /*
                egui::ComboBox::from_id_source("geometry type")
                    .selected_text(current_main_gui_values.collision_panel_link_geometry_type.to_string())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut current_main_gui_values.collision_panel_link_geometry_type, LinkGeometryType::OBBs, "OBBs");
                        ui.selectable_value(&mut current_main_gui_values.collision_panel_link_geometry_type, LinkGeometryType::ConvexShapes, "ConvexShapes");
                        ui.selectable_value(&mut current_main_gui_values.collision_panel_link_geometry_type, LinkGeometryType::OBBSubcomponents, "OBBSubs");
                        ui.selectable_value(&mut current_main_gui_values.collision_panel_link_geometry_type, LinkGeometryType::ConvexShapeSubcomponents, "ConvexShapeSubs");
                    });
                   */
                        _self_collision_info_display_for_one_robot_set(ui, robot_world, &vec_of_fk_results, i, robot_set_entity_and_info_server, collision_window_variables);
                    }
                });
            });
            ui.separator();
            egui::ScrollArea::auto_sized().id_source("b").show(ui, |ui| {
                let num_robots_on_server = robot_set_entity_and_info_server.get_num_individual_robot_set_entity_and_info_containers();
                for i in 0..num_robots_on_server {
                    let ind = robot_set_entity_and_info_server.get_individual_robot_set_entity_and_info_container_ref(i).unwrap();
                    let joint_values = ind.get_robot_set_joint_values_ref();
                    let vec_of_fk_results_ = robot_world.get_robot_set_ref().compute_fk(joint_values);
                    if vec_of_fk_results_.is_err() { return; }
                    let vec_of_fk_results: VecOfRobotFKResult = vec_of_fk_results_.unwrap();

                    _multi_robot_collision_info_display_for_one_robot_set(ui, robot_world, &vec_of_fk_results, i, robot_set_entity_and_info_server, collision_window_variables);
                }
            });
            ui.separator();
            egui::ScrollArea::auto_sized().id_source("c").show(ui, |ui| {
                let num_robots_on_server = robot_set_entity_and_info_server.get_num_individual_robot_set_entity_and_info_containers();
                for i in 0..num_robots_on_server {
                    let ind = robot_set_entity_and_info_server.get_individual_robot_set_entity_and_info_container_ref(i).unwrap();
                    let joint_values = ind.get_robot_set_joint_values_ref();
                    let vec_of_fk_results_ = robot_world.get_robot_set_ref().compute_fk(joint_values);
                    if vec_of_fk_results_.is_err() { return; }
                    let vec_of_fk_results: VecOfRobotFKResult = vec_of_fk_results_.unwrap();

                    _environment_collision_info_display_for_one_robot_set(ui, robot_world, &vec_of_fk_results, i, robot_set_entity_and_info_server, env_entity_and_info_server, collision_window_variables);
                }
            });
        });
    });
}
