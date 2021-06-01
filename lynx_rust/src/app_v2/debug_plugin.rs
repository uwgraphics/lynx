use bevy::prelude::*;
use crate::app_v2::app_structs::game_engine_structs::game_engine_status_structs::FrameCount;
use crate::app_v2::app_structs::asset_structs::robot_set_asset_loader::RobotSetAssetLoader;
use crate::app_v2::app_actions::robot_actions::robot_spawners::{spawn_robot_link_standard_material, spawn_robot_link_visible_glb, spawn_robot_link_invisible_selectable, spawn_robot_set, despawn_all_robot_sets_in_scene};
use crate::app_v2::app_structs::robot_structs::robot_link_spawn_structs::{RobotLinkSpawnIdxInfo, RobotLinkSpawn, RobotLinkSpawnTypeSelectableInvisible, RobotLinkSpawnTypeVisibleGlb, RobotLinkStandardMaterialInfo, RobotLinkSpawnTypeStandardMaterial};
use crate::app_v2::app_structs::robot_structs::robot_world_status_structs::NumberOfRobotSetsSpawnedInScene;
use crate::prelude::LynxVarsGeneric;
use crate::app_v2::app_type_enums::enums::{RobotLinkSpawnType, RobotSetSpawnType, LynxMaterialChangeType, LynxMaterialType, EguiWindowType};
use crate::app_v2::app_actions::robot_actions::robot_pose_actions::pose_robot_set_from_full_joint_state;
use crate::utils::utils_math::prelude::vec_to_dvec;
use crate::app_v2::app_structs::material_structs::lynx_material_structs::LynxMaterialUser;
use crate::app_v2::app_actions::robot_actions::robot_material_actions::change_link_material_in_robot_set;
use nalgebra::DVector;
use crate::app_v2::app_structs::gui_structs::egui_structs::EguiWindowContainer;
use bevy_egui::egui::*;
use bevy_egui::EguiContext;

pub struct DebugPlugin;

impl Plugin for DebugPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_system(debug_once_system1.system())
            .add_system(debug_once_system2.system())
            .add_system(debug_per_frame_system1.system())
            .add_system(debug_per_frame_system2.system());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn debug_once_system1(frame_count: Res<FrameCount>,
                          mut commands: Commands,
                          asset_server: Res<AssetServer>,
                          mut robot_set_asset_loader: ResMut<RobotSetAssetLoader>,
                          mut number_of_robots_in_scene: ResMut<NumberOfRobotSetsSpawnedInScene>,
                          mut lynx_vars: ResMut<LynxVarsGeneric<'static>>) {
    if frame_count.0 < 1 {
        let robot_set = lynx_vars.get_robot_world_ref(None).expect("error").get_robot_set_ref();
        spawn_robot_set(&mut commands, &asset_server, &mut robot_set_asset_loader, &mut number_of_robots_in_scene, robot_set, None, RobotSetSpawnType::Physical);
    }
}

pub fn debug_once_system2(frame_count: Res<FrameCount>,
                          mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                          mut query: Query<(&mut Transform, &RobotLinkSpawnIdxInfo, &RobotLinkSpawnType)>) {
    if frame_count.0 == 1 {
        let robot_set = lynx_vars.get_robot_set_ref_via_robot_world(None).expect("error");
        let res = pose_robot_set_from_full_joint_state(robot_set, &vec_to_dvec(&vec![1., 1., 1., 1., 1., 1.]), 0, &mut query);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn debug_per_frame_system1(keys: Res<Input<KeyCode>>,
                               frame_count: Res<FrameCount>,
                               asset_server: Res<AssetServer>,
                               mut materials: ResMut<Assets<StandardMaterial>>,
                               link_query: Query<(Entity, &RobotLinkSpawnIdxInfo, &RobotLinkSpawnType), (Without<RobotLinkSpawnTypeSelectableInvisible>)>,
                               mut link_visible_glb_query: Query<(&mut Transform), (With<RobotLinkSpawnTypeVisibleGlb>)>,
                               mut link_standard_material_query: Query<(&mut LynxMaterialUser, &RobotLinkSpawnIdxInfo, &RobotLinkStandardMaterialInfo, &mut Visible, &mut Handle<StandardMaterial>), (With<RobotLinkSpawnTypeStandardMaterial>)>) {

    if keys.just_pressed(KeyCode::R) {
        change_link_material_in_robot_set(0,
                                          0,
                                          2,
                                          LynxMaterialChangeType::ForceResetToBase,
                                          &asset_server,
                                          &mut materials,
                                          &link_query,
                                          &mut link_visible_glb_query,
                                          &mut link_standard_material_query
        );
    }

    if keys.just_pressed(KeyCode::A) {
        change_link_material_in_robot_set(0,
                                          0,
                                          2,
                                          LynxMaterialChangeType::ChangeWithImportanceCheck {
                                              material_to_change_to: LynxMaterialType::Default
                                          },
                                          &asset_server,
                                          &mut materials,
                                          &link_query,
                                          &mut link_visible_glb_query,
                                          &mut link_standard_material_query
        );
    }

    if keys.just_pressed(KeyCode::B) {
        change_link_material_in_robot_set(0,
                                          0,
                                          2,
                                          LynxMaterialChangeType::ChangeWithImportanceCheck {
                                              material_to_change_to: LynxMaterialType::Collision
                                          },
                                          &asset_server,
                                          &mut materials,
                                          &link_query,
                                          &mut link_visible_glb_query,
                                          &mut link_standard_material_query
        );
    }

    if keys.just_pressed(KeyCode::C) {
        change_link_material_in_robot_set(0,
                                          0,
                                          2,
                                          LynxMaterialChangeType::ForceChange {
                                              material_to_change_to: LynxMaterialType::Default
                                          },
                                          &asset_server,
                                          &mut materials,
                                          &link_query,
                                          &mut link_visible_glb_query,
                                          &mut link_standard_material_query
        );
    }
}

pub fn debug_per_frame_system2(keys: Res<Input<KeyCode>>,
                               egui_context: Res<EguiContext>,
                               mut egui_window_container: ResMut<EguiWindowContainer>) {

    if keys.just_pressed(KeyCode::J) {
        egui_window_container.open_window(EguiWindowType::RobotSetSelection, None);
    }

    if keys.just_pressed(KeyCode::K) {
        egui_window_container.open_window(EguiWindowType::RobotConfigurationSelection, None);
    }

    if keys.just_pressed(KeyCode::L) {
        egui_window_container.open_window(EguiWindowType::RobotSelection, None);
    }

    // let mut a = egui_window_container.get_window_name_str_and_window_is_open_bundle(EguiWindowType::RobotSelection, None);

    let mut a = egui_window_container.get_window_name_str_and_window_is_open_bundle(EguiWindowType::RobotSetSelection, None);
    bevy_egui::egui::Window::new("test").id(Id::new(a.0.clone())).open(&mut a.1).fixed_pos(Pos2::new(500.,500.)).show(egui_context.ctx(), |ui| {

    });
    if !(a.1) { egui_window_container.close_window_by_idx(a.2, true); }

    let mut a = egui_window_container.get_window_name_str_and_window_is_open_bundle(EguiWindowType::RobotSelection, None);
    bevy_egui::egui::Window::new("test2").id(Id::new(a.0.clone())).open(&mut a.1).show(egui_context.ctx(), |ui| {
        if ui.rect_contains_pointer(ui.max_rect()) { println!("yes!"); }
    });
    if !(a.1) { egui_window_container.close_window_by_idx(a.2, true); }

    let mut a = egui_window_container.get_window_name_str_and_window_is_open_bundle(EguiWindowType::RobotConfigurationSelection, None);
    bevy_egui::egui::Window::new("test3").id(Id::new(a.0.clone())).open(&mut a.1).show(egui_context.ctx(), |ui| {

    });
    if !(a.1) { egui_window_container.close_window_by_idx(a.2, true); }


    bevy_egui::egui::Area::new("hi").show(egui_context.ctx(), |ui| {
       ui.label("Hi!");
    });
}

////////////////////////////////////////////////////////////////////////////////////////////////////