use crate::app::app_states::res_comps::{RobotLinkInfoContainer, RobotSetEntityAndInfoServer};
use bevy::prelude::{ResMut, Query, Transform, StandardMaterial, Handle, Visible, Res, AssetServer, Assets, Vec3, Commands};
use crate::app::app_utils::asset_utils::lynx_material_type::*;
use termion::{color, style};
use bevy_mod_picking::PickableBundle;

pub fn change_material_of_robot_link(lynx_material_type: LynxMaterialType,
                                     asset_server: &Res<AssetServer>,
                                     materials: &mut ResMut<Assets<StandardMaterial>>,
                                     robot_set_entity_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                     robot_server_vector_idx: usize,
                                     robot_set_idx: usize,
                                     robot_link_idx: usize,
                                     q1: &mut Query<(&mut Transform)>,
                                     q2: &mut Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {
    if robot_set_entity_server.is_robot_hidden(robot_server_vector_idx) { return; }

    let robot_pack = robot_set_entity_server.get_link_entity_pack(robot_server_vector_idx, robot_set_idx, robot_link_idx);
    if robot_pack.is_err() {
        println!("{}{}WARNING: robot_set_entity_server.get_link_entity_pack threw a warning in change_material_of_robot_link.{}", style::Bold, color::Fg(color::Yellow), style::Reset);
    }
    let robot_pack_unwrap = robot_pack.as_ref().unwrap();

    let visible_glb_scene_link_entity = &robot_pack_unwrap.visible_glb_scene_link_entity;
    let standard_material_link_entity = &robot_pack_unwrap.standard_material_link_entity;

    if visible_glb_scene_link_entity.is_some() {
        let mut q = q1.get_mut(visible_glb_scene_link_entity.as_ref().unwrap().clone()).expect("error unwrapping visible_glb_scene_link_entity");
        q.scale = Vec3::new(0.,0.,0.);
    }

    if standard_material_link_entity.is_some() {
        let mut q = q2.get_mut(standard_material_link_entity.as_ref().unwrap().clone()).expect("error unwrapping standard_material_link_entity");
        let visible: &mut Visible = &mut q.0;
        let mut handle: &mut Handle<StandardMaterial> = &mut q.1;

        let m = get_lynx_material_handle_id(lynx_material_type.clone(), materials);
        let material_handle: Handle<StandardMaterial> = asset_server.get_handle(m);

        *visible = Visible { is_visible: true, is_transparent: false };
        *handle = material_handle;
    }

    asset_server.free_unused_assets();
}

pub fn reset_material_of_robot_link(lynx_material_type: LynxMaterialType,
                                    asset_server: &Res<AssetServer>,
                                    materials: &mut ResMut<Assets<StandardMaterial>>,
                                    robot_set_entity_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                                    robot_server_vector_idx: usize,
                                    robot_set_idx: usize,
                                    robot_link_idx: usize,
                                    q1: &mut Query<(&mut Transform)>,
                                    q2: &mut Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {
    if robot_set_entity_server.is_robot_hidden(robot_server_vector_idx) { return; }

    let robot_pack = robot_set_entity_server.get_link_entity_pack(robot_server_vector_idx, robot_set_idx, robot_link_idx);
    if robot_pack.is_err() {
        println!("{}, {}, {}", robot_server_vector_idx, robot_set_idx, robot_link_idx);
        println!("{}{}WARNING: robot_set_entity_server.get_link_entity_pack threw a warning in reset_material_of_robot_link.{}", style::Bold, color::Fg(color::Yellow), style::Reset);
    }
    let robot_pack_unwrap = robot_pack.as_ref().unwrap();

    let visible_glb_scene_link_entity = &robot_pack_unwrap.visible_glb_scene_link_entity;
    let standard_material_link_entity = &robot_pack_unwrap.standard_material_link_entity;

    if visible_glb_scene_link_entity.is_some() {
        let mut q = q1.get_mut(visible_glb_scene_link_entity.as_ref().unwrap().clone()).expect("error unwrapping visible_glb_scene_link_entity");
        q.scale = Vec3::new(1.,1.,1.);
    }

    if standard_material_link_entity.is_some() {
        let mut q_ = q2.get_mut(standard_material_link_entity.as_ref().unwrap().clone());
        if q_.is_err() { return; }
        let mut q = q_.as_mut().unwrap();
        let visible: &mut Visible = &mut q.0;
        let mut handle: &mut Handle<StandardMaterial> = &mut q.1;

        let m = get_lynx_material_handle_id(lynx_material_type.clone(), materials);
        let material_handle: Handle<StandardMaterial> = asset_server.get_handle(m);

        if visible_glb_scene_link_entity.is_some() {
            *visible = Visible { is_visible: false, is_transparent: true };
        } else {
            *visible = Visible { is_visible: true, is_transparent: false };
        }
        *handle = material_handle;
    }

    asset_server.free_unused_assets();
}

pub fn hide_robot_link(commands: &mut Commands,
                       robot_set_entity_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                       robot_server_vector_idx: usize,
                       robot_set_idx: usize,
                       robot_link_idx: usize,
                       q1: &mut Query<(&mut Transform)>,
                       q2: &mut Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {
    let robot_pack = robot_set_entity_server.get_link_entity_pack(robot_server_vector_idx, robot_set_idx, robot_link_idx);
    if robot_pack.is_err() {
        println!("{}{}WARNING: robot_set_entity_server.get_link_entity_pack threw a warning in hide_robot_link.{}", style::Bold, color::Fg(color::Yellow), style::Reset);
    }
    let robot_pack_unwrap = robot_pack.as_ref().unwrap();

    let visible_glb_scene_link_entity = &robot_pack_unwrap.visible_glb_scene_link_entity;
    let standard_material_link_entity = &robot_pack_unwrap.standard_material_link_entity;
    let invisible_material_link_entity = &robot_pack_unwrap.invisible_material_link_entity;

    if visible_glb_scene_link_entity.is_some() {
        let mut q = q1.get_mut(visible_glb_scene_link_entity.as_ref().unwrap().clone()).expect("error unwrapping visible_glb_scene_link_entity");
        q.scale = Vec3::new(0.,0.,0.);
    }

    if standard_material_link_entity.is_some() {
        let mut q = q2.get_mut(standard_material_link_entity.as_ref().unwrap().clone()).expect("error unwrapping standard_material_link_entity");
        let visible: &mut Visible = &mut q.0;

        *visible = Visible { is_visible: false, is_transparent: true };
    }

    if invisible_material_link_entity.is_some() {
        commands.entity(invisible_material_link_entity.as_ref().unwrap().clone()).remove_bundle::<PickableBundle>();
    }
}

pub fn unhide_robot_link(commands: &mut Commands,
                       robot_set_entity_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                       robot_server_vector_idx: usize,
                       robot_set_idx: usize,
                       robot_link_idx: usize,
                       q1: &mut Query<(&mut Transform)>,
                       q2: &mut Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {
    let robot_pack = robot_set_entity_server.get_link_entity_pack(robot_server_vector_idx, robot_set_idx, robot_link_idx);
    if robot_pack.is_err() {
        println!("{}{}WARNING: robot_set_entity_server.get_link_entity_pack threw a warning in hide_robot_link.{}", style::Bold, color::Fg(color::Yellow), style::Reset);
    }
    let robot_pack_unwrap = robot_pack.as_ref().unwrap();

    let visible_glb_scene_link_entity = &robot_pack_unwrap.visible_glb_scene_link_entity;
    let standard_material_link_entity = &robot_pack_unwrap.standard_material_link_entity;
    let invisible_material_link_entity = &robot_pack_unwrap.invisible_material_link_entity;

    if visible_glb_scene_link_entity.is_some() {
        let mut q = q1.get_mut(visible_glb_scene_link_entity.as_ref().unwrap().clone()).expect("error unwrapping visible_glb_scene_link_entity");
        q.scale = Vec3::new(1.,1.,1.);
    }

    if standard_material_link_entity.is_some() {
        let mut q = q2.get_mut(standard_material_link_entity.as_ref().unwrap().clone()).expect("error unwrapping standard_material_link_entity");
        let visible: &mut Visible = &mut q.0;

        if visible_glb_scene_link_entity.is_some() {
            *visible = Visible { is_visible: false, is_transparent: true };
        } else {
            *visible = Visible { is_visible: true, is_transparent: false };
        }
    }

    if invisible_material_link_entity.is_some() {
        commands.entity(invisible_material_link_entity.as_ref().unwrap().clone()).insert_bundle(PickableBundle::default());
    }
}

