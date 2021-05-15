use crate::app::app_utils::asset_utils::lynx_material_type::{LynxMaterialType, get_lynx_material_handle_id};
use bevy::prelude::{Res, AssetServer, ResMut, StandardMaterial, Query, Transform, Visible, Handle, Vec3, Assets};
use termion::{style, color};
use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;

pub fn change_material_of_environment_object(lynx_material_type: LynxMaterialType,
                                             asset_server: &Res<AssetServer>,
                                             materials: &mut ResMut<Assets<StandardMaterial>>,
                                             env_entity_and_info_server: &mut ResMut<EnvironmentEntityAndInfoServer>,
                                             env_object_idx: usize,
                                             q1: &mut Query<(&mut Transform)>,
                                             q2: &mut Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {

    let env_object_entity_pack = env_entity_and_info_server.get_env_object_entity_pack(env_object_idx).expect("error on idx");
    let visible_glb_scene_env_object_entity = &env_object_entity_pack.visible_glb_scene_env_object_entity;
    let standard_material_env_object_entity = &env_object_entity_pack.standard_material_env_object_entity;

    if visible_glb_scene_env_object_entity.is_some() {
        let mut q_ = q1.get_mut(visible_glb_scene_env_object_entity.as_ref().unwrap().clone());
        if q_.is_err() { return; }
        let q = q_.as_mut().ok().unwrap();
        q.scale = Vec3::new(0.,0.,0.);
    }

    if standard_material_env_object_entity.is_some() {
        let mut q_ = q2.get_mut(standard_material_env_object_entity.as_ref().unwrap().clone());
        if q_.is_err() { return; }
        let q = q_.as_mut().ok().unwrap();
        let visible: &mut Visible = &mut q.0;
        let mut handle: &mut Handle<StandardMaterial> = &mut q.1;

        let m = get_lynx_material_handle_id(lynx_material_type.clone(), materials);
        let material_handle: Handle<StandardMaterial> = asset_server.get_handle(m);

        *visible = Visible { is_visible: true, is_transparent: false };
        *handle = material_handle;
    }

    asset_server.free_unused_assets();
}

pub fn reset_material_of_environment_object(lynx_material_type: LynxMaterialType,
                                             asset_server: &Res<AssetServer>,
                                             materials: &mut ResMut<Assets<StandardMaterial>>,
                                             env_entity_and_info_server: &mut ResMut<EnvironmentEntityAndInfoServer>,
                                             env_object_idx: usize,
                                             q1: &mut Query<(&mut Transform)>,
                                             q2: &mut Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {

    let env_object_entity_pack = env_entity_and_info_server.get_env_object_entity_pack(env_object_idx).expect("error on idx");
    let visible_glb_scene_env_object_entity = &env_object_entity_pack.visible_glb_scene_env_object_entity;
    let standard_material_env_object_entity = &env_object_entity_pack.standard_material_env_object_entity;

    if visible_glb_scene_env_object_entity.is_some() {
        let mut q_ = q1.get_mut(visible_glb_scene_env_object_entity.as_ref().unwrap().clone());
        if q_.is_err() { return; }
        let mut q = q_.as_mut().ok().unwrap();
        q.scale = Vec3::new(1.,1.,1.);
    }

    if standard_material_env_object_entity.is_some() {
        let mut q_ = q2.get_mut(standard_material_env_object_entity.as_ref().unwrap().clone());
        if q_.is_err() { return; }
        let mut q = q_.as_mut().ok().unwrap();
        let visible: &mut Visible = &mut q.0;
        let mut handle: &mut Handle<StandardMaterial> = &mut q.1;

        let m = get_lynx_material_handle_id(lynx_material_type.clone(), materials);
        let material_handle: Handle<StandardMaterial> = asset_server.get_handle(m);

        if visible_glb_scene_env_object_entity.is_some() {
            *visible = Visible { is_visible: false, is_transparent: true };
        } else {
            *visible = Visible { is_visible: true, is_transparent: false };
        }
        *handle = material_handle;
    }

    asset_server.free_unused_assets();
}
