use crate::robot_modules::prelude::RobotSet;
use crate::utils::utils_se3::prelude::ImplicitDualQuaternion;

use crate::app_v2::app_structs::{
    asset_structs::robot_set_asset_loader::*,
    robot_structs::robot_link_spawn_structs::*,
    robot_structs::robot_world_status_structs::*,
    material_structs::lynx_material_structs::*,
};

use crate::app_v2::app_type_enums::enums::{
    LynxMaterialType,
    RobotLinkSpawnType,
    RobotSetSpawnType,
};

use bevy::prelude::{Commands, Res, AssetServer, ResMut, Assets, StandardMaterial, Entity, Handle, PbrBundle, Visible, Mesh, Transform, Vec3, GlobalTransform, BuildChildren, SpawnSceneAsChildCommands, RenderPipelines, Query, With, DespawnRecursiveExt};
use bevy::scene::Scene;
use bevy_mod_picking::PickableBundle;

pub fn spawn_robot_link_standard_material(commands: &mut Commands,
                                          asset_server: &Res<AssetServer>,
                                          robot_set_asset_loader: &mut ResMut<RobotSetAssetLoader>,
                                          robot_idx_in_robot_set: usize,
                                          link_idx_in_robot: usize,
                                          base_material: Option<LynxMaterialType>,
                                          invisible: bool) -> Result<Entity, String> {
    let handle_id_option = robot_set_asset_loader
        .get_individual_robot_asset_loader_ref(robot_idx_in_robot_set)?
        .get_link_standard_material_mesh_handle_id(link_idx_in_robot)?;

    if handle_id_option.is_none() { return Err("handle_id_option is none".to_string()); }

    let mesh: Handle<Mesh> = asset_server.get_handle(handle_id_option.unwrap());

    let pbr_bundle = if invisible {
        PbrBundle {
            mesh,
            visible: Visible { is_visible: false, is_transparent: true },
            ..Default::default()
        }
    } else {
        PbrBundle {
            mesh,
            ..Default::default()
        }
    };

    let id = commands.spawn_bundle(pbr_bundle)
        .insert(RobotLinkSpawn)
        .insert(RobotLinkSpawnTypeStandardMaterial)
        .insert(RobotLinkSpawnType::StandardMaterial)
        .insert(LynxMaterialUser::new(&base_material))
        .insert(RobotLinkStandardMaterialInfo { invisible_on_reset: invisible })
        .id();

    return Ok(id);
}

pub fn spawn_robot_link_visible_glb(commands: &mut Commands,
                                    asset_server: &Res<AssetServer>,
                                    robot_set_asset_loader: &mut ResMut<RobotSetAssetLoader>,
                                    robot_idx_in_robot_set: usize,
                                    link_idx_in_robot: usize) -> Result<Entity, String> {
    let handle_id_option = robot_set_asset_loader
        .get_individual_robot_asset_loader_ref(robot_idx_in_robot_set)?
        .get_link_visible_glb_scene_handle_id(link_idx_in_robot)?;

    if handle_id_option.is_none() { return Err("handle_id_option is none".to_string()); }

    let scene: Handle<Scene> = asset_server.get_handle(handle_id_option.unwrap());

    let id = commands
        .spawn_bundle((
            Transform::identity(),
            GlobalTransform::identity(),
        )).with_children(|parent| {
        parent.spawn_scene(scene);
    })
        .insert(RobotLinkSpawn)
        .insert(RobotLinkSpawnTypeVisibleGlb)
        .insert(RobotLinkSpawnType::VisibleGlb)
        .id();

    return Ok(id);
}

pub fn spawn_robot_link_invisible_selectable(commands: &mut Commands,
                                             asset_server: &Res<AssetServer>,
                                             robot_set_asset_loader: &mut ResMut<RobotSetAssetLoader>,
                                             robot_idx_in_robot_set: usize,
                                             link_idx_in_robot: usize) -> Result<Entity, String> {
    let handle_id_option = robot_set_asset_loader
        .get_individual_robot_asset_loader_ref(robot_idx_in_robot_set)?
        .get_link_lower_poly_mesh_handle_id(link_idx_in_robot)?;

    if handle_id_option.is_none() { return Err("handle_id_option is none".to_string()); }

    let mesh: Handle<Mesh> = asset_server.get_handle(handle_id_option.unwrap());

    let pbr_bundle = PbrBundle {
        mesh: mesh.clone(),
        render_pipelines: RenderPipelines { pipelines: vec![], bindings: Default::default() },
        ..Default::default()
    };

    let id = commands.spawn_bundle(pbr_bundle)
        .insert(RobotLinkSpawn)
        .insert(RobotLinkSpawnTypeSelectableInvisible)
        .insert(RobotLinkSpawnType::SelectableInvisible)
        .insert_bundle(PickableBundle::default())
        .id();

    return Ok(id);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn spawn_robot_set(commands: &mut Commands,
                       asset_server: &Res<AssetServer>,
                       robot_set_asset_loader: &mut ResMut<RobotSetAssetLoader>,
                       number_of_robot_sets_in_scene: &mut ResMut<NumberOfRobotSetsSpawnedInScene>,
                       robot_set: &RobotSet,
                       base_material: Option<LynxMaterialType>,
                       robot_set_spawn_type: RobotSetSpawnType) -> Result<(), String> {
    let robot_set_idx_in_scene = number_of_robot_sets_in_scene.0;

    let robots = robot_set.get_robots_ref();
    let l1 = robots.len();
    for i in 0..l1 {
        let links = &robots[i].get_configuration_module_ref().robot_model_module.links;
        let l2 = links.len();
        for j in 0..l2 {
            if links[j].active {
                let invisible = robot_set_spawn_type.eq(&RobotSetSpawnType::Physical) && robot_set_asset_loader.link_has_visible_glb(i, j)?;
                let res = spawn_robot_link_standard_material(commands, asset_server, robot_set_asset_loader, i, j, base_material.clone(), invisible);
                _insert_robot_link_idx_info_into_entity_result(commands, &res, robot_set_idx_in_scene, i, j);
                _insert_robot_set_spawn_type_into_entity_result(commands, &res, robot_set_spawn_type.clone());

                let res = spawn_robot_link_invisible_selectable(commands, asset_server, robot_set_asset_loader, i, j);
                _insert_robot_link_idx_info_into_entity_result(commands, &res, robot_set_idx_in_scene, i, j);
                _insert_robot_set_spawn_type_into_entity_result(commands, &res, robot_set_spawn_type.clone());

                if robot_set_spawn_type.eq(&RobotSetSpawnType::Physical) {
                    let res = spawn_robot_link_visible_glb(commands, asset_server, robot_set_asset_loader, i, j);
                    _insert_robot_link_idx_info_into_entity_result(commands, &res, robot_set_idx_in_scene, i, j);
                    _insert_robot_set_spawn_type_into_entity_result(commands, &res, robot_set_spawn_type.clone());
                }
            }
        }
    }

    number_of_robot_sets_in_scene.increment();
    Ok(())
}

fn _insert_robot_link_idx_info_into_entity_result(commands: &mut Commands,
                                                  res: &Result<Entity, String>,
                                                  robot_set_idx_in_scene: usize,
                                                  robot_idx_in_robot_set: usize,
                                                  link_idx_in_robot: usize) {
    if res.is_ok() {
        commands.entity(res.as_ref().unwrap().to_owned()).insert(RobotLinkSpawnIdxInfo {
            robot_set_idx_in_scene,
            robot_idx_in_robot_set,
            link_idx_in_robot,
        });
    }
}

fn _insert_robot_set_spawn_type_into_entity_result(commands: &mut Commands,
                                                   res: &Result<Entity, String>,
                                                   robot_set_spawn_type: RobotSetSpawnType) {
    if res.is_ok() {
        commands.entity(res.as_ref().unwrap().to_owned()).insert(robot_set_spawn_type);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub fn despawn_robot_set(commands: &mut Commands,
                         query: &Query<(Entity, &RobotLinkSpawnIdxInfo)>,
                         robot_set_idx_in_scene: usize) {
    for (entity, robot_link_spawn_idx_info) in query.iter() {
        if robot_link_spawn_idx_info.robot_set_idx_in_scene == robot_set_idx_in_scene {
            commands.entity(entity).despawn_recursive();
        }
    }
}

pub fn despawn_all_robot_sets_in_scene(commands: &mut Commands,
                                       query: &Query<(Entity), (With<RobotLinkSpawn>)>,
                                       number_of_robot_sets_in_scene: &mut ResMut<NumberOfRobotSetsSpawnedInScene>) {
    for entity in query.iter() {
        commands.entity(entity).despawn_recursive();
    }
    number_of_robot_sets_in_scene.0 = 0;
}




