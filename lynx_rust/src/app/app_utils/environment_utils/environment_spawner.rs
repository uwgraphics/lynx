use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;
use crate::utils::utils_collisions::prelude::CollisionEnvironment;
use bevy::prelude::{Commands, AssetServer, Res, ResMut, Assets, StandardMaterial, Handle, Mesh, PbrBundle, Visible, Entity, RenderPipelines, GlobalTransform, BuildChildren};
use crate::app::app_utils::asset_utils::lynx_material_type::{LynxMaterialType, get_lynx_material_handle_id};
use crate::app::app_utils::asset_utils::individual_env_object_asset_loader::IndividualEnvObjectAssetLoader;
use crate::utils::utils_se3::prelude::ImplicitDualQuaternion;
use bevy::asset::HandleId;
use crate::app::app_utils::math_utils::transform_utils::convert_z_up_idq_to_y_up_bevy_transform;
use crate::app::app_states::res_comps::EnvObjectIdx;
use bevy::scene::{Scene, SpawnSceneAsChildCommands};
use bevy_mod_picking::PickableBundle;


pub fn spawn_environment(collision_environment: &CollisionEnvironment,
                         commands: &mut Commands,
                         asset_server: &Res<AssetServer>,
                         materials: &mut ResMut<Assets<StandardMaterial>>,
                         environment_entity_server: &mut ResMut<EnvironmentEntityAndInfoServer>) {

    let l = collision_environment.get_num_objects();
    for i in 0..l {
        let original_environment_name = &collision_environment.original_file_directories[i];
        let object_name = &collision_environment.object_names[i];

        let individual_env_object_asset_loader = IndividualEnvObjectAssetLoader::new(original_environment_name, object_name, asset_server);
        let standard_material_mesh_handle_id = individual_env_object_asset_loader.get_env_object_standard_material_mesh_handle_id_ref();
        let visible_glb_scene_handle_id = individual_env_object_asset_loader.get_env_object_visible_glb_scene_handle_id_ref();
        let env_object_frame = &collision_environment.transforms[i];

        let mut visible_glb_scene_env_object_entity = None;
        let mut standard_material_env_object_entity = None;
        let mut invisible_material_env_object_entity = None;

        if standard_material_mesh_handle_id.is_some() {
            standard_material_env_object_entity = Some(_spawn_standard_material_env_object(commands, asset_server, materials, env_object_frame, standard_material_mesh_handle_id.as_ref().unwrap(), i, visible_glb_scene_handle_id.is_some()));
            invisible_material_env_object_entity = Some(_spawn_selectable_invisible_env_object(commands, asset_server, materials, env_object_frame, standard_material_mesh_handle_id.as_ref().unwrap(), i));

        }

        if visible_glb_scene_handle_id.is_some() {
            visible_glb_scene_env_object_entity = Some(_spawn_visible_glb_scene_env_object(commands, asset_server, env_object_frame, visible_glb_scene_handle_id.as_ref().unwrap(), i));
        }

        environment_entity_server.push_env_object_entities(object_name.clone(), visible_glb_scene_env_object_entity, standard_material_env_object_entity, invisible_material_env_object_entity, Some(LynxMaterialType::Default));
    }
}


fn _spawn_standard_material_env_object(commands: &mut Commands,
                                       asset_server: &Res<AssetServer>,
                                       materials: &mut ResMut<Assets<StandardMaterial>>,
                                       frame: &ImplicitDualQuaternion,
                                       standard_material_mesh_handle_id: &HandleId,
                                       env_object_idx: usize,
                                       invisible: bool) -> Entity {

    let mesh: Handle<Mesh> = asset_server.get_handle(*standard_material_mesh_handle_id);
    let mut transform = convert_z_up_idq_to_y_up_bevy_transform(frame);

    let material_handle_id = get_lynx_material_handle_id(LynxMaterialType::Default, materials);
    let material: Handle<StandardMaterial> = asset_server.get_handle(material_handle_id);

    let pbr_bundle = if invisible {
        PbrBundle {
            mesh: mesh.clone(),
            material: material.clone(),
            transform,
            visible: Visible { is_visible: false, is_transparent: true },
            ..Default::default()
        }
    } else {
        PbrBundle {
            mesh: mesh.clone(),
            material: material.clone(),
            transform,
            ..Default::default()
        }
    };

    let id = commands.spawn_bundle(pbr_bundle)
        .insert(EnvObjectIdx(env_object_idx))
        .id();

    return id;
}

fn _spawn_selectable_invisible_env_object(commands: &mut Commands,
                                          asset_server: &Res<AssetServer>,
                                          materials: &mut ResMut<Assets<StandardMaterial>>,
                                          frame: &ImplicitDualQuaternion,
                                          standard_material_mesh_handle_id: &HandleId,
                                          env_object_idx: usize) -> Entity {

    let mesh: Handle<Mesh> = asset_server.get_handle(*standard_material_mesh_handle_id);
    let mut transform = convert_z_up_idq_to_y_up_bevy_transform(frame);

    let material_handle_id = get_lynx_material_handle_id(LynxMaterialType::Default, materials);
    let material: Handle<StandardMaterial> = asset_server.get_handle(material_handle_id);

    let pbr_bundle = PbrBundle {
        mesh: mesh.clone(),
        material: material.clone(),
        transform,
        render_pipelines: RenderPipelines { pipelines: vec![], bindings: Default::default() },
        ..Default::default()
    };

    let id = commands.spawn_bundle(pbr_bundle)
        .insert(EnvObjectIdx(env_object_idx))
        .insert_bundle(PickableBundle::default())
        .id();

    return id;
}

fn _spawn_visible_glb_scene_env_object(commands: &mut Commands,
                                       asset_server: &Res<AssetServer>,
                                       frame: &ImplicitDualQuaternion,
                                       visible_glb_scene_handle_id: &HandleId,
                                       env_object_idx: usize) -> Entity {

    let scene: Handle<Scene> = asset_server.get_handle(*visible_glb_scene_handle_id);
    let mut transform = convert_z_up_idq_to_y_up_bevy_transform(frame);

    let id = commands
        .spawn_bundle((
            transform,
            GlobalTransform::identity(),
        )).with_children(|parent| {
        parent.spawn_scene(scene);
    })
        .insert(EnvObjectIdx(env_object_idx))
        .id();

    return id;
}