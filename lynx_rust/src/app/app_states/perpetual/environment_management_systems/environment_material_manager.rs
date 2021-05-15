use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;
use bevy::prelude::{ResMut, Res, AssetServer, StandardMaterial, Visible, Query, Transform, Handle, Assets};
use crate::app::app_utils::environment_utils::environment_materials_utils::{change_material_of_environment_object, reset_material_of_environment_object};

pub fn environment_material_manager_system(mut env_entity_server: ResMut<EnvironmentEntityAndInfoServer>,
                                           asset_server: Res<AssetServer>,
                                           mut materials: ResMut<Assets<StandardMaterial>>,
                                           mut q1: Query<(&mut Transform)>,
                                           mut q2: Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {
    let changed_materials_pack = env_entity_server.get_changed_env_object_material_pack().clone();
    for m in &changed_materials_pack {
        change_material_of_environment_object(m.1,
                                              &asset_server,
                                              &mut materials,
                                              &mut env_entity_server,
                                              m.0,
                                              &mut q1,
                                              &mut q2);
    }

    let reset_materials_pack = env_entity_server.get_reset_env_object_material_pack().clone();
    for m in &reset_materials_pack {
        let mut found_in_changed_materials_pack = false;
        let l = changed_materials_pack.len();
        for j in 0..l {
            if changed_materials_pack[j].0 == m.0 { found_in_changed_materials_pack = true; }
        }

        if !found_in_changed_materials_pack {
            reset_material_of_environment_object(m.1,
                                                 &asset_server,
                                                 &mut materials,
                                                 &mut env_entity_server,
                                                 m.0,
                                                 &mut q1,
                                                 &mut q2);
        }
    }

    env_entity_server.wipe_environment_objects_with_changed_materials_list();
    env_entity_server.wipe_environment_objects_with_reset_materials_list();
}