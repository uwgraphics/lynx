use bevy::asset::{HandleId, AssetServer, Assets, Handle};
use bevy::ecs::prelude::{Res, ResMut};
use bevy::prelude::{Mesh, Scene};
use bevy_stl;
use crate::utils::utils_files_and_strings::prelude::*;
use crate::app::app_utils::file_and_string_utils::directory_structure_utils::get_file_path_from_dummy_assets_to_mesh_environment_dir;

pub struct IndividualEnvObjectAssetLoader {
    _env_name: String,
    _env_object_name: String,
    _env_object_standard_material_mesh_handle_id: Option<HandleId>,
    _env_object_wireframe_mesh_handle_id: Option<HandleId>,
    _env_object_visible_glb_scene_handle_id: Option<HandleId>
}
impl IndividualEnvObjectAssetLoader {
    pub fn new(env_name: &String, env_object_name: &String, asset_server: &Res<AssetServer>) -> Self {
        let mut _env_object_standard_material_mesh_handle_id = None;
        let mut _env_object_wireframe_mesh_handle_id = None;
        let mut _env_object_visible_glb_scene_handle_id = None;

        let fp_to_mesh_assets_dir = get_path_to_src() + "/assets/mesh_environments/";
        let fp_to_mesh_environment = fp_to_mesh_assets_dir.clone() + env_name.as_str() + "/";
        let stl_fp = fp_to_mesh_environment.clone() + env_object_name.as_str() + ".stl";
        let glb_fp = fp_to_mesh_environment.clone() + env_object_name.as_str() + ".glb";

        if check_if_path_exists(stl_fp.clone()) {
            let path = get_file_path_from_dummy_assets_to_mesh_environment_dir(env_name) + "/" + env_object_name.as_str() + ".stl";
            let h: Handle<Mesh> = asset_server.load(path.as_str());
            _env_object_standard_material_mesh_handle_id = Some(h.id);
            let path = get_file_path_from_dummy_assets_to_mesh_environment_dir(env_name) + "/" + env_object_name.as_str() + ".stl#wireframe";
            let h: Handle<Mesh> = asset_server.load(path.as_str());
            _env_object_wireframe_mesh_handle_id = Some(h.id);
        }

        if check_if_path_exists(glb_fp.clone()) {
            let path = get_file_path_from_dummy_assets_to_mesh_environment_dir(env_name) + "/" + env_object_name.as_str() + ".glb#Scene0";
            let h: Handle<Scene> = asset_server.load(path.as_str());
            _env_object_visible_glb_scene_handle_id = Some(h.id);
        }

        return Self { _env_name: env_name.clone(),
            _env_object_name: env_object_name.clone(),
            _env_object_standard_material_mesh_handle_id,
            _env_object_wireframe_mesh_handle_id,
            _env_object_visible_glb_scene_handle_id };
    }

    pub fn get_env_object_standard_material_mesh_handle_id_ref(&self) -> &Option<HandleId> {
        return &self._env_object_standard_material_mesh_handle_id;
    }

    pub fn get_env_object_wireframe_mesh_handle_id_ref(&self) -> &Option<HandleId> {
        return &self._env_object_wireframe_mesh_handle_id;
    }

    pub fn get_env_object_visible_glb_scene_handle_id_ref(&self) -> &Option<HandleId> {
        return &self._env_object_visible_glb_scene_handle_id;
    }

}