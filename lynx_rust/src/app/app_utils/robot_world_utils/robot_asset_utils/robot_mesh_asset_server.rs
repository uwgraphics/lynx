use crate::app::app_utils::robot_world_utils::robot_asset_utils::individual_robot_mesh_asset_container::*;
use std::collections::HashMap;
use bevy::asset::AssetServer;
use bevy::ecs::prelude::Res;

pub struct RobotMeshAssetServer {
    _individual_robot_mesh_asset_containers: Vec<IndividualRobotAssetContainer>,
    _hashmap: HashMap<String, usize>
}

impl RobotMeshAssetServer {
    pub fn new_empty() -> Self {
        let _individual_robot_mesh_asset_containers = Vec::new();
        let _hashmap = HashMap::new();

        Self { _individual_robot_mesh_asset_containers, _hashmap }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_individual_robot_asset_container_ref(&mut self, robot_name: &String, asset_server: &Res<AssetServer>) -> Result<&IndividualRobotAssetContainer, String> {
        let res = self._hashmap.get(robot_name);
        if res.is_some() {
            let idx = res.unwrap();
            return Ok(&self._individual_robot_mesh_asset_containers[*idx]);
        } else {
            let individual_robot_mesh_asset_container = IndividualRobotAssetContainer::new(robot_name, asset_server);
            let l = self._individual_robot_mesh_asset_containers.len();
            self._hashmap.insert(robot_name.clone(), l);
            self._individual_robot_mesh_asset_containers.push(individual_robot_mesh_asset_container);

            return Ok(&self._individual_robot_mesh_asset_containers[l]);
        }
    }
}