use crate::app_v2::app_structs::asset_structs::individual_robot_asset_loader::IndividualRobotAssetLoader;
use crate::prelude::RobotSet;
use bevy::prelude::{AssetServer, Res};

pub struct RobotSetAssetLoader {
    _robot_asset_loaders: Vec<IndividualRobotAssetLoader>
}
impl RobotSetAssetLoader {
    pub fn new(robot_set: &RobotSet, asset_server: &Res<AssetServer>) -> Self {
        let mut _robot_asset_loaders = Vec::new();

        let l = robot_set.get_num_robots();
        for i in 0..l {
            let robot_name = robot_set.get_robots_ref()[i].get_robot_name_ref();
            _robot_asset_loaders.push( IndividualRobotAssetLoader::new(robot_name, asset_server) );
        }

        Self { _robot_asset_loaders  }
    }

    pub fn get_robot_asset_loaders_ref(&self) -> &Vec<IndividualRobotAssetLoader> {
        return &self._robot_asset_loaders;
    }

    pub fn get_robot_asset_loaders_mut_ref(&mut self) -> &mut Vec<IndividualRobotAssetLoader> {
        return &mut self._robot_asset_loaders;
    }

    pub fn get_individual_robot_asset_loader_ref(&self, idx: usize) -> Result<&IndividualRobotAssetLoader, String> {
        if idx >= self._robot_asset_loaders.len() {
            return Err(format!("idx {:?} is too for for number of individual_robot_asset_loaders ({:?})", idx, self._robot_asset_loaders.len()));
        }

        return Ok(&self._robot_asset_loaders[idx]);
    }

    pub fn get_individual_robot_asset_loader_mut_ref(&mut self, idx: usize) -> Result<&mut IndividualRobotAssetLoader, String> {
        if idx >= self._robot_asset_loaders.len() {
            return Err(format!("idx {:?} is too for for number of individual_robot_asset_loaders ({:?})", idx, self._robot_asset_loaders.len()));
        }

        return Ok(&mut self._robot_asset_loaders[idx]);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn link_has_visible_glb(&self, robot_idx_in_set: usize, link_idx_in_robot: usize) -> Result<bool, String> {
        return self.get_individual_robot_asset_loader_ref(robot_idx_in_set)?.link_has_visible_glb(link_idx_in_robot);
    }
}