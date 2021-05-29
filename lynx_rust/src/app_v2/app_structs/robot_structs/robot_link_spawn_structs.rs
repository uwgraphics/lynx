use crate::app_v2::app_type_enums::enums::RobotLinkSpawnType;

#[derive(Clone, Debug)]
pub struct RobotLinkSpawnIdxInfo {
    pub robot_set_idx_in_scene: usize,
    pub robot_idx_in_robot_set: usize,
    pub link_idx_in_robot: usize,
}

#[derive(Clone, Debug)]
pub struct RobotLinkStandardMaterialInfo {
    pub invisible_on_reset: bool
}

pub struct RobotLinkSpawn;

pub struct RobotLinkSpawnTypeStandardMaterial;
pub struct RobotLinkSpawnTypeSelectableInvisible;
pub struct RobotLinkSpawnTypeVisibleGlb;
