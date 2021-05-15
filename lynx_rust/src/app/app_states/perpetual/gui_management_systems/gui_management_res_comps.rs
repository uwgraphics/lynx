pub use crate::app::app_states::perpetual::gui_management_systems::gui_values::*;
use crate::prelude::LinkGeometryType;


pub struct MultipleRobotLinkFocusWindowOpenManager {
    pub window_open_on_last_update: Vec<bool>
}
impl MultipleRobotLinkFocusWindowOpenManager {
    pub fn new_empty() -> Self {
        Self { window_open_on_last_update: Vec::new() }
    }
}

pub struct MultipleEnvironmentFocusWindowOpenManager {
    pub window_open_on_last_update: Vec<bool>
}
impl MultipleEnvironmentFocusWindowOpenManager {
    pub fn new_empty() -> Self {
        Self { window_open_on_last_update: Vec::new() }
    }
}

pub struct CollisionWindowVariables {
    pub self_collision_link_geometry_type: LinkGeometryType,
    pub multi_robot_collision_link_geometry_type: LinkGeometryType,
    pub environment_collision_link_geometry_type: LinkGeometryType,
    pub reset_collision_tensor_checkbox: bool
}
impl CollisionWindowVariables {
    pub fn new() -> Self {
        Self {
            self_collision_link_geometry_type: LinkGeometryType::OBBs,
            multi_robot_collision_link_geometry_type: LinkGeometryType::OBBs,
            environment_collision_link_geometry_type: LinkGeometryType::OBBs,
            reset_collision_tensor_checkbox: false
        }
    }
}
