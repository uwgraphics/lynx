use std::time::{Instant, Duration};

pub use crate::app::app_states::perpetual::{
    camera_management_systems::camera_management_res_comps::*,
    robot_management_systems::robot_management_res_comps::*,
    gui_management_systems::gui_management_res_comps::*,
    environment_management_systems::environment_management_res_comps::*};

pub struct InstantContainer {
    pub robot_reset: Instant
}
impl InstantContainer {
    pub fn new() -> Self {
        Self {
            robot_reset: Instant::now()
        }
    }
}

pub struct GreenScreenOn(pub bool);