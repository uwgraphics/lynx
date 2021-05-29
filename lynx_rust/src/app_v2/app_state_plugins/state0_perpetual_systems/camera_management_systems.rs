use bevy::prelude::{Res, EventReader, MouseButton, Transform, Windows, Query, Input};
use crate::app_v2::app_structs::camera_structs::pan_orbit_camera::PanOrbitCamera;
use bevy::render::camera::PerspectiveProjection;
use bevy::input::mouse::{MouseMotion, MouseWheel};
use crate::app_v2::app_actions::camera_actions::{
    pan_orbit_camera_user_update::pan_orbit_camera_user_update
};

pub fn main_camera_management_system(
    windows: Res<Windows>,
    mut ev_motion: EventReader<MouseMotion>,
    mut ev_scroll: EventReader<MouseWheel>,
    input_mouse: Res<Input<MouseButton>>,
    mut query: Query<(&mut PanOrbitCamera, &mut Transform, &PerspectiveProjection)>,
) {
    pan_orbit_camera_user_update(&windows, &mut ev_motion, &mut ev_scroll, &input_mouse, &mut query);
}