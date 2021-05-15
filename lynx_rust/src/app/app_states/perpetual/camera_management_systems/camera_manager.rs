use bevy::math::{Vec3, Vec2, Quat, Mat3};
use bevy::ecs::prelude::{Res, Query};
use bevy::prelude::{Windows, Transform};
use bevy::app::EventReader;
use bevy::input::mouse::{MouseMotion, MouseButton, MouseWheel};
use bevy::input::Input;
use bevy::render::camera::PerspectiveProjection;
use crate::app::app_states::perpetual::camera_management_systems::pan_orbit_camera_system::*;
use crate::app::app_states::perpetual::camera_management_systems::camera_management_res_comps::*;

pub fn camera_manager(camera_type: Res<CameraType>,
                      windows: Res<Windows>,
                      mut ev_motion: EventReader<MouseMotion>,
                      mut ev_scroll: EventReader<MouseWheel>,
                      input_mouse: Res<Input<MouseButton>>,
                      mut query: Query<(&mut PanOrbitCamera, &mut Transform, &PerspectiveProjection)>
) {
    match *camera_type {
        CameraType::PanOrbitCamera => {
            pan_orbit_camera(&windows, &mut ev_motion, &mut ev_scroll, &input_mouse, &mut query);
        }
    }
}