use bevy::prelude::{Vec3, PerspectiveCameraBundle, Transform, Commands, Entity};
use bevy::render::camera::PerspectiveProjection;
use crate::app_v2::app_structs::camera_structs::pan_orbit_camera::PanOrbitCamera;
use bevy_mod_picking::PickingCameraBundle;


pub fn spawn_pan_orbit_perspective_camera(commands: &mut Commands,
                                          location: Vec3) -> Entity {
    let radius = location.length();
    let id = commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_translation(location)
            .looking_at(Vec3::new(0.0,0.0,0.0), Vec3::new(0.0, 1.0, 0.0))
        ,
        perspective_projection: PerspectiveProjection {
            near: 0.001,
            ..Default::default()
        },
        ..Default::default()
    })
        .insert(PanOrbitCamera { radius, ..Default::default() })
        .insert_bundle(PickingCameraBundle::default()).id();
    return id;
}