use bevy::math::Vec3;

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum CameraType {
    PanOrbitCamera
}

pub struct PanOrbitCamera {
    pub focus: Vec3,
    pub radius: f32,
    pub upside_down: bool,
}

impl Default for PanOrbitCamera {
    fn default() -> Self {
        PanOrbitCamera {
            focus: Vec3::ZERO,
            radius: 5.0,
            upside_down: false,
        }
    }
}