use bevy::prelude::{ResMut, Assets, StandardMaterial};
use bevy::asset::HandleId;
use bevy::render::prelude::Color;

// materials listed lower in the enum are considered "greater" (will override colors that are "less than")
#[derive(PartialEq, Clone, Debug, Copy, PartialOrd)]
pub enum LynxMaterialType {
    Default,
    VisualizationDefault,
    PathPlanningStart,
    PathPlanningGoal,
    Interpolate{ start_color_rgb: [f64; 3], end_color_rgb: [f64; 3], u: f64, alpha: f64 },
    Focus,
    Collision,
    Hover,
    FocusWindowHover,
    Selection,
    CollisionHighlight1,
    CollisionHighlight2,
}

pub fn get_lynx_material_handle_id(material_handle_type: LynxMaterialType, materials: &mut ResMut<Assets<StandardMaterial>>) -> HandleId {
    return match material_handle_type {
        LynxMaterialType::Default => { materials.add(Color::rgb(0.3, 0.3, 0.3).into()).id },
        LynxMaterialType::VisualizationDefault => { materials.add(Color::rgb(0.9, 0.5, 0.6).into()).id },
        LynxMaterialType::Collision => { materials.add(Color::rgba(1.0, 0.2, 0.3, 0.6).into()).id },
        LynxMaterialType::Hover => { materials.add(Color::rgba(147.0/255.0,112.0/255.0,219.0/255.0, 0.4).into()).id }
        LynxMaterialType::Focus => { materials.add(Color::rgba(0.0, 0.4, 1.0, 0.8).into()).id }
        LynxMaterialType::FocusWindowHover => { materials.add(Color::rgba(0.0, 0.9, 1.0, 0.7).into()).id }
        LynxMaterialType::Selection => { materials.add(Color::rgb(0.0, 0.6, 0.9).into()).id },
        LynxMaterialType::CollisionHighlight1 => { materials.add(Color::rgba(0.9, 0.7, 0.3, 0.6).into()).id },
        LynxMaterialType::CollisionHighlight2 => { materials.add(Color::rgba(1.0, 0.6, 0.4, 0.6).into()).id },
        LynxMaterialType::PathPlanningStart => { materials.add(Color::rgba(0.85, 0.85, 0.85, 0.6).into()).id },
        LynxMaterialType::PathPlanningGoal => { materials.add(Color::rgba(0.239, 0.96, 0.43, 0.6).into()).id },
        LynxMaterialType::Interpolate { start_color_rgb, end_color_rgb, u , alpha} => {
            let u_ = u.max(0.0).min(1.0);
            let mut c = Color::rgba(0.0, 0.0, 0.0, alpha as f32);
            c.set_r( ((1.0 - u_) * start_color_rgb[0] + u_ * end_color_rgb[0]) as f32 );
            c.set_g( ((1.0 - u_) * start_color_rgb[1] + u_ * end_color_rgb[1]) as f32 );
            c.set_b( ((1.0 - u_) * start_color_rgb[2] + u_ * end_color_rgb[2]) as f32 );
            materials.add(c.into()).id
        }
    }
}