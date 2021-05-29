use bevy::asset::HandleId;
use bevy::prelude::{ResMut, StandardMaterial, Assets, Color};

// materials listed lower in the enum are considered "greater" (will override colors that are "less than")
#[derive(PartialEq, Clone, Debug, Copy, PartialOrd)]
pub enum LynxMaterialType {
    Interpolate{ start_color_rgb: [f64; 3], end_color_rgb: [f64; 3], u: f64, alpha: f64 },
    Default,
    VisualizationDefault,
    PathPlanningStart,
    PathPlanningGoal,
    Focus,
    Collision,
    Hover,
    FocusWindowHover,
    Selection,
    CollisionHighlight1,
    CollisionHighlight2,
}
impl LynxMaterialType {
    pub fn get_material_handle_id(&self, materials: &mut ResMut<Assets<StandardMaterial>>) -> HandleId {
        return match self {
            LynxMaterialType::Interpolate { start_color_rgb, end_color_rgb, u , alpha} => {
                let u_ = u.max(0.0).min(1.0);
                let mut c = Color::rgba(0.0, 0.0, 0.0, *alpha as f32);
                c.set_r( ((1.0 - u_) * start_color_rgb[0] + u_ * end_color_rgb[0]) as f32 );
                c.set_g( ((1.0 - u_) * start_color_rgb[1] + u_ * end_color_rgb[1]) as f32 );
                c.set_b( ((1.0 - u_) * start_color_rgb[2] + u_ * end_color_rgb[2]) as f32 );
                materials.add(c.into()).id
            }
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
        }
    }
}

#[derive(PartialEq, Clone, Debug, PartialOrd)]
pub enum LynxMaterialChangeType {
    ForceChange { material_to_change_to: LynxMaterialType },
    ChangeWithImportanceCheck { material_to_change_to: LynxMaterialType },
    ForceChangeOnlyFromGivenMaterials { material_to_change_to: LynxMaterialType, materials_to_change_from: Vec<LynxMaterialType> },
    ForceResetToBase,
    ResetToBaseWithImportanceCheck,
    ForceResetToBaseOnlyFromGivenMaterials { materials_to_change_from: Vec<LynxMaterialType> }
}
impl LynxMaterialChangeType {
    pub fn change_or_reset(&self) -> LynxMaterialChangeOrReset {
        return match self {
            LynxMaterialChangeType::ForceChange { .. } => { LynxMaterialChangeOrReset::Change }
            LynxMaterialChangeType::ChangeWithImportanceCheck { .. } => { LynxMaterialChangeOrReset::Change }
            LynxMaterialChangeType::ForceChangeOnlyFromGivenMaterials { .. } => { LynxMaterialChangeOrReset::Change }
            LynxMaterialChangeType::ForceResetToBase => { LynxMaterialChangeOrReset::Reset }
            LynxMaterialChangeType::ResetToBaseWithImportanceCheck => { LynxMaterialChangeOrReset::Reset }
            LynxMaterialChangeType::ForceResetToBaseOnlyFromGivenMaterials { .. } => { LynxMaterialChangeOrReset::Reset }
        }
    }
}

#[derive(PartialEq, Clone, Debug, PartialOrd)]
pub enum LynxMaterialChangeOrReset {
    Change, Reset
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LineType {
    Visualization,
    Grid
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum CameraType {
    PanOrbitCamera
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RobotLinkSpawnType {
    StandardMaterial,
    SelectableInvisible,
    VisibleGlb
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RobotSetSpawnType {
    Visualization,
    Physical
}

