use crate::app_v2::app_type_enums::enums::{LynxMaterialType, LynxMaterialChangeType};

#[derive(Clone, Debug)]
pub struct LynxMaterialUser {
    pub base_material: LynxMaterialType,
    pub curr_material: LynxMaterialType
}
impl LynxMaterialUser {
    pub fn new(base_material: &Option<LynxMaterialType>) -> Self {
        if base_material.is_some() {
            Self {
                base_material: base_material.unwrap().clone(),
                curr_material: base_material.unwrap().clone()
            }
        } else {
            Self {
                base_material: LynxMaterialType::Default,
                curr_material: LynxMaterialType::Default
            }
        }
    }

    pub fn change_material(&mut self, change_material_type: LynxMaterialChangeType) -> bool {
        match change_material_type {
            LynxMaterialChangeType::ForceChange { material_to_change_to } => {
                return if material_to_change_to == self.curr_material { false }
                else { self.curr_material = material_to_change_to.clone(); true }
            }
            LynxMaterialChangeType::ChangeWithImportanceCheck { material_to_change_to } => {
                return if material_to_change_to == self.curr_material { false }
                else {
                    return if material_to_change_to > self.curr_material { self.curr_material = material_to_change_to.clone(); true }
                    else { false }
                }
            }
            LynxMaterialChangeType::ForceChangeOnlyFromGivenMaterials { material_to_change_to, materials_to_change_from } => {
                return if material_to_change_to == self.curr_material { false }
                else {
                    return if materials_to_change_from.contains(&self.curr_material) { self.curr_material = material_to_change_to.clone(); true }
                    else { false }
                }
            }
            LynxMaterialChangeType::ForceResetToBase => {
                self.curr_material = self.base_material.clone(); true
            }
            LynxMaterialChangeType::ResetToBaseWithImportanceCheck => {
                return if self.base_material > self.curr_material { self.curr_material = self.base_material.clone(); true }
                else { false }
            }
            LynxMaterialChangeType::ForceResetToBaseOnlyFromGivenMaterials { materials_to_change_from } => {
                return if materials_to_change_from.contains(&self.curr_material) { self.curr_material = self.base_material.clone(); true }
                else { false }
            }
        }
    }
}