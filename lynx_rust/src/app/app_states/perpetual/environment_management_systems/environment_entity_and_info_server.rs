use bevy::prelude::{*};
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;

pub struct EnvironmentEntityAndInfoServer {
    _env_object_names: Vec<String>,
    _visible_glb_scene_env_object_entities: Vec<Option<Entity>>,
    _standard_material_env_object_entities: Vec<Option<Entity>>,
    _invisible_material_env_object_entities: Vec<Option<Entity>>,
    _base_env_object_materials: Vec<Option<LynxMaterialType>>,
    _curr_env_object_materials: Vec<Option<LynxMaterialType>>,
    _env_objects_with_changed_materials: Vec<(usize, LynxMaterialType)>,
    _env_objects_with_reset_materials: Vec<(usize, LynxMaterialType)>,
    _env_objects_with_focus: Vec<usize>,
    _env_objects_that_recently_gained_focus: Vec<usize>,
    _env_objects_that_recently_left_focus: Vec<usize>
}

impl EnvironmentEntityAndInfoServer {
    pub fn new() -> Self {
        let _env_object_names = Vec::new();
        let _visible_glb_scene_env_object_entities = Vec::new();
        let _standard_material_env_object_entities = Vec::new();
        let _invisible_material_eenv_object_entities = Vec::new();
        let _base_env_object_materials = Vec::new();
        let _curr_env_object_materials = Vec::new();
        let _env_objects_with_changed_materials = Vec::new();
        let _env_objects_with_reset_materials = Vec::new();
        let _env_objects_with_focus = Vec::new();
        let _env_objects_that_recently_gained_focus = Vec::new();
        let _env_objects_that_recently_left_focus = Vec::new();

        Self {
            _env_object_names,
            _visible_glb_scene_env_object_entities,
            _standard_material_env_object_entities,
            _invisible_material_env_object_entities: _invisible_material_eenv_object_entities,
            _base_env_object_materials,
            _curr_env_object_materials,
            _env_objects_with_changed_materials,
            _env_objects_with_reset_materials,
            _env_objects_with_focus,
            _env_objects_that_recently_gained_focus,
            _env_objects_that_recently_left_focus
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn push_env_object_entities(&mut self,
                                    env_object_name: String,
                                    visible_glb_scene_env_object_entity: Option<Entity>,
                                    standard_material_env_object_entity: Option<Entity>,
                                    invisible_material_env_object_entity: Option<Entity>,
                                    env_object_material: Option<LynxMaterialType>) -> usize {
        let idx = self._env_object_names.len();
        self._env_object_names.push(env_object_name);
        self._visible_glb_scene_env_object_entities.push(visible_glb_scene_env_object_entity);
        self._standard_material_env_object_entities.push(standard_material_env_object_entity);
        self._invisible_material_env_object_entities.push(invisible_material_env_object_entity);
        self._curr_env_object_materials.push(env_object_material.clone());
        self._base_env_object_materials.push(env_object_material);
        return idx;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_visible_glb_scene_env_object_entities_ref(&self) -> &Vec<Option<Entity>> {
        return &self._visible_glb_scene_env_object_entities;
    }

    pub fn get_standard_material_env_object_entites_ref(&self) -> &Vec<Option<Entity>> {
        return &self._standard_material_env_object_entities;
    }

    pub fn get_invisible_material_env_object_entites_ref(&self) -> &Vec<Option<Entity>> {
        return &self._invisible_material_env_object_entities;
    }

    pub fn get_env_object_materials(&self) -> &Vec<Option<LynxMaterialType>> {
        return &self._curr_env_object_materials;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn change_env_object_material_data(&mut self, idx: usize, material: LynxMaterialType) -> Result<(), String> {
        if idx >= self._curr_env_object_materials.len() {
            return Err(format!("idx {} is too high for number of env objects ({:?})", idx, self._curr_env_object_materials.len()));
        }

        if self._curr_env_object_materials[idx].is_none() {
            return Err(format!("curr_env_object_material with idx {:?} is None", idx));
        }

        let curr_material = self._curr_env_object_materials[idx].as_ref().unwrap();
        if &material == curr_material {
            return Ok(())
        }

        return if &material > curr_material {
            self._curr_env_object_materials[idx] = Some(material);
            self._env_objects_with_changed_materials.push((idx, material.clone()));
            Ok(())
        } else {
            Ok(())
        }
    }

    pub fn change_env_object_material_data_force_away_from_particular_materials(&mut self, idx: usize, material: LynxMaterialType, materials_to_change_from: Vec<LynxMaterialType>) -> Result<(), String> {
        if idx >= self._curr_env_object_materials.len() {
            return Err(format!("idx {} is too high for number of env objects ({:?})", idx, self._curr_env_object_materials.len()));
        }

        if self._curr_env_object_materials[idx].is_none() {
            return Err(format!("curr_env_object_material with idx {:?} is None", idx));
        }

        let curr_material = self._curr_env_object_materials[idx].as_ref().unwrap();
        if &material == curr_material {
            return Ok(());
        }

        return if &material > curr_material || materials_to_change_from.contains(curr_material) {
            self._curr_env_object_materials[idx] = Some(material);
            self._env_objects_with_changed_materials.push((idx, material.clone()));
            Ok(())
        } else {
            Ok(())
        }
    }

    pub fn reset_env_object_material_data(&mut self, idx: usize) -> Result<(), String> {
        if idx >= self._curr_env_object_materials.len() {
            return Err(format!("idx {} is too high for number of env objects ({:?})", idx, self._curr_env_object_materials.len()));
        }

        if self._curr_env_object_materials[idx].is_none() {
            return Err(format!("curr_env_object_material with idx {:?} is None", idx));
        }

        self._curr_env_object_materials[idx] = self._base_env_object_materials[idx];
        self._env_objects_with_reset_materials.push((idx, self._base_env_object_materials[idx].as_ref().unwrap().clone()));

        Ok(())
    }

    pub fn reset_env_object_material_data_force_away_from_particular_materials(&mut self, idx: usize, materials_to_change_from: Vec<LynxMaterialType>) -> Result<(), String> {
        if idx >= self._curr_env_object_materials.len() {
            return Err(format!("idx {} is too high for number of env objects ({:?})", idx, self._curr_env_object_materials.len()));
        }

        if self._curr_env_object_materials[idx].is_none() {
            return Err(format!("curr_env_object_material with idx {:?} is None", idx));
        }

        let curr_material = self._curr_env_object_materials[idx].as_ref().unwrap();

        if materials_to_change_from.contains(curr_material) {
            self._curr_env_object_materials[idx] = self._base_env_object_materials[idx];
            self._env_objects_with_reset_materials.push((idx, self._base_env_object_materials[idx].as_ref().unwrap().clone()));
        }

        Ok(())
    }

    /*
    pub fn change_link_material_data_force_away_from_particular_materials(&mut self, robot_set_idx: usize, robot_link_idx: usize, link_material: LynxMaterialType, link_materials_to_change_from: Vec<LynxMaterialType>) -> Result<bool, String> {
        if robot_set_idx >= self._curr_link_materials.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._curr_link_materials.len()));
        }

        if robot_link_idx >= self._curr_link_materials[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._curr_link_materials[robot_set_idx].len()));
        }

        if self._curr_link_materials[robot_set_idx][robot_link_idx].is_none() {
            return Err(format!("curr_link_material with robot_set_idx {:?} and link_idx {:?} is None", robot_set_idx, robot_link_idx));
        }

        let curr_link_material = self._curr_link_materials[robot_set_idx][robot_link_idx].as_ref().unwrap();
        if &link_material == curr_link_material {
            return Ok(false);
        }

        return if &link_material > curr_link_material || link_materials_to_change_from.contains(curr_link_material) {
            self._curr_link_materials[robot_set_idx][robot_link_idx] = Some(link_material);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn reset_link_material_data(&mut self, robot_set_idx: usize, robot_link_idx: usize) -> Result<(), String> {
        if robot_set_idx >= self._curr_link_materials.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._curr_link_materials.len()));
        }

        if robot_link_idx >= self._curr_link_materials[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._curr_link_materials[robot_set_idx].len()));
        }

        if self._curr_link_materials[robot_set_idx][robot_link_idx].is_none() {
            return Err(format!("curr_link_material with robot_set_idx {:?} and link_idx {:?} is None", robot_set_idx, robot_link_idx));
        }

        self._curr_link_materials[robot_set_idx][robot_link_idx] = self._base_link_materials[robot_set_idx][robot_link_idx].clone();

        Ok(())
    }

    pub fn reset_link_material_data_only_if_away_from_particular_materials(&mut self, robot_set_idx: usize, robot_link_idx: usize, link_materials_to_change_from: Vec<LynxMaterialType>) -> Result<bool, String> {
        if robot_set_idx >= self._curr_link_materials.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._curr_link_materials.len()));
        }

        if robot_link_idx >= self._curr_link_materials[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._curr_link_materials[robot_set_idx].len()));
        }

        if self._curr_link_materials[robot_set_idx][robot_link_idx].is_none() {
            return Err(format!("curr_link_material with robot_set_idx {:?} and link_idx {:?} is None", robot_set_idx, robot_link_idx));
        }

        let curr_link_material = self._curr_link_materials[robot_set_idx][robot_link_idx].as_ref().unwrap();

        return if link_materials_to_change_from.contains(curr_link_material) {
            self._curr_link_materials[robot_set_idx][robot_link_idx] = self._base_link_materials[robot_set_idx][robot_link_idx].clone();
            Ok(true)
        } else {
            Ok(false)
        }
    }
     */

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_env_object_entity_pack(&self, idx: usize) -> Result<EnvObjectEntityPack, String> {
        if idx >= self._standard_material_env_object_entities.len() {
            return Err(format!("idx {} is too high for number of env objects ({:?})", idx, self._standard_material_env_object_entities.len()));
        }

        let visible_glb_scene_env_object_entity = self._visible_glb_scene_env_object_entities[idx].clone();
        let standard_material_env_object_entity = self._standard_material_env_object_entities[idx].clone();
        let invisible_material_env_object_entity = self._invisible_material_env_object_entities[idx].clone();

        return Ok( EnvObjectEntityPack {
            visible_glb_scene_env_object_entity,
            standard_material_env_object_entity,
            invisible_material_env_object_entity
        });
    }

    pub fn get_curr_material(&self, idx: usize) -> Result<LynxMaterialType, String> {
        if idx >= self._curr_env_object_materials.len() {
            return Err(format!("idx {} is too high for number of env objects ({:?})", idx, self._curr_env_object_materials.len()));
        }

        if self._curr_env_object_materials[idx].is_none() {
            return Err(format!("curr_env_object_material with idx {:?} is None", idx));
        }

        return Ok(self._curr_env_object_materials[idx].unwrap());
    }

    pub fn get_base_material(&self, idx: usize) -> Result<LynxMaterialType, String> {
        if idx >= self._base_env_object_materials.len() {
            return Err(format!("idx {} is too high for number of env objects ({:?})", idx, self._curr_env_object_materials.len()));
        }

        if self._base_env_object_materials[idx].is_none() {
            return Err(format!("base_env_object_material with idx {:?} is None", idx));
        }

        return Ok(self._base_env_object_materials[idx].unwrap());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn wipe_environment_objects_with_changed_materials_list(&mut self) {
        self._env_objects_with_changed_materials = Vec::new();
    }

    pub fn wipe_environment_objects_with_reset_materials_list(&mut self) {
        self._env_objects_with_reset_materials = Vec::new();
    }

    pub fn wipe_environment_objects_that_recently_gained_focus(&mut self) {
        self._env_objects_that_recently_gained_focus = Vec::new();
    }

    pub fn wipe_environment_objects_that_recently_left_focus(&mut self) {
        self._env_objects_that_recently_left_focus = Vec::new();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_env_object_with_focus(&mut self, idx: usize) {
        Self::_remove_from_list(idx, &mut self._env_objects_with_focus);
        Self::_remove_from_list(idx, &mut self._env_objects_that_recently_gained_focus);
        Self::_remove_from_list(idx, &mut self._env_objects_that_recently_left_focus);

        self._env_objects_that_recently_gained_focus.push(idx);
        self._env_objects_with_focus.push(idx);
    }

    pub fn set_env_object_without_focus(&mut self, idx: usize) {
        Self::_remove_from_list(idx, &mut self._env_objects_with_focus);
        Self::_remove_from_list(idx, &mut self._env_objects_that_recently_gained_focus);
        Self::_remove_from_list(idx, &mut self._env_objects_that_recently_left_focus);

        self._env_objects_that_recently_left_focus.push(idx);
    }

    pub fn set_all_env_objects_without_focus(&mut self) {
        let env_objs_with_focus = self._env_objects_with_focus.clone();
        for o in &env_objs_with_focus {
            self.set_env_object_without_focus(*o);
        }
    }

    fn _remove_from_list(idx: usize, list: &mut Vec<usize>) {
        let mut rmv_idxs = Vec::new();

        let l = list.len();
        for i in 0..l {
            if idx == list[i] {
                rmv_idxs.insert(0, i);
            }
        }

        for r in rmv_idxs {
            list.remove(r);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_env_object_names(&self) -> &Vec<String> {
        return &self._env_object_names;
    }

    pub fn get_changed_env_object_material_pack(&self) -> &Vec<(usize, LynxMaterialType)> {
        return &self._env_objects_with_changed_materials;
    }

    pub fn get_reset_env_object_material_pack(&self) -> &Vec<(usize, LynxMaterialType)> {
        return &self._env_objects_with_reset_materials;
    }

    pub fn get_env_objects_with_focus(&self) -> &Vec<usize> {
        return &self._env_objects_with_focus;
    }

    pub fn get_env_objects_that_recently_gained_focus_ref(&self) -> &Vec<usize> {
        return &self._env_objects_that_recently_gained_focus;
    }

    pub fn get_env_objects_that_recently_left_focus_ref(&self) -> &Vec<usize> {
        return &self._env_objects_that_recently_left_focus;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_num_objects(&self) -> usize {
        return self._env_object_names.len();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn despawn_all(&mut self, commands: &mut Commands) {
        for l in &self._visible_glb_scene_env_object_entities {
            if l.is_some() {
                let e = l.as_ref().unwrap();
                commands.entity(e.clone()).despawn_recursive();
            }
        }

        for l in &self._standard_material_env_object_entities {
            if l.is_some() {
                let e = l.as_ref().unwrap();
                commands.entity(e.clone()).despawn_recursive();
            }
        }

        for l in &self._invisible_material_env_object_entities {
            if l.is_some() {
                let e = l.as_ref().unwrap();
                commands.entity(e.clone()).despawn_recursive();
            }
        }

        *self = Self::new();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Debug, Clone)]
pub struct EnvObjectEntityPack {
    pub visible_glb_scene_env_object_entity: Option<Entity>,
    pub standard_material_env_object_entity: Option<Entity>,
    pub invisible_material_env_object_entity: Option<Entity>
}