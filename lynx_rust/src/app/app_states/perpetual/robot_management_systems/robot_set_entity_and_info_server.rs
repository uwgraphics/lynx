use crate::app::app_states::perpetual::robot_management_systems::robot_management_res_comps::*;
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;
use bevy::prelude::{Commands, Entity, DespawnRecursiveExt};
use std::time::{Instant, Duration};
use nalgebra::DVector;
use std::{thread, time};


pub struct RobotSetEntityAndInfoServer {
    _individual_robot_set_entity_and_info_container: Vec<IndividualRobotSetEntityAndInfoContainer>,
    _links_with_changed_materials: Vec<((usize, usize, usize), LynxMaterialType)>,
    _links_with_reset_materials: Vec<((usize, usize, usize), LynxMaterialType)>,
    _links_with_focus: Vec<(usize, usize, usize)>,
    _links_that_recently_gained_focus: Vec<(usize, usize, usize)>,
    _links_that_recently_left_focus: Vec<(usize, usize, usize)>,
    _hidden_robots: Vec<usize>,
    _robots_that_were_just_hidden: Vec<usize>,
    _robots_that_were_just_unhidden: Vec<usize>
}
impl RobotSetEntityAndInfoServer {
    pub fn new() -> Self {
        let _individual_robot_set_entity_and_info_container = Vec::new();
        let _links_with_changed_materials = Vec::new();
        let _links_with_reset_materials = Vec::new();
        let _links_with_focus = Vec::new();
        let _links_that_recently_gained_focus = Vec::new();
        let _links_that_recently_left_focus = Vec::new();
        let _hidden_robots = Vec::new();
        let _robots_that_were_just_hidden = Vec::new();
        let _robots_that_were_just_unhidden = Vec::new();
        Self { _individual_robot_set_entity_and_info_container,
            _links_with_changed_materials,
            _links_with_reset_materials,
            _links_with_focus,
            _links_that_recently_gained_focus,
            _links_that_recently_left_focus,
            _hidden_robots,
            _robots_that_were_just_hidden,
            _robots_that_were_just_unhidden
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_individual_robot_set_entity_and_info_container(&mut self, robot_set_entity_container: IndividualRobotSetEntityAndInfoContainer) -> usize {
        let idx = self._individual_robot_set_entity_and_info_container.len();
        self._individual_robot_set_entity_and_info_container.push(robot_set_entity_container);
        return idx;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_individual_robot_set_entity_and_info_container_ref(&self, idx: usize) -> Result<&IndividualRobotSetEntityAndInfoContainer, String> {
        if idx >= self._individual_robot_set_entity_and_info_container.len() {
            return Err(format!("idx ({:?}) is too large given the number of robot_set_entity_containers ({:?}) in RobotSetEntityServer", idx, self._individual_robot_set_entity_and_info_container.len()));
        }
        return Ok(&self._individual_robot_set_entity_and_info_container[idx]);
    }

    pub fn get_all_individual_robot_set_entity_and_info_containers_ref(&self) -> &Vec<IndividualRobotSetEntityAndInfoContainer> {
        return &self._individual_robot_set_entity_and_info_container;
    }

    pub fn get_all_individual_robot_set_entity_and_info_containers_mut_ref(&mut self) -> &mut Vec<IndividualRobotSetEntityAndInfoContainer> {
        return &mut self._individual_robot_set_entity_and_info_container;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_num_individual_robot_set_entity_and_info_containers(&self) -> usize { return self._individual_robot_set_entity_and_info_container.len(); }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_entity_from_robot_link_info_container(&self, robot_link_info_container: &RobotLinkInfoContainer) -> &Option<Entity> {
        let server_idx = robot_link_info_container.robot_server_vector_idx;

        let robot_set_entity_containers = &self._individual_robot_set_entity_and_info_container[server_idx];
        return match robot_link_info_container.robot_link_mesh_type {
            RobotLinkMeshType::VislbleGlb => {
                &robot_set_entity_containers._visible_glb_scene_link_entities[robot_link_info_container.robot_set_idx][robot_link_info_container.robot_link_idx]
            }
            RobotLinkMeshType::StandardMaterial => {
                &robot_set_entity_containers._standard_material_link_entities[robot_link_info_container.robot_set_idx][robot_link_info_container.robot_link_idx]
            }
            RobotLinkMeshType::InvisibleMaterial => {
                &None
            }
        }

    }

    pub fn get_link_entity_pack(&self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize) -> Result<RobotLinkEntityPack, String> {
        if robot_server_vector_idx >= self._individual_robot_set_entity_and_info_container.len() {
            return Err(format!("robot_server_vector_idx {:?} is too large given number of robot_set_entity_containers ({:?})", robot_server_vector_idx, self._individual_robot_set_entity_and_info_container.len()));
        }

        let robot_set_entity_container = &self._individual_robot_set_entity_and_info_container[robot_server_vector_idx];
        let _visible_glb_scene_link_entity = robot_set_entity_container._visible_glb_scene_link_entities[robot_set_idx][robot_link_idx].clone();
        let _standard_material_link_entity = robot_set_entity_container._standard_material_link_entities[robot_set_idx][robot_link_idx].clone();
        let _invisible_material_link_entity = robot_set_entity_container._invisible_material_link_entities[robot_set_idx][robot_link_idx].clone();

        return Ok(RobotLinkEntityPack {
            visible_glb_scene_link_entity: _visible_glb_scene_link_entity,
            standard_material_link_entity: _standard_material_link_entity,
            invisible_material_link_entity: _invisible_material_link_entity
        });
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_robot_set_joint_values_ref(&self, robot_server_vector_idx: usize) -> Result<&DVector<f64>, String> {
        let check = self._individual_robot_set_entity_and_info_container.get(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        return Ok(r.get_robot_set_joint_values_ref());
    }

    pub fn get_robot_set_joint_values_mut_ref(&mut self, robot_server_vector_idx: usize) -> Result<&mut DVector<f64>, String> {
        let check = self._individual_robot_set_entity_and_info_container.get_mut(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        return Ok(r.get_robot_set_joint_values_mut_ref());
    }

    pub fn set_robot_set_joint_values_ref(&mut self, robot_server_vector_idx: usize, new_values: &DVector<f64>) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get_mut(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        return r.set_robot_set_joint_values_ref(new_values);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn change_link_material_data(&mut self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize, link_material: LynxMaterialType) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get_mut(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        let change_link_material_res = r.change_link_material_data(robot_set_idx, robot_link_idx, link_material)?;

        if change_link_material_res {
            let mat = r.get_curr_material(robot_set_idx, robot_link_idx)?;
            self._links_with_changed_materials.push( ((robot_server_vector_idx, robot_set_idx, robot_link_idx), mat) )
        };

        Ok(())
    }

    pub fn change_link_material_data_whole_robot_set(&mut self, robot_server_vector_idx: usize, link_material: LynxMaterialType) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        let l = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials.len();
        for i in 0..l {
            let l2 = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials[i].len();
            for j in 0..l2 {
                self.change_link_material_data(robot_server_vector_idx, i, j, link_material.clone());
            }
        }

        Ok(())
    }

    pub fn change_link_base_material_data(&mut self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize, link_material: LynxMaterialType) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get_mut(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        r.change_link_base_material_data(robot_set_idx, robot_link_idx, link_material)?;

        let mat = r.get_curr_material(robot_set_idx, robot_link_idx)?;
        self._links_with_changed_materials.push( ((robot_server_vector_idx, robot_set_idx, robot_link_idx), mat) );

        Ok(())
    }

    pub fn change_link_base_material_data_whole_robot_set(&mut self, robot_server_vector_idx: usize, link_material: LynxMaterialType) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        let l = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials.len();
        for i in 0..l {
            let l2 = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials[i].len();
            for j in 0..l2 {
                self.change_link_base_material_data(robot_server_vector_idx, i, j, link_material.clone());
            }
        }

        Ok(())
    }

    pub fn change_link_material_data_force_away_from_particular_materials(&mut self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize, link_material: LynxMaterialType, link_materials_to_change_from: Vec<LynxMaterialType>) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get_mut(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        let change_link_material_res = r.change_link_material_data_force_away_from_particular_materials(robot_set_idx, robot_link_idx, link_material, link_materials_to_change_from)?;

        if change_link_material_res {
            let mat = r.get_curr_material(robot_set_idx, robot_link_idx)?;
            self._links_with_changed_materials.push( ((robot_server_vector_idx, robot_set_idx, robot_link_idx), mat) )
        };

        Ok(())
    }

    pub fn change_link_material_data_force_away_from_particular_materials_whole_robot_set(&mut self, robot_server_vector_idx: usize, link_material: LynxMaterialType, link_materials_to_change_from: Vec<LynxMaterialType>) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        let l = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials.len();
        for i in 0..l {
            let l2 = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials[i].len();
            for j in 0..l2 {
                self.change_link_material_data_force_away_from_particular_materials(robot_server_vector_idx, i, j, link_material.clone(), link_materials_to_change_from.clone());
            }
        }

        Ok(())
    }

    pub fn reset_link_material_data(&mut self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get_mut(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        r.reset_link_material_data(robot_set_idx, robot_link_idx)?;
        let mat = r.get_base_material(robot_set_idx, robot_link_idx)?;

        self._links_with_reset_materials.push( ((robot_server_vector_idx, robot_set_idx, robot_link_idx), mat) );

        Ok(())
    }

    pub fn reset_link_material_data_whole_robot_set(&mut self, robot_server_vector_idx: usize) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        let l = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials.len();
        for i in 0..l {
            let l2 = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials[i].len();
            for j in 0..l2 {
                self.reset_link_material_data(robot_server_vector_idx, i, j);
            }
        }

        Ok(())
    }

    pub fn reset_link_material_data_only_if_away_from_particular_materials(&mut self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize, link_materials_to_change_from: Vec<LynxMaterialType>) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get_mut(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        let res = r.reset_link_material_data_only_if_away_from_particular_materials(robot_set_idx, robot_link_idx, link_materials_to_change_from)?;
        if res {
            // let d = time::Duration::from_micros(3);
            // thread::sleep(d);

            let mat = r.get_base_material(robot_set_idx, robot_link_idx)?;

            self._links_with_reset_materials.push( ((robot_server_vector_idx, robot_set_idx, robot_link_idx), mat) );

        }

        Ok(())
    }

    pub fn reset_link_material_data_only_if_away_from_particular_materials_whole_robot_set(&mut self, robot_server_vector_idx: usize, link_materials_to_change_from: Vec<LynxMaterialType>) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        let l = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials.len();
        for i in 0..l {
            let l2 = self._individual_robot_set_entity_and_info_container[robot_server_vector_idx]._curr_link_materials[i].len();
            for j in 0..l2 {
                self.reset_link_material_data_only_if_away_from_particular_materials(robot_server_vector_idx, i, j, link_materials_to_change_from.clone());
            }
        }

        Ok(())
    }

    pub fn get_curr_material(&self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize) -> Result<LynxMaterialType, String> {
        let check = self._individual_robot_set_entity_and_info_container.get(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        return r.get_curr_material(robot_set_idx, robot_link_idx);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_changed_link_material_pack(&self) -> &Vec<((usize, usize, usize), LynxMaterialType)> {
        return &self._links_with_changed_materials;
    }

    pub fn get_reset_link_material_pack(&self) -> &Vec<((usize, usize, usize), LynxMaterialType)> {
        return &self._links_with_reset_materials;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn wipe_links_with_changed_materials_list(&mut self) {
        self._links_with_changed_materials = Vec::new();
    }

    pub fn wipe_links_with_reset_materials_list(&mut self) {
        self._links_with_reset_materials = Vec::new();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_link_with_focus(&mut self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize) {
        Self::_remove_from_list(robot_server_vector_idx, robot_set_idx, robot_link_idx, &mut self._links_with_focus);
        Self::_remove_from_list(robot_server_vector_idx, robot_set_idx, robot_link_idx, &mut self._links_that_recently_gained_focus);
        Self::_remove_from_list(robot_server_vector_idx, robot_set_idx, robot_link_idx, &mut self._links_that_recently_left_focus);

        self._links_that_recently_gained_focus.push((robot_server_vector_idx, robot_set_idx, robot_link_idx));
        self._links_with_focus.push((robot_server_vector_idx, robot_set_idx, robot_link_idx));
    }

    pub fn set_link_without_focus(&mut self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize) {
        Self::_remove_from_list(robot_server_vector_idx, robot_set_idx, robot_link_idx, &mut self._links_with_focus);
        Self::_remove_from_list(robot_server_vector_idx, robot_set_idx, robot_link_idx, &mut self._links_that_recently_gained_focus);
        Self::_remove_from_list(robot_server_vector_idx, robot_set_idx, robot_link_idx, &mut self._links_that_recently_left_focus);

        self._links_that_recently_left_focus.push((robot_server_vector_idx, robot_set_idx, robot_link_idx));
    }

    pub fn set_all_links_without_focus(&mut self) {
        let links_with_focus = self._links_with_focus.clone();
        for l in &links_with_focus {
            self.set_link_without_focus(l.0, l.1, l.2);
        }
    }

    fn _remove_from_list(robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize, list: &mut Vec<(usize, usize, usize)>) {

        let mut rmv_idxs = Vec::new();

        let l = list.len();
        for i in 0..l {
            if robot_link_idx == list[i].2 && robot_set_idx == list[i].1 && robot_server_vector_idx == list[i].0 {
                rmv_idxs.insert(0, i);
            }
        }

        for r in rmv_idxs {
            list.remove(r);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_links_with_focus_ref(&self) -> &Vec<(usize, usize, usize)> {
        return &self._links_with_focus;
    }

    pub fn get_links_that_recently_gained_focus_ref(&self) -> &Vec<(usize, usize, usize)> {
        return &self._links_that_recently_gained_focus;
    }

    pub fn get_links_that_recently_left_focus_ref(&self) -> &Vec<(usize, usize, usize)> {
        return &self._links_that_recently_left_focus;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn wipe_links_that_recently_gained_focus(&mut self) {
        self._links_that_recently_gained_focus = Vec::new();
    }

    pub fn wipe_links_that_recently_left_focus(&mut self) {
        self._links_that_recently_left_focus = Vec::new();
    }

    /*
    pub fn set_has_focus(&mut self, robot_server_vector_idx: usize, robot_set_idx: usize, robot_link_idx: usize, has_focus: bool) -> Result<(), String> {
        let check = self._individual_robot_set_entity_and_info_container.get_mut(robot_server_vector_idx);
        let mut r = if check.is_some() { check.unwrap() } else { return Err(format!("robot_server_vector_idx ({:?}) is too high", robot_server_vector_idx)) };

        r.set_has_focus(robot_set_idx, robot_link_idx, has_focus);

        Ok(())
    }

    pub fn set_all_has_focus_to_false(&mut self) {
        let l = self._individual_robot_set_entity_and_info_container.len();
        for i in 0..l {
            self._individual_robot_set_entity_and_info_container[i].set_all_has_focus_to_false();
        }
    }
     */

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn hide_robot(&mut self, robot_server_vector_idx: usize) -> Result<(), String> {
        if robot_server_vector_idx >= self.get_num_individual_robot_set_entity_and_info_containers() {
            return Err(format!("robot_server_vector_idx ({:?}) is too high given the number of robots ({:?})", robot_server_vector_idx, self.get_num_individual_robot_set_entity_and_info_containers()));
        }

        Self::_remove_from_hide_related_list(robot_server_vector_idx, &mut self._robots_that_were_just_unhidden);
        Self::_remove_from_hide_related_list(robot_server_vector_idx, &mut self._robots_that_were_just_hidden);
        Self::_remove_from_hide_related_list(robot_server_vector_idx, &mut self._hidden_robots);

        /*
        let l = self._links_with_focus.len();
        for i in 0..l {
            if self._links_with_focus[i].0 == robot_server_vector_idx {
                self.set_link_without_focus(self._links_with_focus[i].0, self._links_with_focus[i].1, self._links_with_focus[i].2);
            }
        }
        */

        self._hidden_robots.push(robot_server_vector_idx);
        self._robots_that_were_just_hidden.push(robot_server_vector_idx);

        Ok(())
    }

    pub fn unhide_robot(&mut self, robot_server_vector_idx: usize) -> Result<(), String> {
        if robot_server_vector_idx >= self.get_num_individual_robot_set_entity_and_info_containers() {
            return Err(format!("robot_server_vector_idx ({:?}) is too high given the number of robots ({:?})", robot_server_vector_idx, self.get_num_individual_robot_set_entity_and_info_containers()));
        }

        Self::_remove_from_hide_related_list(robot_server_vector_idx, &mut self._robots_that_were_just_unhidden);
        Self::_remove_from_hide_related_list(robot_server_vector_idx, &mut self._robots_that_were_just_hidden);
        Self::_remove_from_hide_related_list(robot_server_vector_idx, &mut self._hidden_robots);

        self._robots_that_were_just_unhidden.push(robot_server_vector_idx);

        Ok(())
    }

    pub fn wipe_robots_that_were_just_hidden(&mut self) {
        self._robots_that_were_just_hidden = Vec::new();
    }

    pub fn wipe_robots_that_were_just_unhidden(&mut self) {
        self._robots_that_were_just_unhidden = Vec::new();
    }

    pub fn is_robot_hidden(&self, robot_server_vector_idx: usize) -> bool {
        return self._hidden_robots.contains(&robot_server_vector_idx);
    }

    pub fn get_hidden_robot_idxs(&self) -> &Vec<usize> {
        return &self._hidden_robots;
    }

    pub fn get_robots_that_were_just_hidden_idxs(&self) -> &Vec<usize> {
        return &self._robots_that_were_just_hidden;
    }

    pub fn get_robots_that_were_just_unhidden_idxs(&self) -> &Vec<usize> {
        return &self._robots_that_were_just_unhidden;
    }

    fn _remove_from_hide_related_list(robot_server_vector_idx: usize, list: &mut Vec<usize>) {
        let mut rmv_idxs = Vec::new();

        let l = list.len();
        for i in 0..l {
            if robot_server_vector_idx == list[i] {
                rmv_idxs.insert(0, i);
            }
        }

        for r in rmv_idxs {
            list.remove(r);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn despawn_all(&mut self, commands: &mut Commands) {
        self._individual_robot_set_entity_and_info_container.iter().for_each(|x| x.despawn(commands));

        *self = Self::new();
    }

    pub fn despawn(&mut self, robot_server_vector_idx: usize, commands: &mut Commands) -> Result<(), String> {
        if robot_server_vector_idx >= self._individual_robot_set_entity_and_info_container.len() {
            return Err(format!("robot_server_vector_idx {} is too high for number of robots on server ({:?})", robot_server_vector_idx, self._individual_robot_set_entity_and_info_container));
        }

        self._individual_robot_set_entity_and_info_container[robot_server_vector_idx].despawn(commands);

        Self::_remove_from_list_with_given_robot_server_vector_idx1(robot_server_vector_idx, &mut self._links_with_focus);
        Self::_remove_from_list_with_given_robot_server_vector_idx1(robot_server_vector_idx, &mut self._links_that_recently_left_focus);
        Self::_remove_from_list_with_given_robot_server_vector_idx1(robot_server_vector_idx, &mut self._links_that_recently_gained_focus);
        Self::_remove_from_list_with_given_robot_server_vector_idx2(robot_server_vector_idx, &mut self._links_with_changed_materials);
        Self::_remove_from_list_with_given_robot_server_vector_idx2(robot_server_vector_idx, &mut self._links_with_reset_materials);
        Self::_remove_from_list_with_given_robot_server_vector_idx3(robot_server_vector_idx, &mut self._hidden_robots);
        Self::_remove_from_list_with_given_robot_server_vector_idx3(robot_server_vector_idx, &mut self._robots_that_were_just_hidden);
        Self::_remove_from_list_with_given_robot_server_vector_idx3(robot_server_vector_idx, &mut self._robots_that_were_just_unhidden);

        Ok(())
    }

    fn _remove_from_list_with_given_robot_server_vector_idx1(robot_server_vector_idx: usize, list: &mut Vec<(usize, usize, usize)>) {
        let mut rmv_idxs = Vec::new();

        let l = list.len();
        for i in 0..l {
            if list[i].0 == robot_server_vector_idx {
                rmv_idxs.insert(0, i);
            }
        }

        for r in rmv_idxs {
            list.remove(r);
        }
    }

    fn _remove_from_list_with_given_robot_server_vector_idx2(robot_server_vector_idx: usize, list: &mut Vec<((usize, usize, usize), LynxMaterialType)>) {
        let mut rmv_idxs = Vec::new();

        let l = list.len();
        for i in 0..l {
            if (list[i].0).0 == robot_server_vector_idx {
                rmv_idxs.insert(0, i);
            }
        }

        for r in rmv_idxs {
            list.remove(r);
        }
    }

    fn _remove_from_list_with_given_robot_server_vector_idx3(robot_server_vector_idx: usize, list: &mut Vec<usize>) {
        let mut rmv_idxs = Vec::new();

        let l = list.len();
        for i in 0..l {
            if list[i] == robot_server_vector_idx {
                rmv_idxs.insert(0, i);
            }
        }

        for r in rmv_idxs {
            list.remove(r);
        }
    }

    pub fn purge_robot_server_vectors(&mut self, robot_server_vector_idx: usize) {
        Self::_remove_from_list_with_given_robot_server_vector_idx1(robot_server_vector_idx, &mut self._links_with_focus);
        Self::_remove_from_list_with_given_robot_server_vector_idx1(robot_server_vector_idx, &mut self._links_that_recently_left_focus);
        Self::_remove_from_list_with_given_robot_server_vector_idx1(robot_server_vector_idx, &mut self._links_that_recently_gained_focus);
        Self::_remove_from_list_with_given_robot_server_vector_idx2(robot_server_vector_idx, &mut self._links_with_changed_materials);
        Self::_remove_from_list_with_given_robot_server_vector_idx2(robot_server_vector_idx, &mut self._links_with_reset_materials);
        // Self::_remove_from_list_with_given_robot_server_vector_idx3(robot_server_vector_idx, &mut self._hidden_robots);
        // Self::_remove_from_list_with_given_robot_server_vector_idx3(robot_server_vector_idx, &mut self._robots_that_were_just_hidden);
        // Self::_remove_from_list_with_given_robot_server_vector_idx3(robot_server_vector_idx, &mut self._robots_that_were_just_unhidden);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Debug, Clone)]
pub struct IndividualRobotSetEntityAndInfoContainer {
    _robot_set_joint_values: DVector<f64>,
    _visible_glb_scene_link_entities: Vec<Vec<Option<Entity>>>,
    _standard_material_link_entities: Vec<Vec<Option<Entity>>>,
    _invisible_material_link_entities: Vec<Vec<Option<Entity>>>,
    _base_link_materials: Vec<Vec<Option<LynxMaterialType>>>,
    _curr_link_materials: Vec<Vec<Option<LynxMaterialType>>>
}
impl IndividualRobotSetEntityAndInfoContainer {
    pub fn new(robot_set_num_dof: usize) -> Self {
        let _robot_set_joint_values = DVector::from_element(robot_set_num_dof, 0.0);
        let _visible_glb_scene_link_entities = Vec::new();
        let _standard_material_link_entities = Vec::new();
        let _invisible_material_link_entities = Vec::new();
        let _base_link_materials = Vec::new();
        let _curr_link_materials = Vec::new();
        // let _has_focus = Vec::new();
        // let _just_given_focus = Vec::new();
        // let _just_given_focus_check_count = Vec::new();
        // let _just_left_focus = Vec::new();
        // let _just_left_focus_check_count = Vec::new();

        Self { _robot_set_joint_values,
            _visible_glb_scene_link_entities,
            _standard_material_link_entities,
            _invisible_material_link_entities,
            _base_link_materials,
            _curr_link_materials}
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn push_link_entities(&mut self, robot_idx: usize,
                              visible_glb_scene_link_entity: Option<Entity>,
                              standard_material_link_entity: Option<Entity>,
                              invisible_material_link_entities: Option<Entity>,
                              link_material: Option<LynxMaterialType>) {

        while self._visible_glb_scene_link_entities.len() <= robot_idx {
            self._visible_glb_scene_link_entities.push(Vec::new());
            self._standard_material_link_entities.push(Vec::new());
            self._invisible_material_link_entities.push(Vec::new());
            self._curr_link_materials.push(Vec::new());
        }

        self._visible_glb_scene_link_entities[robot_idx].push(visible_glb_scene_link_entity);
        self._standard_material_link_entities[robot_idx].push(standard_material_link_entity);
        self._invisible_material_link_entities[robot_idx].push(invisible_material_link_entities);

        self._curr_link_materials[robot_idx].push(link_material.clone());
        self._base_link_materials[robot_idx].push(link_material);

        // self._has_focus[robot_idx].push(false);
        // self._just_given_focus[robot_idx].push(false);
        // self._just_given_focus_check_count[robot_idx].push(0);
        // self._just_left_focus[robot_idx].push(false);
        // self._just_left_focus_check_count[robot_idx].push(0);
    }

    pub fn push_all_link_entities_from_one_robot(&mut self,
                                                 visible_glb_scene_link_entities: Vec<Option<Entity>>,
                                                 standard_material_link_entities: Vec<Option<Entity>>,
                                                 invisible_material_link_entities: Vec<Option<Entity>>,
                                                 link_materials: Vec<Option<LynxMaterialType>>) {

        let l = visible_glb_scene_link_entities.len();

        self._visible_glb_scene_link_entities.push(visible_glb_scene_link_entities);
        self._standard_material_link_entities.push(standard_material_link_entities);
        self._invisible_material_link_entities.push(invisible_material_link_entities);
        self._curr_link_materials.push(link_materials.clone());
        self._base_link_materials.push(link_materials);

        // self._has_focus.push(vec![false; l]);
        // self._just_given_focus.push(vec![false; l]);
        // self._just_given_focus_check_count.push(vec![0; l]);
        // self._just_left_focus.push(vec![false; l]);
        // self._just_left_focus_check_count.push(vec![0; l]);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_num_robots_in_set(&self) -> usize {
        return self._standard_material_link_entities.len();
    }

    pub fn get_num_links_in_robot(&self, idx: usize) -> Result<usize, String> {
        if idx >= self._standard_material_link_entities[idx].len() {
            return Err(format!("idx ({:?}) is too high for number of robots ({:?})", idx, self._standard_material_link_entities[idx].len()));
        }

        return Ok(self._standard_material_link_entities[idx].len());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_visible_glb_scene_link_entities_ref(&self) -> &Vec<Vec<Option<Entity>>> {
        return &self._visible_glb_scene_link_entities;
    }

    pub fn get_standard_material_link_entites_ref(&self) -> &Vec<Vec<Option<Entity>>> {
        return &self._standard_material_link_entities;
    }

    pub fn get_invisible_material_link_entites_ref(&self) -> &Vec<Vec<Option<Entity>>> {
        return &self._invisible_material_link_entities;
    }

    pub fn get_link_materials(&self) -> &Vec<Vec<Option<LynxMaterialType>>> {
        return &self._curr_link_materials;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_robot_set_joint_values_ref(&self) -> &DVector<f64> {
        return &self._robot_set_joint_values;
    }

    pub fn get_robot_set_joint_values_mut_ref(&mut self) -> &mut DVector<f64> {
        return &mut self._robot_set_joint_values;
    }

    pub fn set_robot_set_joint_values_ref(&mut self, new_robot_set_joint_values: &DVector<f64>) -> Result<(), String> {
        let l = self._robot_set_joint_values.len();
        if new_robot_set_joint_values.len() != l {
            return Err(format!("new_robot_set_joint_values does not have the right number of DOFs ({:?} instead of {:?})", l, new_robot_set_joint_values.len()));
        }
        self._robot_set_joint_values = new_robot_set_joint_values.clone();

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn change_link_material_data(&mut self, robot_set_idx: usize, robot_link_idx: usize, link_material: LynxMaterialType) -> Result<bool, String> {
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

        return if &link_material > curr_link_material {
            self._curr_link_materials[robot_set_idx][robot_link_idx] = Some(link_material);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn change_link_base_material_data(&mut self, robot_set_idx: usize, robot_link_idx: usize, link_material: LynxMaterialType) -> Result<(), String> {
        if robot_set_idx >= self._curr_link_materials.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._curr_link_materials.len()));
        }

        if robot_link_idx >= self._curr_link_materials[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._curr_link_materials[robot_set_idx].len()));
        }

        if self._curr_link_materials[robot_set_idx][robot_link_idx].is_none() {
            return Err(format!("curr_link_material with robot_set_idx {:?} and link_idx {:?} is None", robot_set_idx, robot_link_idx));
        }

        self._curr_link_materials[robot_set_idx][robot_link_idx] = Some(link_material.clone());
        self._base_link_materials[robot_set_idx][robot_link_idx] = Some(link_material.clone());

        return Ok(())
    }

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

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_curr_material(&self, robot_set_idx: usize, robot_link_idx: usize) -> Result<LynxMaterialType, String> {
        if robot_set_idx >= self._curr_link_materials.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._curr_link_materials.len()));
        }

        if robot_link_idx >= self._curr_link_materials[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._curr_link_materials[robot_set_idx].len()));
        }

        if self._curr_link_materials[robot_set_idx][robot_link_idx].is_none() {
            return Err(format!("curr_link_material with robot_set_idx {:?} and link_idx {:?} is None", robot_set_idx, robot_link_idx));
        }

        return Ok( self._curr_link_materials[robot_set_idx][robot_link_idx].as_ref().unwrap().clone() );
    }

    pub fn get_base_material(&self, robot_set_idx: usize, robot_link_idx: usize) -> Result<LynxMaterialType, String> {
        if robot_set_idx >= self._base_link_materials.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._base_link_materials.len()));
        }

        if robot_link_idx >= self._base_link_materials[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._base_link_materials[robot_set_idx].len()));
        }

        if self._base_link_materials[robot_set_idx][robot_link_idx].is_none() {
            return Err(format!("curr_link_material with robot_set_idx {:?} and link_idx {:?} is None", robot_set_idx, robot_link_idx));
        }

        return Ok( self._base_link_materials[robot_set_idx][robot_link_idx].as_ref().unwrap().clone() );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    /*
    pub fn set_has_focus(&mut self, robot_set_idx: usize, robot_link_idx: usize, has_focus: bool) -> Result<(), String> {
        if robot_set_idx >= self._has_focus.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._has_focus.len()));
        }

        if robot_link_idx >= self._has_focus[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._has_focus[robot_set_idx].len()));
        }

        self._has_focus[robot_set_idx][robot_link_idx] = has_focus;
        if has_focus {
            self._just_given_focus[robot_set_idx][robot_link_idx] = true;
            self._just_given_focus_check_count[robot_set_idx][robot_link_idx] = 0;
            self._just_left_focus[robot_set_idx][robot_link_idx] = false;
            self._just_left_focus_check_count[robot_set_idx][robot_link_idx] = 0;
        } else {
            self._just_given_focus[robot_set_idx][robot_link_idx] = false;
            self._just_given_focus_check_count[robot_set_idx][robot_link_idx] = 0;
            self._just_left_focus[robot_set_idx][robot_link_idx] = true;
            self._just_left_focus_check_count[robot_set_idx][robot_link_idx] = 0;
        }

        Ok(())
    }

    pub fn set_all_has_focus_to_false(&mut self) {
        let l = self._has_focus.len();
        for i in 0..l {
            let l2 = self._has_focus[i].len();
            for j in 0..l2 {
                self._has_focus[i][j] = false;
                self._just_given_focus[i][j] = false;
                self._just_given_focus_check_count[i][j] = 0;
                self._just_left_focus[i][j] = true;
                self._just_left_focus_check_count[i][j] = 0;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_has_focus(&self, robot_set_idx: usize, robot_link_idx: usize) -> Result<bool, String> {
        if robot_set_idx >= self._has_focus.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._has_focus.len()));
        }

        if robot_link_idx >= self._has_focus[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._has_focus[robot_set_idx].len()));
        }

        return Ok(self._has_focus[robot_set_idx][robot_link_idx]);
    }

    pub fn get_has_focus_mut_ref(&mut self, robot_set_idx: usize, robot_link_idx: usize) -> Result<&mut bool, String> {
        if robot_set_idx >= self._has_focus.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._has_focus.len()));
        }

        if robot_link_idx >= self._has_focus[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._has_focus[robot_set_idx].len()));
        }

        return Ok(&mut self._has_focus[robot_set_idx][robot_link_idx]);
    }

    pub fn get_just_given_focus(&mut self, robot_set_idx: usize, robot_link_idx: usize) -> Result<bool, String> {
        if robot_set_idx >= self._just_given_focus.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._just_given_focus.len()));
        }

        if robot_link_idx >= self._just_given_focus[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._just_given_focus[robot_set_idx].len()));
        }

        let mut count = &mut self._just_given_focus_check_count[robot_set_idx][robot_link_idx];
        let mut just_given_focus = &mut self._just_given_focus[robot_set_idx][robot_link_idx];

        if *count >= 5 {
            *just_given_focus = false;
            *count = 0;
        }

        return Ok(*just_given_focus);
    }

    pub fn get_just_left_focus(&mut self, robot_set_idx: usize, robot_link_idx: usize) -> Result<bool, String> {
        if robot_set_idx >= self._just_left_focus.len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of robots in set ({:?})", robot_set_idx, self._just_given_focus.len()));
        }

        if robot_link_idx >= self._just_left_focus[robot_set_idx].len() {
            return Err(format!("robot_set_idx ({:?}) is too high for number of links in robot ({:?})", robot_set_idx, self._just_given_focus[robot_set_idx].len()));
        }

        let mut count = &mut self._just_left_focus_check_count[robot_set_idx][robot_link_idx];
        let mut just_left_focus = &mut self._just_left_focus[robot_set_idx][robot_link_idx];

        if *count >= 5 {
            *just_left_focus = false;
            *count = 0;
        }

        return Ok(*just_left_focus);
    }
     */

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn despawn(&self, commands: &mut Commands) {
        for r in &self._visible_glb_scene_link_entities {
            for l in r {
                if l.is_some() {
                    let e = l.as_ref().unwrap();
                    commands.entity(e.clone()).despawn_recursive();
                }
            }
        }

        for r in &self._standard_material_link_entities {
            for l in r {
                if l.is_some() {
                    let e = l.as_ref().unwrap();
                    commands.entity(e.clone()).despawn_recursive();
                }
            }
        }

        for r in &self._invisible_material_link_entities {
            for l in r {
                if l.is_some() {
                    let e = l.as_ref().unwrap();
                    commands.entity(e.clone()).despawn_recursive();
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Debug, Clone)]
pub struct RobotLinkEntityPack {
    pub visible_glb_scene_link_entity: Option<Entity>,
    pub standard_material_link_entity: Option<Entity>,
    pub invisible_material_link_entity: Option<Entity>
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
