use crate::app::app_states::res_comps::RobotSetEntityAndInfoServer;
use crate::app::app_utils::robot_utils::robot_link_materials_utils::*;
use bevy::prelude::{ResMut, Res, AssetServer, Assets, StandardMaterial, Query, Transform, Visible, Handle};

pub fn robot_link_material_manager_system(mut robot_set_entity_server: ResMut<RobotSetEntityAndInfoServer>,
                                          asset_server: Res<AssetServer>,
                                          mut materials: ResMut<Assets<StandardMaterial>>,
                                          mut q1: Query<(&mut Transform)>,
                                          mut q2: Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {

    let changed_materials_pack = robot_set_entity_server.get_changed_link_material_pack().clone();
    for m in &changed_materials_pack {
        change_material_of_robot_link(m.1,
                                      &asset_server,
                                      &mut materials,
                                      &mut robot_set_entity_server,
                                      (m.0).0,
                                      (m.0).1,
                                      (m.0).2,
                                      &mut q1,
                                      &mut q2);
    }
    let reset_materials_pack = robot_set_entity_server.get_reset_link_material_pack().clone();
    // println!("{:?}", changed_materials_pack);
    // println!("{:?}", reset_materials_pack);
    // println!("--------------------");
    for m in &reset_materials_pack {
        let mut found_in_changed_materials_pack = false;
        let l = changed_materials_pack.len();
        for j in 0..l {
            if changed_materials_pack[j].0 == m.0 { found_in_changed_materials_pack = true; }
        }

        if !found_in_changed_materials_pack {
            reset_material_of_robot_link(m.1,
                                         &asset_server,
                                         &mut materials,
                                         &mut robot_set_entity_server,
                                         (m.0).0,
                                         (m.0).1,
                                         (m.0).2,
                                         &mut q1,
                                         &mut q2);
        }
    }

    robot_set_entity_server.wipe_links_with_changed_materials_list();
    robot_set_entity_server.wipe_links_with_reset_materials_list();
}