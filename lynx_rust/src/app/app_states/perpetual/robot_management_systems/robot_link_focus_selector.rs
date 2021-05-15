use bevy::prelude::{EventReader, ResMut, Query, Res, Input, MouseButton, KeyCode};
use bevy_mod_picking::{PickingEvent, HoverEvent, PickingCamera};
use crate::app::app_states::res_comps::{RobotSetEntityAndInfoServer, RobotLinkInfoContainer};
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;

pub fn robot_link_focus_selector(pick_source_query: Query<&PickingCamera>,
                                 mut robot_set_entity_server: ResMut<RobotSetEntityAndInfoServer>,
                                 mut query: Query<(&mut RobotLinkInfoContainer)>,
                                 btn: Res<Input<MouseButton>>,
                                 key: Res<Input<KeyCode>>) {

    if btn.just_pressed(MouseButton::Left) {
        for pick_source in pick_source_query.iter() {
            let e = pick_source.intersect_top();
            if e.is_some() {
                let e_ = e.unwrap().0;
                let q = query.get_mut(e_);
                if q.is_ok() {
                    let q_ = &*q.unwrap();

                    if !(key.pressed(KeyCode::LShift) || key.pressed(KeyCode::RShift)) {
                        robot_set_entity_server.set_all_links_without_focus();
                    }

                    robot_set_entity_server.set_link_with_focus(q_.robot_server_vector_idx, q_.robot_set_idx, q_.robot_link_idx);
                    // println!("{:?} has focus", robot_set_entity_server.get_links_with_focus_ref());
                }
            }
        }
    }

    if key.just_pressed(KeyCode::A) {
        robot_set_entity_server.set_all_links_without_focus();
    }

    /*
    let mut empty_picking_source = true;
    for pick_source in pick_source_query.iter() {
        let e = pick_source.intersect_list();
        if e.is_some() {
            empty_picking_source = false;
            break;
        }
    }

    if empty_picking_source && btn.just_pressed(MouseButton::Left) {
        robot_set_entity_server.set_all_has_focus_to_false();
        println!("set all has focus to false");
    }
    */
}