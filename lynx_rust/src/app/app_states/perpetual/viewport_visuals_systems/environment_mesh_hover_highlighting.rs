use bevy::prelude::{Query, Transform, RenderPipelines, Mesh, Handle, StandardMaterial, Res, Input, KeyCode, ResMut, Assets, AssetServer, Visible, Children, EventReader};
use bevy_mod_picking::{PickingCamera, Hover, HoverEvent, SelectionEvent, PickingEvent};
use bevy::pbr::render_graph::PBR_PIPELINE_HANDLE;
use bevy::render::pipeline::RenderPipeline;
use crate::app::app_utils::asset_utils::lynx_material_type::{get_lynx_material_handle_id, LynxMaterialType};
use crate::app::app_states::res_comps::{RobotSetEntityAndInfoServer, RobotLinkInfoContainer, EnvObjectIdx, CurrentMainGUIValues};
use bevy::scene::Scene;
use crate::app::app_utils::robot_utils::robot_link_materials_utils::{change_material_of_robot_link, reset_material_of_robot_link};
use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;

pub fn environment_mesh_hover_highlighting(mut event: EventReader<PickingEvent>,
                                           mut env_entity_server: ResMut<EnvironmentEntityAndInfoServer>,
                                           mut query: Query<(&mut EnvObjectIdx)>,
                                           mut current_main_gui_values: ResMut<CurrentMainGUIValues>) {

    if current_main_gui_values.environment_selectable {
        for ev in event.iter() {
            let picking_event: &PickingEvent = &*ev;
            match picking_event {
                PickingEvent::Selection(s) => {}
                PickingEvent::Hover(h) => {
                    let entity = match h {
                        HoverEvent::JustEntered(e) => { e }
                        HoverEvent::JustLeft(e) => { e }
                    };
                    let q_ = query.get_mut(entity.clone());
                    let q = q_.as_ref();
                    if q.is_ok() {
                        let q = q.unwrap();
                        let curr_material = env_entity_server.get_curr_material(q.0).expect("error");
                        if curr_material <= LynxMaterialType::Hover {
                            match h {
                                HoverEvent::JustEntered(e) => {
                                    env_entity_server.change_env_object_material_data(q.0, LynxMaterialType::Hover);
                                }
                                HoverEvent::JustLeft(e) => {
                                    env_entity_server.reset_env_object_material_data(q.0);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/*
pub fn mesh_selection_highlighting(pick_source_query: Query<&PickingCamera>,
                                   mut query: Query<(&mut RobotLinkInfoContainer)>,
                                   mut event: EventReader<PickingEvent>,
                                   asset_server: Res<AssetServer>,
                                   mut materials: ResMut<Assets<StandardMaterial>>,
                                   key: Res<Input<KeyCode>>,
                                   mut robot_set_entity_server: ResMut<RobotSetEntityServer>,
                                   mut q1: Query<(&mut Transform)>,
                                   mut q2: Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {

    /*
    for pick_source in pick_source_query.iter() {
        let e = pick_source.intersect_top();
        if e.is_some() {
            let e_ = e.unwrap().0;
            let q = query.get_mut(e_);
            // println!("{:?}", q);
        }
    }

    for ev in event.iter() {
        // eprintln!("event {:?}", *ev);
        let picking_event: &PickingEvent = &*ev;
        match picking_event {
            PickingEvent::Selection(s) => {
                match s {
                    SelectionEvent::JustSelected(e) => {
                        let q = query.get_mut(e.clone());
                        println!("Just Selected: {:?}", q);
                    }
                    SelectionEvent::JustDeselected(e) => {
                        let q = query.get_mut(e.clone());
                        println!("Just Deselected: {:?}", q);
                    }
                }
            }
            PickingEvent::Hover(h) => {
                match h {
                    HoverEvent::JustEntered(e) => {
                        let q_ = query.get_mut(e.clone());
                        let q = q_.as_ref().unwrap();
                        // println!("Just Entered: {:?}", q);
                        change_material_of_robot_link(LynxMaterialType::Hover,
                                                      &asset_server,
                                                      &mut materials,
                                                      &mut robot_set_entity_server,
                                                      q.robot_server_vector_idx,
                                                      q.robot_set_idx,
                                                      q.robot_link_idx,
                                                      &mut q1,
                                                      &mut q2);
                    }
                    HoverEvent::JustLeft(e) => {
                        let q_ = query.get_mut(e.clone());
                        let q = q_.as_ref().unwrap();
                        // println!("Just Left: {:?}", q);
                        reset_material_of_robot_link(LynxMaterialType::Default,
                                                     &asset_server,
                                                     &mut materials,
                                                     &mut robot_set_entity_server,
                                                     q.robot_server_vector_idx,
                                                     q.robot_set_idx,
                                                     q.robot_link_idx,
                                                     &mut q1,
                                                     &mut q2);
                    }
                }
            }
        }
    }
    */

    /*
    let entity = robot_set_entity_server.get_physical_robot_set_entity_container_ref(0);
    if entity.is_ok() {
        let e = entity.as_ref().unwrap().get_visible_glb_scene_link_entities_ref()[0][5].clone();
        let u = e.unwrap();
        let mut res = query.get_mut(u);
        if res.is_ok() {
            let mut res_unwrap = res.as_mut().ok().unwrap();
            let c: &mut Children = &mut *res_unwrap.1;
            let e = c.get(0);
            if e.is_some() {
                let res2 = query2.get(e.unwrap().clone());
                println!("{:?}", res2);
            }
        }
    }

     */
    /*
    for pick_source in pick_source_query.iter() {
        let a = pick_source.intersect_list();
        if a.is_some() {
            let b = a.unwrap();
            let first_entity = &b[0].0;

            let mut res = query.get_mut(first_entity.clone()).unwrap();
            println!("{:?}", res);

            let mut r0: &mut Transform = &mut res.0;
            let mut r1: &mut Visible = &mut res.1;

            if key.pressed(KeyCode::LShift) {
                // let r = RenderPipelines::from_pipelines(vec![RenderPipeline::new(
                //    PBR_PIPELINE_HANDLE.typed(),
                // )]);
                // r1.pipelines = vec![RenderPipeline::new(PBR_PIPELINE_HANDLE.typed())];
                // r1.bindings = Default::default();
                println!("got here");
                *res.1 = Visible { is_visible: false, is_transparent: false }
            } else {
                // let r = RenderPipelines {
                //     pipelines: vec![],
                //    bindings: Default::default(),
                // };
                // r1.pipelines = vec![];
                *res.1 = Visible { is_visible: true, is_transparent: true }
            }

        }
    }

     */
}
*/