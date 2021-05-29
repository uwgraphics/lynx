use crate::app_v2::app_type_enums::enums::{LynxMaterialChangeType, RobotLinkSpawnType, LynxMaterialChangeOrReset};
use crate::app_v2::app_structs::{
    robot_structs::robot_link_spawn_structs::*,
    robot_structs::robot_link_spawn_structs::RobotLinkSpawnIdxInfo,
    material_structs::lynx_material_structs::LynxMaterialUser
};
use bevy::prelude::{Query, Entity, Visible, Handle, StandardMaterial, Transform, With, Without, ResMut, Assets, AssetServer, Res, Vec3};

pub fn change_link_material_in_robot_set(robot_set_idx_in_scene: usize,
                                         robot_idx_in_robot_set: usize,
                                         link_idx_in_robot: usize,
                                         lynx_material_change_type: LynxMaterialChangeType,
                                         asset_server: &Res<AssetServer>,
                                         materials: &mut ResMut<Assets<StandardMaterial>>,
                                         link_query: &Query<(Entity, &RobotLinkSpawnIdxInfo, &RobotLinkSpawnType), (Without<RobotLinkSpawnTypeSelectableInvisible>)>,
                                         link_visible_glb_query: &mut Query<(&mut Transform), (With<RobotLinkSpawnTypeVisibleGlb>)>,
                                         link_standard_material_query: &mut Query<(&mut LynxMaterialUser, &RobotLinkSpawnIdxInfo, &RobotLinkStandardMaterialInfo, &mut Visible, &mut Handle<StandardMaterial>), (With<RobotLinkSpawnTypeStandardMaterial>)>
) -> Result<(), String> {

    let mut has_changed = false;
    for (mut lynx_material_user_, robot_link_spawn_idx_info_, _, _, _) in link_standard_material_query.iter_mut() {
        let robot_link_spawn_idx_info: &RobotLinkSpawnIdxInfo = &robot_link_spawn_idx_info_;

        if robot_link_spawn_idx_info.robot_set_idx_in_scene == robot_set_idx_in_scene &&
            robot_link_spawn_idx_info.robot_idx_in_robot_set == robot_idx_in_robot_set &&
            robot_link_spawn_idx_info.link_idx_in_robot == link_idx_in_robot {

            let mut lynx_material_user: &mut LynxMaterialUser = &mut lynx_material_user_;
            has_changed = lynx_material_user.change_material(lynx_material_change_type.clone());
            break;
        }
    }

    if has_changed {
        for (entity_, robot_link_spawn_idx_info_, robot_link_spawn_type_) in link_query.iter() {
            let robot_link_spawn_idx_info: &RobotLinkSpawnIdxInfo = &robot_link_spawn_idx_info_;

            if robot_link_spawn_idx_info.robot_set_idx_in_scene == robot_set_idx_in_scene &&
                robot_link_spawn_idx_info.robot_idx_in_robot_set == robot_idx_in_robot_set &&
                robot_link_spawn_idx_info.link_idx_in_robot == link_idx_in_robot {

                let entity: &Entity = &entity_;
                let robot_link_spawn_type: &RobotLinkSpawnType = &robot_link_spawn_type_;

                match robot_link_spawn_type {
                    RobotLinkSpawnType::StandardMaterial => {
                        let query_res = link_standard_material_query.get_mut(entity.clone());
                        if query_res.is_ok() {
                            let (lynx_material_user_, _, robot_link_standard_material_info_, mut visible_, mut handle_standard_material_) = query_res.unwrap();

                            let lynx_material_user: &LynxMaterialUser = &lynx_material_user_;
                            let robot_link_standard_material_info: &RobotLinkStandardMaterialInfo = &robot_link_standard_material_info_;
                            let mut visible: &mut Visible = &mut visible_;
                            let mut handle_standard_material: &mut Handle<StandardMaterial> = &mut handle_standard_material_;

                            let handle_id = lynx_material_user.curr_material.get_material_handle_id(materials);
                            let h: Handle<StandardMaterial> = asset_server.get_handle(handle_id);
                            *handle_standard_material = h;

                            match lynx_material_change_type.change_or_reset() {
                                LynxMaterialChangeOrReset::Change => {
                                    if robot_link_standard_material_info.invisible_on_reset {
                                        visible.is_transparent = false;
                                        visible.is_visible = true;
                                    }
                                }
                                LynxMaterialChangeOrReset::Reset => {
                                    if robot_link_standard_material_info.invisible_on_reset {
                                        visible.is_transparent = true;
                                        visible.is_visible = false;
                                    }
                                }
                            }
                        }
                    }
                    RobotLinkSpawnType::VisibleGlb => {
                        let query_res = link_visible_glb_query.get_mut(entity.clone());
                        if query_res.is_ok() {
                            let mut transform: &mut Transform = &mut query_res.unwrap();

                            match lynx_material_change_type.change_or_reset() {
                                LynxMaterialChangeOrReset::Change => { transform.scale = Vec3::new(0.,0.,0.); }
                                LynxMaterialChangeOrReset::Reset => { transform.scale = Vec3::new(1.,1.,1.); }
                            }
                        }
                    }
                    _ => { }
                }
            }
        }
    }

    Ok(())
}