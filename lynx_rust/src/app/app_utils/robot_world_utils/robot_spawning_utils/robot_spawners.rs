use crate::app::app_utils::robot_world_utils::robot_asset_utils::prelude::*;
use crate::app::app_utils::math_utils::transform_utils::*;
use bevy::ecs::prelude::{Res, ResMut, Commands};
use bevy::asset::{AssetServer, Handle, Assets};
use crate::robot_modules::robot::Robot;
use bevy::pbr::{PbrBundle};
use bevy::prelude::{Mesh, Color, Transform, StandardMaterial};
use bevy::math::{Vec3, Quat};

pub fn spawn_robot(robot: &Robot,
                   asset_server: &Res<AssetServer>,
                   commands: &mut Commands,
                   robot_mesh_asset_server: &mut ResMut<RobotMeshAssetServer>,
                   materials: &mut ResMut<Assets<StandardMaterial>>) {
    let individual_robot_asset_container = robot_mesh_asset_server
        .get_individual_robot_asset_container_ref(robot.get_robot_name_ref(), &asset_server)
        .expect("error getting individual robot asset container in spawn_robot");

    let a = individual_robot_asset_container.get_link_selectable_mesh_handle_ids_ref();
    let b = individual_robot_asset_container.get_link_wireframe_mesh_handle_ids_ref();
    let c = individual_robot_asset_container.get_link_visible_glb_scene_handle_ids_ref();

    let fk_res = robot.get_fk_module_ref().compute_fk_on_all_zeros_config().expect("error on fk in spawn_robot");
    let link_mesh_visual_offsets = robot.get_mesh_info_module_ref().get_link_mesh_visual_offsets();

    let l = fk_res.get_link_frames_ref().len();
    for i in 0..l {
        if a[i].is_some() {
            let mesh : Handle<Mesh> = asset_server.get_handle(a[i].unwrap());
            let t = convert_z_up_idq_to_y_up_bevy_transform_with_visual_mesh_offset(fk_res.get_link_frames_ref()[i].as_ref().unwrap(), link_mesh_visual_offsets[i].as_ref().unwrap());
            // let t = convert_z_up_idq_to_y_up_bevy_transform(fk_res.get_link_frames_ref()[i].as_ref().unwrap());
            commands.spawn_bundle(
             PbrBundle {
                 mesh: mesh,
                 material: materials.add(Color::rgb(0.0, 0.6, 0.9).into()),
                 transform: t,
                 ..Default::default()
             });
        }
    }
}