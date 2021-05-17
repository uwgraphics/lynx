use crate::robot_modules::{robot_set::RobotSet, robot::Robot};
use crate::app::app_utils::math_utils::transform_utils::*;
use crate::app::app_states::res_comps::*;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use crate::app::app_utils::asset_utils::{lynx_material_type::*, individual_robot_asset_loader::IndividualRobotAssetLoader};
use nalgebra::{Vector3};
use bevy::prelude::{Res, AssetServer, Commands, ResMut, Entity, Handle, Mesh, StandardMaterial, PbrBundle, RenderPipelines, Scene, GlobalTransform, BuildChildren, SpawnSceneAsChildCommands, Assets, Vec3, Draw, Visible};
use bevy::asset::HandleId;
use bevy_mod_picking::PickableBundle;
use bevy_egui::egui::Color32;
use bevy::render::pipeline::RenderPipeline;
use bevy::pbr::render_graph::PBR_PIPELINE_HANDLE;
use crate::app::app_utils::asset_utils::robot_set_asset_loader::RobotSetAssetLoader;

pub fn spawn_robot_set(robot_set: &RobotSet,
                       commands: &mut Commands,
                       asset_server: &Res<AssetServer>,
                       materials: &mut ResMut<Assets<StandardMaterial>>,
                       robot_set_entity_server: &mut ResMut<RobotSetEntityAndInfoServer>,
                       robot_set_asset_loader: &mut ResMut<RobotSetAssetLoader>,
                       robot_link_spawn_type: RobotLinkSpawnType,
                       base_link_material: Option<LynxMaterialType>) {
    let robots = robot_set.get_robots_ref();
    let mut robot_set_entity_container = IndividualRobotSetEntityAndInfoContainer::new(robot_set.get_total_num_dofs());
    let server_idx = robot_set_entity_server.get_num_individual_robot_set_entity_and_info_containers();
    let l = robots.len();
    for i in 0..l {
        let individual_robot_asset_loader = &robot_set_asset_loader.get_robot_asset_loaders_ref()[i];
        let output = _spawn_robot(&robots[i], commands, asset_server, materials, individual_robot_asset_loader, i, server_idx, robot_link_spawn_type.clone(), base_link_material.clone());
        robot_set_entity_container.push_all_link_entities_from_one_robot(
            output.visible_glb_scene_link_entities,
            output.standard_material_link_entities,
            output.invisible_material_link_entities,
            output.link_materials);
    }

    robot_set_entity_server.add_individual_robot_set_entity_and_info_container(robot_set_entity_container);
}

/*
pub fn spawn_visualization_robot_set(robot_set: &RobotSet,
                                     commands: &mut Commands,
                                     asset_server: &Res<AssetServer>,
                                     materials: &mut ResMut<Assets<StandardMaterial>>,
                                     robot_set_entity_server: &mut ResMut<RobotSetEntityServer>,
                                     lynx_material_type: LynxMaterialType) {
    let robots = robot_set.get_robots_ref();
    let mut visualization_robot_set_entity_container = VisualizationRobotSetEntityContainer::new_empty();
    let server_idx = robot_set_entity_server.get_num_visualization_robot_set_entity_containers();
    let l = robots.len();
    for i in 0..l {
        let output = _spawn_visualization_robot(&robots[i], commands, asset_server, materials, i, server_idx, lynx_material_type);
        visualization_robot_set_entity_container.push_all_link_entities_from_one_robot(output.selectable_standard_link_entities);
    }

    robot_set_entity_server.add_visualization_robot_set_entity_container(visualization_robot_set_entity_container);
}

*/
////////////////////////////////////////////////////////////////////////////////////////////////////

struct SpawnRobotOutput {
    pub visible_glb_scene_link_entities: Vec<Option<Entity>>,
    pub standard_material_link_entities: Vec<Option<Entity>>,
    pub invisible_material_link_entities: Vec<Option<Entity>>,
    pub link_materials: Vec<Option<LynxMaterialType>>
}
fn _spawn_robot(robot: &Robot,
                commands: &mut Commands,
                asset_server: &Res<AssetServer>,
                materials: &mut ResMut<Assets<StandardMaterial>>,
                individual_robot_asset_loader: &IndividualRobotAssetLoader,
                robot_idx_in_set: usize,
                robot_server_idx: usize,
                robot_link_spawn_type: RobotLinkSpawnType,
                base_link_material: Option<LynxMaterialType>) -> SpawnRobotOutput {
    let mut visible_glb_scene_link_entities = Vec::new();
    let mut standard_material_link_entities = Vec::new();
    let mut invisible_material_link_entities = Vec::new();
    let mut link_materials = Vec::new();

    // let individual_robot_asset_container = IndividualRobotAssetLoader::new(robot.get_robot_name_ref(), asset_server);

    let a = individual_robot_asset_loader.get_link_visible_glb_scene_handle_ids_ref();
    let b = individual_robot_asset_loader.get_link_standard_material_mesh_handle_ids_ref();
    let c = individual_robot_asset_loader.get_link_lower_poly_mesh_handle_ids_ref();

    let fk_res = robot.get_fk_module_ref().compute_fk_on_all_zeros_config().expect("error on fk in spawn_robot");
    // let link_mesh_visual_offsets = robot.get_mesh_info_module_ref().get_link_mesh_visual_offsets();
    // let link_mesh_visual_scalings = robot.get_mesh_info_module_ref().get_link_mesh_visual_scalings();
    // let link_urdf_visual_offsets = robot.get_mesh_info_module_ref().get_link_urdf_visual_offsets();

    let links = &robot.get_configuration_module_ref().robot_model_module.links;
    let link_frames = fk_res.get_link_frames_ref();
    let l = link_frames.len();
    for i in 0..l {
        if link_frames[i].is_some() && !links[i].is_mobile_base_link {
            let link_frame = link_frames[i].as_ref().unwrap();
            // let link_mesh_visual_offset = link_mesh_visual_offsets[i].as_ref().expect("error unwrapping link_mesh_visual_offset");
            // let link_mesh_visual_scaling = link_mesh_visual_scalings[i].expect("error unwrapping link_mesh_visual_scaling");
            let link_mesh_visual_offset = &links[i].urdf_link.link_visual_mesh_offset.as_ref().expect("error unwrapping link_mesh_visual_offset");
            let link_mesh_visual_scaling = &links[i].urdf_link.link_visual_mesh_scaling.as_ref().expect("error unwrapping link_mesh_visual_offset");
            // let link_urdf_visual_offset = link_urdf_visual_offsets[i].as_ref().expect("error unwrapping link_urdf_visual_offset");
            let link_urdf_visual_offset = &links[i].urdf_link.link_visual_urdf_offset.as_ref().expect("error unwrapping link_urdf_visual_offset");
            let visible_glb_scene_handle_id = &a[i];
            let standard_material_mesh_handle_id = &b[i];
            let lower_poly_mesh_handle_id = &c[i];
            let spawn_link_output = match robot_link_spawn_type {
                RobotLinkSpawnType::Physical => {
                    _spawn_physical_link(commands,
                                         asset_server,
                                         materials,
                                         link_frame,
                                         link_mesh_visual_offset,
                                         &link_mesh_visual_scaling,
                                         link_urdf_visual_offset,
                                         visible_glb_scene_handle_id,
                                         standard_material_mesh_handle_id,
                                         lower_poly_mesh_handle_id,
                                         robot.get_robot_name_ref(),
                                         i,
                                         robot_idx_in_set,
                                         robot_server_idx,
                                         base_link_material.clone())
                }
                RobotLinkSpawnType::Visualization => {
                    _spawn_visualization_link(commands,
                                              asset_server,
                                              materials,
                                              link_frame,
                                              link_mesh_visual_offset,
                                              &link_mesh_visual_scaling,
                                              standard_material_mesh_handle_id,
                                              lower_poly_mesh_handle_id,
                                              robot.get_robot_name_ref(),
                                              i,
                                              robot_idx_in_set,
                                              robot_server_idx,
                                              base_link_material.clone())
                }
            };

            visible_glb_scene_link_entities.push(spawn_link_output.visible_glb_scene_link_entity);
            standard_material_link_entities.push(spawn_link_output.standard_material_link_entity);
            invisible_material_link_entities.push(spawn_link_output.invisible_material_link_entity);
            link_materials.push(spawn_link_output.link_material);
        } else {
            visible_glb_scene_link_entities.push(None);
            standard_material_link_entities.push(None);
            invisible_material_link_entities.push(None);
            link_materials.push(None);
        }
    }

    return SpawnRobotOutput { visible_glb_scene_link_entities, standard_material_link_entities, invisible_material_link_entities, link_materials };
}

/*
struct SpawnVisualizationRobotOutput { pub selectable_standard_link_entities: Vec<Option<Entity>> }
fn _spawn_visualization_robot(robot: &Robot,
                              commands: &mut Commands,
                              asset_server: &Res<AssetServer>,
                              materials: &mut ResMut<Assets<StandardMaterial>>,
                              robot_idx_in_set: usize,
                              robot_server_idx: usize,
                              lynx_material_type: LynxMaterialType) -> SpawnVisualizationRobotOutput {
    let mut selectable_standard_link_entities = Vec::new();

    let individual_robot_asset_container = IndividualRobotAssetContainer::new(robot.get_robot_name_ref(), asset_server);

    let a = individual_robot_asset_container.get_link_visible_glb_scene_handle_ids_ref();
    let b = individual_robot_asset_container.get_link_standard_material_mesh_handle_ids_ref();

    let fk_res = robot.get_fk_module_ref().compute_fk_on_all_zeros_config().expect("error on fk in spawn_robot");
    let link_mesh_visual_offsets = robot.get_mesh_info_module_ref().get_link_mesh_visual_offsets();
    let link_urdf_visual_offsets = robot.get_mesh_info_module_ref().get_link_urdf_visual_offsets();

    let links = &robot.get_configuration_module_ref().robot_model_module.links;
    let link_frames = fk_res.get_link_frames_ref();
    let l = link_frames.len();
    for i in 0..l {
        if link_frames[i].is_some() && !links[i].is_mobile_base_link {
            let link_frame = link_frames[i].as_ref().unwrap();
            let link_mesh_visual_offset = link_mesh_visual_offsets[i].as_ref().expect("error unwrapping link_mesh_visual_offset");
            let link_urdf_visual_offset = link_urdf_visual_offsets[i].as_ref().expect("error unwrapping link_urdf_visual_offset");
            let standard_material_mesh_handle_id = &b[i];
            let spawn_visualization_link_output = _spawn_visualization_link(commands,
                                                                            asset_server,
                                                                            materials,
                                                                            link_frame,
                                                                            link_mesh_visual_offset,
                                                                            standard_material_mesh_handle_id,
                                                                            robot.get_robot_name_ref(),
                                                                            i,
                                                                            robot_idx_in_set,
                                                                            robot_server_idx,
                                                                            lynx_material_type);
            selectable_standard_link_entities.push(spawn_visualization_link_output.selectable_standard_link_entity);
        } else {
            selectable_standard_link_entities.push(None);
        }
    }

    return SpawnVisualizationRobotOutput { selectable_standard_link_entities };
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

struct SpawnLinkOutput {
    pub visible_glb_scene_link_entity: Option<Entity>,
    pub standard_material_link_entity: Option<Entity>,
    pub invisible_material_link_entity: Option<Entity>,
    pub link_material: Option<LynxMaterialType>
}

fn _spawn_physical_link(commands: &mut Commands,
                        asset_server: &Res<AssetServer>,
                        materials: &mut ResMut<Assets<StandardMaterial>>,
                        link_frame: &ImplicitDualQuaternion,
                        link_mesh_visual_offset: &ImplicitDualQuaternion,
                        link_mesh_visual_scaling: &Vector3<f64>,
                        link_urdf_visual_offset: &ImplicitDualQuaternion,
                        visible_glb_scene_handle_id: &Option<HandleId>,
                        standard_material_mesh_handle_id: &Option<HandleId>,
                        lower_poly_mesh_handle_id: &Option<HandleId>,
                        robot_name: &String,
                        link_idx: usize,
                        robot_idx_in_set: usize,
                        robot_server_idx: usize,
                        base_link_material: Option<LynxMaterialType>) -> SpawnLinkOutput {
    let mut out = SpawnLinkOutput {
        visible_glb_scene_link_entity: None,
        standard_material_link_entity: None,
        invisible_material_link_entity: None,
        link_material: None
    };

    if standard_material_mesh_handle_id.is_some() {
        if base_link_material.is_some() { out.link_material = base_link_material.clone() }
        else { out.link_material = Some(LynxMaterialType::Default); }
        let id = _spawn_standard_material_link(commands,
                                               asset_server,
                                               materials,
                                               link_frame,
                                               link_mesh_visual_offset,
                                               link_mesh_visual_scaling,
                                               standard_material_mesh_handle_id.as_ref().unwrap(),
                                               robot_name,
                                               link_idx,
                                               robot_idx_in_set,
                                               robot_server_idx,
                                               RobotLinkSpawnType::Physical,
                                               visible_glb_scene_handle_id.is_some(),
                                               out.link_material.as_ref().unwrap().clone());
        out.standard_material_link_entity = Some(id);

        let id = _spawn_selectable_invisible_link(commands,
                                                  asset_server,
                                                  materials,
                                                  link_frame,
                                                  link_mesh_visual_offset,
                                                  link_mesh_visual_scaling,
                                                  lower_poly_mesh_handle_id.as_ref().unwrap(),
                                                  robot_name,
                                                  link_idx,
                                                  robot_idx_in_set,
                                                  robot_server_idx,
                                                  RobotLinkSpawnType::Physical);
        out.invisible_material_link_entity = Some(id);
    }

    if visible_glb_scene_handle_id.is_some() {
        let id = _spawn_visible_glb_scene_link(commands,
                                               asset_server,
                                               link_frame,
                                               link_urdf_visual_offset,
                                               visible_glb_scene_handle_id.as_ref().unwrap(),
                                               robot_name,
                                               link_idx,
                                               robot_idx_in_set,
                                               robot_server_idx);

        out.visible_glb_scene_link_entity = Some(id);
    }

    return out;
}

fn _spawn_visualization_link(commands: &mut Commands,
                             asset_server: &Res<AssetServer>,
                             materials: &mut ResMut<Assets<StandardMaterial>>,
                             link_frame: &ImplicitDualQuaternion,
                             link_mesh_visual_offset: &ImplicitDualQuaternion,
                             link_mesh_visual_scaling: &Vector3<f64>,
                             standard_material_mesh_handle_id: &Option<HandleId>,
                             lower_poly_mesh_handle_id: &Option<HandleId>,
                             robot_name: &String,
                             link_idx: usize,
                             robot_idx_in_set: usize,
                             robot_server_idx: usize,
                             base_link_material: Option<LynxMaterialType>) -> SpawnLinkOutput {
    let mut out = SpawnLinkOutput {
        visible_glb_scene_link_entity: None,
        standard_material_link_entity: None,
        invisible_material_link_entity: None,
        link_material: None
    };

    if standard_material_mesh_handle_id.is_some() {
        if base_link_material.is_some() { out.link_material = base_link_material }
        else { out.link_material = Some(LynxMaterialType::VisualizationDefault); }
        let id = _spawn_standard_material_link(commands,
                                               asset_server,
                                               materials,
                                               link_frame,
                                               link_mesh_visual_offset,
                                               link_mesh_visual_scaling,
                                               standard_material_mesh_handle_id.as_ref().unwrap(),
                                               robot_name,
                                               link_idx,
                                               robot_idx_in_set,
                                               robot_server_idx,
                                               RobotLinkSpawnType::Visualization,
                                               false,
                                               out.link_material.as_ref().unwrap().clone() );
        out.standard_material_link_entity = Some(id);

        let id = _spawn_selectable_invisible_link(commands,
                                                  asset_server,
                                                  materials,
                                                  link_frame,
                                                  link_mesh_visual_offset,
                                                  link_mesh_visual_scaling,
                                                  lower_poly_mesh_handle_id.as_ref().unwrap(),
                                                  robot_name,
                                                  link_idx,
                                                  robot_idx_in_set,
                                                  robot_server_idx,
                                                  RobotLinkSpawnType::Visualization);
        out.invisible_material_link_entity = Some(id);
    }

    return out;
}

/*
struct SpawnVisualizationLinkOutput { pub selectable_standard_link_entity: Option<Entity> }
fn _spawn_visualization_link(commands: &mut Commands,
                             asset_server: &Res<AssetServer>,
                             materials: &mut ResMut<Assets<StandardMaterial>>,
                             link_frame: &ImplicitDualQuaternion,
                             link_mesh_visual_offset: &ImplicitDualQuaternion,
                             standard_material_mesh_handle_id: &Option<HandleId>,
                             robot_name: &String,
                             link_idx: usize,
                             robot_idx_in_set: usize,
                             robot_server_idx: usize,
                             lynx_material_type: LynxMaterialType) -> SpawnVisualizationLinkOutput {
    let mut out = SpawnVisualizationLinkOutput { selectable_standard_link_entity: None };

    let id = _spawn_standard_material_link(commands,
                                                      asset_server,
                                                      materials,
                                                      link_frame,
                                                      link_mesh_visual_offset,
                                                      standard_material_mesh_handle_id.as_ref().unwrap(),
                                                      robot_name,
                                                      link_idx,
                                                      robot_idx_in_set,
                                                      robot_server_idx,
                                                      RobotLinkSpawnType::Visualization,
                                                      false,
                                                      lynx_material_type);
    out.selectable_standard_link_entity = Some(id);

    return out;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

fn _spawn_standard_material_link(commands: &mut Commands,
                                 asset_server: &Res<AssetServer>,
                                 materials: &mut ResMut<Assets<StandardMaterial>>,
                                 link_frame: &ImplicitDualQuaternion,
                                 link_mesh_visual_offset: &ImplicitDualQuaternion,
                                 link_mesh_visual_scaling: &Vector3<f64>,
                                 standard_material_mesh_handle_id: &HandleId,
                                 robot_name: &String,
                                 link_idx: usize,
                                 robot_idx_in_set: usize,
                                 robot_server_idx: usize,
                                 robot_link_spawn_type: RobotLinkSpawnType,
                                 invisible: bool,
                                 lynx_material_type: LynxMaterialType) -> Entity {
    let mesh: Handle<Mesh> = asset_server.get_handle(*standard_material_mesh_handle_id);
    /*
    let mut transform = convert_z_up_idq_to_y_up_bevy_transform_with_visual_and_urdf_mesh_offset(link_frame,
                                                                                                 link_mesh_visual_offset,
                                                                                                 link_urdf_visual_offset);
     */

    // let mut transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame,
    //                                                                        link_mesh_visual_offset);

    // transform.scale = Vec3::new(link_mesh_visual_scaling[0] as f32, link_mesh_visual_scaling[1] as f32, link_mesh_visual_scaling[2] as f32);

    let mut transform = convert_z_up_idq_to_y_up_bevy_transform(link_frame);

    let material_handle_id = get_lynx_material_handle_id(lynx_material_type.clone(), materials);
    let material: Handle<StandardMaterial> = asset_server.get_handle(material_handle_id);

    let pbr_bundle = if invisible {
        PbrBundle {
            mesh: mesh.clone(),
            material: material.clone(),
            transform,
            visible: Visible { is_visible: false, is_transparent: true },
            ..Default::default()
        }
    } else {
        PbrBundle {
            mesh: mesh.clone(),
            material: material.clone(),
            transform,
            ..Default::default()
        }
    };

    let robot_link_info_container = RobotLinkInfoContainer {
        robot_name: robot_name.clone(),
        robot_link_spawn_type: robot_link_spawn_type.clone(),
        robot_link_mesh_type: RobotLinkMeshType::StandardMaterial,
        robot_server_vector_idx: robot_server_idx,
        robot_set_idx: robot_idx_in_set,
        robot_link_idx: link_idx,
    };

    let id = commands.spawn_bundle(pbr_bundle)
        .insert(robot_link_info_container)
        .id();

    return id;
}

fn _spawn_selectable_invisible_link(commands: &mut Commands,
                                    asset_server: &Res<AssetServer>,
                                    materials: &mut ResMut<Assets<StandardMaterial>>,
                                    link_frame: &ImplicitDualQuaternion,
                                    link_mesh_visual_offset: &ImplicitDualQuaternion,
                                    link_mesh_visual_scaling: &Vector3<f64>,
                                    lower_poly_mesh_handle_id: &HandleId,
                                    robot_name: &String,
                                    link_idx: usize,
                                    robot_idx_in_set: usize,
                                    robot_server_idx: usize,
                                    robot_link_spawn_type: RobotLinkSpawnType) -> Entity {
    let mesh: Handle<Mesh> = asset_server.get_handle(*lower_poly_mesh_handle_id);
    /*
    let mut transform = convert_z_up_idq_to_y_up_bevy_transform_with_visual_and_urdf_mesh_offset(link_frame,
                                                                                                 link_mesh_visual_offset,
                                                                                                 link_urdf_visual_offset);
     */

    // let mut transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame,
    //                                                                        link_mesh_visual_offset);

    let mut transform = convert_z_up_idq_to_y_up_bevy_transform(link_frame);

    // transform.scale = Vec3::new(link_mesh_visual_scaling[0] as f32, link_mesh_visual_scaling[1] as f32, link_mesh_visual_scaling[2] as f32);

    let material_handle_id = get_lynx_material_handle_id(LynxMaterialType::Default, materials);
    let material: Handle<StandardMaterial> = asset_server.get_handle(material_handle_id);

    let pbr_bundle = PbrBundle {
        mesh: mesh.clone(),
        material: material.clone(),
        transform,
        render_pipelines: RenderPipelines { pipelines: vec![], bindings: Default::default() },
        ..Default::default()
    };

    let robot_link_info_container = RobotLinkInfoContainer {
        robot_name: robot_name.clone(),
        robot_link_spawn_type: robot_link_spawn_type.clone(),
        robot_link_mesh_type: RobotLinkMeshType::InvisibleMaterial,
        robot_server_vector_idx: robot_server_idx,
        robot_set_idx: robot_idx_in_set,
        robot_link_idx: link_idx,
    };

    let id = commands.spawn_bundle(pbr_bundle)
        .insert(robot_link_info_container)
        .insert_bundle(PickableBundle::default()).id();

    return id;
}

fn _spawn_visible_glb_scene_link(commands: &mut Commands,
                                 asset_server: &Res<AssetServer>,
                                 link_frame: &ImplicitDualQuaternion,
                                 link_urdf_visual_offset: &ImplicitDualQuaternion,
                                 visible_glb_scene_handle_id: &HandleId,
                                 robot_name: &String,
                                 link_idx: usize,
                                 robot_idx_in_set: usize,
                                 robot_server_idx: usize) -> Entity {
    let scene: Handle<Scene> = asset_server.get_handle(*visible_glb_scene_handle_id);
    let mut transform = convert_z_up_idq_to_y_up_bevy_transform_with_offset(link_frame,
                                                                            link_urdf_visual_offset);

    let robot_link_info_container = RobotLinkInfoContainer {
        robot_name: robot_name.clone(),
        robot_link_spawn_type: RobotLinkSpawnType::Physical,
        robot_link_mesh_type: RobotLinkMeshType::VislbleGlb,
        robot_server_vector_idx: robot_server_idx,
        robot_set_idx: robot_idx_in_set,
        robot_link_idx: link_idx,
    };

    let id = commands
        .spawn_bundle((
            transform,
            GlobalTransform::identity(),
        )).with_children(|parent| {
        parent.spawn_scene(scene);
    })
        .insert(robot_link_info_container)
        .id();

    return id;
}
