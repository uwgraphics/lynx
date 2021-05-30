use bevy::prelude::*;
use bevy::render::camera::PerspectiveProjection;
use bevy_mod_picking::PickingCameraBundle;

use crate::app_v2::app_actions::{
    camera_actions::camera_spawners::spawn_pan_orbit_perspective_camera,
    light_actions::light_spawners::spawn_light
};
use crate::app_v2::app_structs::{
    asset_structs::robot_set_asset_loader::*,
    camera_structs::pan_orbit_camera::*,
    robot_structs::robot_world_status_structs::*,
    gui_structs::egui_structs::*
};
use crate::app_v2::app_structs::game_engine_structs::game_engine_status_structs::FrameCount;
use crate::robot_modules::prelude::*;
use crate::utils::utils_vars::prelude::*;

pub fn insert_init_resources(mut commands: Commands, asset_server: Res<AssetServer>) {
    let mut lynx_vars_generic = LynxVarsGeneric::new_parallel_packaged_with_robot_world(None, vec!["ur5"], vec![None], None).expect("robot loading error");
    let robot_world = get_lynx_var_mut_ref_generic!(&mut lynx_vars_generic, RobotWorld, "robot_world").expect("error loading robot_world");
    let robot_set = robot_world.get_robot_set_mut_ref();

    commands.insert_resource(RobotSetAssetLoader::new(robot_set, &asset_server));
    commands.insert_resource(lynx_vars_generic);
    commands.insert_resource(FrameCount::default());
    commands.insert_resource(NumberOfRobotSetsSpawnedInScene::default());
    commands.insert_resource(EguiWindowContainer::new_empty());

    /*
    commands.insert_resource(RobotSetEntityAndInfoServer::new());
    commands.insert_resource(SpawnNewPhysicalRobot(true));
    commands.insert_resource(SpawnNewEnvironment(false));
    commands.insert_resource(CameraType::PanOrbitCamera);
    commands.insert_resource(CurrentMainGUIValues::new());
    commands.insert_resource(MultipleRobotLinkFocusWindowOpenManager::new_empty());
    commands.insert_resource(MultipleEnvironmentFocusWindowOpenManager::new_empty());
    commands.insert_resource(CollisionWindowVariables::new());
    commands.insert_resource(EnvironmentEntityAndInfoServer::new());
    commands.insert_resource(InstantContainer::new());
    commands.insert_resource(PathPlanningStartAndGoalStatePack::new());
    commands.insert_resource(GreenScreenOn(false));
    */
}

pub fn spawn_init_lights(mut commands: Commands) {
    spawn_light(&mut commands, Vec3::new(1.0, 1.0, 2.0));
    spawn_light(&mut commands, Vec3::new(-1.0, 1.0, -2.0));
    spawn_light(&mut commands, Vec3::new(4.0, 0.0, -1.0));
    spawn_light(&mut commands, Vec3::new(0.0, 7.0, 0.2));
}

pub fn spawn_init_camera(mut commands: Commands) {
    let location = Vec3::new(3.0, 1.0, 0.4);
    spawn_pan_orbit_perspective_camera(&mut commands, location);
}