use crate::app::app_states::perpetual::perpetual_res_comps::*;
use crate::robot_modules::prelude::{Robot, RobotWorld};
use crate::utils::utils_vars::lynx_vars_generic::LynxVarsGeneric;
use crate::app::app_states::app_states_enum::*;
use bevy::render::camera::PerspectiveProjection;
use bevy_mod_picking::*;
use bevy::prelude::{Plugin, AppBuilder, Commands, ResMut, Assets, StandardMaterial, IntoSystem, LightBundle, Vec3, PerspectiveCameraBundle, State, AssetServer, Res};
use bevy::transform::prelude::Transform;
use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;
use crate::app::app_utils::asset_utils::individual_robot_asset_loader::IndividualRobotAssetLoader;
use crate::app::app_utils::asset_utils::robot_set_asset_loader::RobotSetAssetLoader;

pub struct StartupPlugin;
impl Plugin for StartupPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_startup_system(lynx_startup.system());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn lynx_startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    _insert_init_resources(&mut commands, &asset_server);
    _spawn_init_lights(&mut commands);
    _spawn_init_camera(&mut commands);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

fn _insert_init_resources(commands: &mut Commands, asset_server: &Res<AssetServer>) {
    let mut lynx_vars_generic = LynxVarsGeneric::new_parallel_packaged_with_robot_world(None, vec!["ur5"], vec![None], None).expect("robot loading error");
    let robot_world = get_lynx_var_mut_ref_generic!(&mut lynx_vars_generic, RobotWorld, "robot_world").expect("error loading robot_world");
    let robot_set = robot_world.get_robot_set_mut_ref();

    commands.insert_resource(RobotSetAssetLoader::new(robot_set, asset_server));
    commands.insert_resource(lynx_vars_generic);
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
}

fn _spawn_init_lights(commands: &mut Commands) {
    commands.spawn_bundle(LightBundle {
        light: Default::default(),
        transform: Transform::from_translation(Vec3::new(1.0, 1.0, 2.0)),
        global_transform: Default::default()
    });
    commands.spawn_bundle(LightBundle {
        light: Default::default(),
        transform: Transform::from_translation(Vec3::new(-1.0, 1.0, -2.0)),
        global_transform: Default::default()
    });
    commands.spawn_bundle(LightBundle {
        light: Default::default(),
        transform: Transform::from_translation(Vec3::new(4.0, 0.0, -1.0)),
        global_transform: Default::default()
    });
    commands.spawn_bundle(LightBundle {
        light: Default::default(),
        transform: Transform::from_translation(Vec3::new(0.0, 7.0, 0.2)),
        global_transform: Default::default()
    });
}

fn _spawn_init_camera(commands: &mut Commands) {
    let translation = Vec3::new(3.0, 1.0, 0.4);
    let radius = translation.length();
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_translation(Vec3::new(3.0, 1.0, 0.4))
            .looking_at(Vec3::new(0.0,0.0,0.0), Vec3::new(0.0, 1.0, 0.0))
        ,
        perspective_projection: PerspectiveProjection {
            near: 0.001,
            ..Default::default()
        },
        ..Default::default()
    })
        .insert(PanOrbitCamera { radius, ..Default::default() })
        .insert_bundle(PickingCameraBundle::default());
}

////////////////////////////////////////////////////////////////////////////////////////////////////