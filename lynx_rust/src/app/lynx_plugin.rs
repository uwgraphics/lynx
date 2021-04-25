use bevy::prelude::*;
use bevy_stl;
use crate::utils::utils_vars::lynx_vars_generic::LynxVarsGeneric;
use crate::app::app_utils::robot_world_utils::prelude::*;
use crate::app::app_utils::robot_world_utils::prelude::robot_spawners::spawn_robot;
use crate::prelude::Robot;
use bevy::asset::Asset;

pub struct LynxPlugin;

impl Plugin for LynxPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_plugin(bevy_stl::StlPlugin);
        app.add_startup_system(lynx_startup.system());
        app.add_system(test2.system());
    }
}

fn lynx_startup(mut commands: Commands) {
    let mut lynx_vars_generic = LynxVarsGeneric::new_parallel_packaged_with_robot_world(None, vec!["ur5"], vec![None], None).expect("robot loading error");
    commands.insert_resource(lynx_vars_generic);
    commands.insert_resource(RobotMeshAssetServer::new_empty());
    commands.insert_resource(true);
}

fn test2(mut commands: Commands,
         asset_server: Res<AssetServer>,
         mut robot_mesh_asset_server: ResMut<RobotMeshAssetServer>,
         mut materials: ResMut<Assets<StandardMaterial>>,
         mut b: ResMut<bool>) {
    if *b {
        commands.spawn_bundle(LightBundle {
            light: Default::default(),
            transform: Transform::from_translation(Vec3::new(1.0, 1.0, 2.0)),
            global_transform: Default::default()
        });

        commands.spawn_bundle(LightBundle {
            light: Default::default(),
            transform: Transform::from_translation(Vec3::new(-1.0, 1.0, 2.0)),
            global_transform: Default::default()
        });

        commands.spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_translation(Vec3::new(3.0, 1.0, 0.4))
                .looking_at(Vec3::new(0.0,0.0,0.0), Vec3::new(0.0, 1.0, 0.0))
            ,
            ..Default::default()
        });

        println!("got here");
        let robot = Robot::new("baxter", None).expect("error");
        spawn_robot(&robot, &asset_server, &mut commands, &mut robot_mesh_asset_server, &mut materials);

        let robot = Robot::new("ur5", Some("sample_config")).expect("error");
        spawn_robot(&robot, &asset_server, &mut commands, &mut robot_mesh_asset_server, &mut materials);

        *b = false;
    }
}
