use bevy::prelude::{Commands, ResMut, Query, Entity, With, Res, AssetServer, DespawnRecursiveExt, Assets, StandardMaterial};
use crate::app::app_states::perpetual::perpetual_res_comps::*;
use crate::app::app_utils::robot_utils::full_robot_set_spawners::*;
use crate::utils::utils_vars::prelude::*;
use crate::robot_modules::prelude::RobotWorld;
use bevy_egui::{EguiContext, egui};
use crate::app::app_utils::asset_utils::lynx_material_type::LynxMaterialType;
use crate::app::app_utils::asset_utils::robot_set_asset_loader::RobotSetAssetLoader;
use std::time::Instant;


pub fn robot_spawn_manager_system(mut spawn_new_physical_robot: ResMut<SpawnNewPhysicalRobot>,
                                  mut commands: Commands,
                                  mut materials: ResMut<Assets<StandardMaterial>>,
                                  mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                                  asset_server: Res<AssetServer>,
                                  mut robot_set_asset_loader: ResMut<RobotSetAssetLoader>,
                                  mut robot_set_entity_server: ResMut<RobotSetEntityAndInfoServer>,
                                  mut robot_set_joint_values_query: Query<(Entity, &mut RobotSetJointValues)>,
                                  mut instant_container: ResMut<InstantContainer>
                                  ) {
    if spawn_new_physical_robot.0 {
        robot_set_entity_server.despawn_all(&mut commands);

        let robot_world = get_lynx_var_ref_generic!(&mut *lynx_vars, RobotWorld, "robot_world").expect("error loading robot_world from lynx_vars in robot_spawn_manager_system");
        let robot_set = robot_world.get_robot_set_ref();
        *robot_set_asset_loader = RobotSetAssetLoader::new(robot_set, &asset_server);

        spawn_robot_set(robot_set, &mut commands, &asset_server, &mut materials, &mut robot_set_entity_server, &mut robot_set_asset_loader, RobotLinkSpawnType::Physical, None);

        // *physical_robot_set_joint_values = PhysicalRobotSetJointValues::new_from_lynx_vars(&mut *lynx_vars).expect("error loading PhysicalRobotSetJointValues");
        for r in robot_set_joint_values_query.iter_mut() {
            let e: Entity = r.0.clone();
            commands.entity(e).despawn_recursive();
        }
        commands.spawn().insert( RobotSetJointValues::new_from_lynx_vars(&mut *lynx_vars, RobotSetType::Physical).expect("error") );

        instant_container.robot_reset = Instant::now();

        spawn_new_physical_robot.0 = false;
    }
}