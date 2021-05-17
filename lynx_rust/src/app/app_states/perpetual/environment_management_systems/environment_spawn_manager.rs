use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;
use crate::utils::utils_vars::prelude::*;
use crate::app::app_states::res_comps::*;
use crate::robot_modules::prelude::RobotWorld;
use crate::app::app_utils::environment_utils::environment_spawner::*;
use bevy::prelude::{ResMut, Commands, Assets, StandardMaterial, Res, AssetServer};

pub fn environment_spawn_manager_system(mut spawn_new_environment: ResMut<SpawnNewEnvironment>,
                                        mut commands: Commands,
                                        mut materials: ResMut<Assets<StandardMaterial>>,
                                        mut lynx_vars: ResMut<LynxVarsGeneric<'static>>,
                                        asset_server: Res<AssetServer>,
                                        mut environment_entity_server: ResMut<EnvironmentEntityAndInfoServer>) {
    if spawn_new_environment.0 {
        environment_entity_server.despawn_all(&mut commands);

        let robot_world = get_lynx_var_ref_generic!(&mut *lynx_vars, RobotWorld, "robot_world").expect("error loading robot_world from lynx_vars in environment_spawn_manager_system");
        let collision_environment_ = robot_world.get_collision_environment_option_ref();
        if collision_environment_.is_none() {
            spawn_new_environment.0 = false;
            return;
        }
        let collision_environment = collision_environment_.as_ref().unwrap();
        collision_environment.print_summary();

        spawn_environment(collision_environment, &mut commands, &asset_server, &mut materials, &mut environment_entity_server);

        // let robot_world_ = robot_world.clone();
        // set_or_add_lynx_var_generic!(&mut *lynx_vars, RobotWorld, "robot_world", robot_world_);

        spawn_new_environment.0 = false;
    }
}