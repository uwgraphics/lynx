use bevy::prelude::*;
use crate::app::app_states::res_comps::RobotSetEntityAndInfoServer;
use crate::app::app_utils::robot_utils::robot_link_materials_utils::*;
use crate::utils::utils_vars::prelude::LynxVarsGeneric;
use crate::robot_modules::prelude::*;

pub fn robot_hidden_manager_system(mut commands: Commands,
                                   mut robot_set_entity_server: ResMut<RobotSetEntityAndInfoServer>,
                                   lynx_vars: Res<LynxVarsGeneric<'static>>,
                                   mut q1: Query<(&mut Transform)>,
                                   mut q2: Query<(&mut Visible, &mut Handle<StandardMaterial>)>) {

    let robot_world = get_lynx_var_ref_generic!(& *lynx_vars, RobotWorld, "robot_world").expect("error getting RobotWorld");
    let robot_set = robot_world.get_robot_set_ref();
    let num_robots = robot_set.get_num_robots();

    let recently_hidden = robot_set_entity_server.get_robots_that_were_just_hidden_idxs().clone();

    for h in recently_hidden {
        for i in 0..num_robots {
            let links_ref = &robot_set.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.links;
            let num_links = links_ref.len();
            for j in 0..num_links {
                hide_robot_link(&mut commands, &mut robot_set_entity_server, h, i, j, &mut q1, &mut q2);
                robot_set_entity_server.set_link_without_focus(h, i,j);
            }
        }
    }
    robot_set_entity_server.wipe_robots_that_were_just_hidden();

    let recently_unhidden = robot_set_entity_server.get_robots_that_were_just_unhidden_idxs().clone();

    for h in recently_unhidden {
        for i in 0..num_robots {
            let links_ref = &robot_set.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.links;
            let num_links = links_ref.len();
            for j in 0..num_links {
                unhide_robot_link(&mut commands, &mut robot_set_entity_server, h, i, j, &mut q1, &mut q2);
            }
        }
    }
    robot_set_entity_server.wipe_robots_that_were_just_unhidden();

}