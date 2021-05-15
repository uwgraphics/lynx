use bevy::prelude::{Query, ResMut, MouseButton, Input, Res, KeyCode};
use bevy_mod_picking::PickingCamera;
use crate::app::app_states::perpetual::environment_management_systems::environment_entity_and_info_server::EnvironmentEntityAndInfoServer;
use crate::app::app_states::res_comps::{EnvObjectIdx, CurrentMainGUIValues};


pub fn environment_focus_selector(pick_source_query: Query<&PickingCamera>,
                                  mut env_entity_server: ResMut<EnvironmentEntityAndInfoServer>,
                                  mut query: Query<(&mut EnvObjectIdx)>,
                                  btn: Res<Input<MouseButton>>,
                                  key: Res<Input<KeyCode>>,
                                  mut current_main_gui_values: ResMut<CurrentMainGUIValues>) {
    if current_main_gui_values.environment_selectable {
        if btn.just_pressed(MouseButton::Left) {
            for pick_source in pick_source_query.iter() {
                let e = pick_source.intersect_top();
                if e.is_some() {
                    let e_ = e.unwrap().0;
                    let q = query.get_mut(e_);
                    if q.is_ok() {
                        let q_ = &*q.unwrap();

                        if !(key.pressed(KeyCode::LShift) || key.pressed(KeyCode::RShift)) {
                            env_entity_server.set_all_env_objects_without_focus();
                        }

                        env_entity_server.set_env_object_with_focus(q_.0);
                    }
                }
            }
        }

        if key.just_pressed(KeyCode::A) {
            env_entity_server.set_all_env_objects_without_focus();
        }
    }
}