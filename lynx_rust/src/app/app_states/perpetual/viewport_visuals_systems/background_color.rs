use bevy::prelude::*;
use crate::app::app_states::res_comps::GreenScreenOn;


pub fn background_greenscreen_system(mut commands: Commands,
                                     keys: Res<Input<KeyCode>>,
                                     mut greenscreen_on: ResMut<GreenScreenOn>) {
    if keys.just_pressed(KeyCode::B) && (keys.pressed(KeyCode::LShift) || keys.pressed(KeyCode::RShift)) {
        if greenscreen_on.0 {
            commands.insert_resource(ClearColor(Color::rgb(0.4, 0.4, 0.4)));
            greenscreen_on.0 = false;
        } else {
            commands.insert_resource(ClearColor(Color::rgb(0.0, 0.5, 0.5)));
            greenscreen_on.0 = true;
        }
    }
}