use bevy::prelude::*;
use crate::app::lynx_plugin::LynxPlugin;


pub fn app_main() {
    App::build()
        .insert_resource(Msaa{samples: 4})
        .insert_resource(WindowDescriptor {
            title: "LYNX".to_string(),
            ..Default::default()
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(LynxPlugin)
        .run();
}
