use bevy::prelude::{Commands, LightBundle, Transform, Vec3, Entity};

pub fn spawn_light(commands: &mut Commands, location: Vec3) -> Entity {
    let id = commands.spawn_bundle(LightBundle {
        light: Default::default(),
        transform: Transform::from_translation(location),
        global_transform: Default::default(),
    }).id();
    return id;
}