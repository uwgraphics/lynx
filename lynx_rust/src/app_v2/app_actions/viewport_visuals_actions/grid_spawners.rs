use crate::app_v2::app_actions::viewport_visuals_actions::line_spawners::*;
use bevy::prelude::{Commands, ResMut, Assets, Mesh, StandardMaterial, Vec3, Color, Query, Entity};
use crate::app_v2::app_type_enums::enums::LineType;


pub fn spawn_grid_lines(commands: &mut Commands,
                        meshes: &mut ResMut<Assets<Mesh>>,
                        materials: &mut ResMut<Assets<StandardMaterial>>) {
    let x_and_y_width = 3.0;
    let normal_width = 2.0;

    spawn_line_lynx_space(commands, meshes, materials, Vec3::new(0., 0., 0.), Vec3::new(10., 0., 0.), Color::rgb(1., 0., 0.), x_and_y_width, LineType::Grid);
    spawn_line_lynx_space(commands, meshes, materials, Vec3::new(0., 0., 0.), Vec3::new(-10., 0., 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);

    spawn_line_lynx_space(commands, meshes, materials, Vec3::new(0., 0., 0.), Vec3::new(0., 10., 0.), Color::rgb(0., 1., 0.), x_and_y_width, LineType::Grid);
    spawn_line_lynx_space(commands, meshes, materials, Vec3::new(0., 0., 0.), Vec3::new(0., -10., 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);

    for i in 1..10 {
        spawn_line_lynx_space(commands, meshes, materials, Vec3::new(-10.0, i as f32, 0.), Vec3::new(10., i as f32, 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
        spawn_line_lynx_space(commands, meshes, materials, Vec3::new(-10.0, -i as f32, 0.), Vec3::new(10., -i as f32, 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
    }

    for i in 1..10 {
        spawn_line_lynx_space(commands, meshes, materials, Vec3::new(i as f32, -10.0, 0.), Vec3::new(i as f32, 10.0, 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
        spawn_line_lynx_space(commands, meshes, materials, Vec3::new(-i as f32, -10.0, 0.), Vec3::new(-i as f32, 10.0, 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
    }
}

pub fn despawn_grid_lines(commands: &mut Commands, query: &Query<(Entity, &LineType)>) {
    despawn_lines_of_given_line_type(commands, &LineType::Grid, query);
}