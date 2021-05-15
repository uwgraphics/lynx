use crate::app::app_utils::drawing_utils::line_drawing::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugLines;
use bevy::render::prelude::Color;

pub fn draw_grid_lines(mut commands: Commands,
                       mut meshes: ResMut<Assets<Mesh>>,
                       mut materials: ResMut<Assets<StandardMaterial>>) {

    let x_and_y_width = 3.0;
    let normal_width = 2.0;

    draw_line_lynx_space(&mut commands, &mut meshes, &mut materials, Vec3::new(0.,0.,0.), Vec3::new(10.,0.,0.), Color::rgb(1., 0., 0.), x_and_y_width, LineType::Grid);
    draw_line_lynx_space(&mut commands, &mut meshes, &mut materials, Vec3::new(0.,0.,0.), Vec3::new(-10.,0.,0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);

    draw_line_lynx_space(&mut commands, &mut meshes, &mut materials, Vec3::new(0.,0.,0.), Vec3::new(0.,10.,0.), Color::rgb(0., 1., 0.), x_and_y_width, LineType::Grid);
    draw_line_lynx_space(&mut commands, &mut meshes, &mut materials, Vec3::new(0.,0.,0.), Vec3::new(0.,-10.,0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);

    for i in 1..10 {
        draw_line_lynx_space(&mut commands, &mut meshes, &mut materials, Vec3::new(-10.0,i as f32,0.), Vec3::new(10.,i as f32,0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
        draw_line_lynx_space(&mut commands, &mut meshes, &mut materials, Vec3::new(-10.0,-i as f32,0.), Vec3::new(10.,-i as f32,0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
    }

    for i in 1..10 {
        draw_line_lynx_space(&mut commands, &mut meshes, &mut materials, Vec3::new(i as f32,-10.0, 0.), Vec3::new(i as f32,10.0, 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
        draw_line_lynx_space(&mut commands, &mut meshes, &mut materials, Vec3::new(-i as f32,-10.0, 0.), Vec3::new(-i as f32,10.0, 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
    }
}