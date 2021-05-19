use crate::app::app_utils::drawing_utils::line_drawing::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugLines;
use bevy::render::prelude::Color;
use crate::app::app_states::res_comps::CurrentMainGUIValues;

pub fn draw_grid_lines(mut commands: Commands,
                       mut meshes: ResMut<Assets<Mesh>>,
                       mut materials: ResMut<Assets<StandardMaterial>>) {
    draw_grid_lines_generic(&mut commands, &mut meshes, &mut materials);
}

pub fn draw_grid_lines_generic(commands: &mut Commands,
                               meshes: &mut ResMut<Assets<Mesh>>,
                               materials: &mut ResMut<Assets<StandardMaterial>>) {
    let x_and_y_width = 3.0;
    let normal_width = 2.0;

    draw_line_lynx_space(commands, meshes, materials, Vec3::new(0.,0.,0.), Vec3::new(10.,0.,0.), Color::rgb(1., 0., 0.), x_and_y_width, LineType::Grid);
    draw_line_lynx_space(commands, meshes, materials, Vec3::new(0.,0.,0.), Vec3::new(-10.,0.,0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);

    draw_line_lynx_space(commands, meshes, materials, Vec3::new(0.,0.,0.), Vec3::new(0.,10.,0.), Color::rgb(0., 1., 0.), x_and_y_width, LineType::Grid);
    draw_line_lynx_space(commands, meshes, materials, Vec3::new(0.,0.,0.), Vec3::new(0.,-10.,0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);

    for i in 1..10 {
        draw_line_lynx_space(commands, meshes, materials, Vec3::new(-10.0,i as f32,0.), Vec3::new(10.,i as f32,0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
        draw_line_lynx_space(commands, meshes, materials, Vec3::new(-10.0,-i as f32,0.), Vec3::new(10.,-i as f32,0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
    }

    for i in 1..10 {
        draw_line_lynx_space(commands, meshes, materials, Vec3::new(i as f32,-10.0, 0.), Vec3::new(i as f32,10.0, 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
        draw_line_lynx_space(commands, meshes, materials, Vec3::new(-i as f32,-10.0, 0.), Vec3::new(-i as f32,10.0, 0.), Color::rgb(0.6, 0.6, 0.6), normal_width, LineType::Grid);
    }
}

pub fn grid_lines_manager_system(mut commands: Commands,
                                 mut meshes: ResMut<Assets<Mesh>>,
                                 mut materials: ResMut<Assets<StandardMaterial>>,
                                 mut current_main_gui_values: ResMut<CurrentMainGUIValues>,
                                 lines_query: Query<(Entity, &LineType)>) {
    if current_main_gui_values.just_clicked_grid_visible_toggle {
        if current_main_gui_values.grid_visible {
            draw_grid_lines_generic(&mut commands, &mut meshes, &mut materials);
        } else {
            for l in lines_query.iter() {
                let entity = l.0;
                let line_type: &LineType = l.1;
                match line_type {
                    LineType::Grid => {
                        commands.entity(entity).despawn_recursive();
                    }
                    _ => { }
                }
            }
        }
        current_main_gui_values.just_clicked_grid_visible_toggle = false;
    }
}

