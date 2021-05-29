use bevy::prelude::{Commands, Assets, ResMut, Mesh, StandardMaterial};
use crate::app_v2::app_actions::viewport_visuals_actions::grid_spawners::spawn_grid_lines;


pub fn grid_management_system(mut commands: Commands,
                              mut meshes: ResMut<Assets<Mesh>>,
                              mut materials: ResMut<Assets<StandardMaterial>>) {
    spawn_grid_lines(&mut commands, &mut meshes, &mut materials);
}