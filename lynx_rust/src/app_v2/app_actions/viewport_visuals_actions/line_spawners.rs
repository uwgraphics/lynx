use bevy::prelude::{Commands, ResMut, Assets, Mesh, Color, shape, Transform, Quat, Vec3, PbrBundle, StandardMaterial, Mat3, Entity, Query};
use crate::app_v2::app_type_enums::enums::*;

pub fn spawn_line_bevy_space(commands: &mut Commands,
                             meshes: &mut ResMut<Assets<Mesh>>,
                             materials: &mut ResMut<Assets<StandardMaterial>>,
                             start_point: Vec3,
                             end_point: Vec3,
                             color: Color,
                             width_in_mm: f32,
                             line_type: LineType) -> Entity {
    /* by bevy space, I mean y up */

    let length = (end_point - start_point).length();

    let mut m = Mesh::from(shape::Capsule {
        radius: 0.001,
        rings: 10,
        depth: length,
        latitudes: 10,
        longitudes: 10,
        uv_profile: Default::default()
    });

    let mat = StandardMaterial {
        base_color: color,
        base_color_texture: None,
        roughness: 0.0,
        metallic: 0.0,
        metallic_roughness_texture: None,
        reflectance: 0.0,
        normal_map: None,
        double_sided: false,
        occlusion_texture: None,
        emissive: Default::default(),
        emissive_texture: None,
        unlit: true
    };

    let midpoint = (end_point + start_point) / 2.0;
    let y_axis = (end_point - start_point) / length;
    let cross1 = y_axis.cross( Vec3::new(0.,1.,0.));
    let x_axis = if (cross1.length() == 0.0) { y_axis.cross( Vec3::new(0.,0.,1.)) } else { cross1 };
    let z_axis = x_axis.cross(y_axis);
    let mat3 = Mat3::from_cols_array_2d( &[ [x_axis[0], x_axis[1], x_axis[2]], [y_axis[0], y_axis[1], y_axis[2]], [z_axis[0], z_axis[1], z_axis[2]] ] );

    let mut t = Transform::identity();
    t.rotation = Quat::from_rotation_mat3(&mat3);
    t.translation = midpoint;
    t.apply_non_uniform_scale(Vec3::new(width_in_mm, 1.0, width_in_mm));

    let id = commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(m),
        material: materials.add(mat),
        transform: t,
        ..Default::default()
    }).insert(line_type).id();
    return id;
}

pub fn spawn_line_lynx_space(commands: &mut Commands,
                             meshes: &mut ResMut<Assets<Mesh>>,
                             materials: &mut ResMut<Assets<StandardMaterial>>,
                             start_point: Vec3,
                             end_point: Vec3,
                             color: Color,
                             width_in_mm: f32,
                             line_type: LineType) -> Entity {
    /* by lynx space, I mean z up */

    let new_start_point = Vec3::new(start_point[0], start_point[2], -start_point[1]);
    let new_end_point = Vec3::new(end_point[0], end_point[2], -end_point[1]);

    return spawn_line_bevy_space(commands, meshes, materials, new_start_point, new_end_point, color, width_in_mm, line_type);
}

pub fn despawn_lines_of_given_line_type(commands: &mut Commands, line_type: &LineType, query: &Query<(Entity, &LineType)>) {
    for (entity, lt) in query.iter() {
        if lt == line_type {
            commands.entity(entity).despawn();
        }
    }
}