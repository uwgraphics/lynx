use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use bevy::transform::prelude::Transform;
use bevy::math::{Vec3, Quat};
use nalgebra::Vector3;

pub fn convert_z_up_idq_to_y_up_bevy_transform(idq: &ImplicitDualQuaternion) -> Transform {
    let idq_new = ImplicitDualQuaternion::new_from_euler_angles(-std::f64::consts::FRAC_PI_2, 0., 0., Vector3::new(0.,0.,0.)).multiply(idq);
    let t = Transform {
        translation: Vec3::new( idq_new.translation.x as f32, idq_new.translation.y as f32, idq_new.translation.z as f32 ),
        rotation: Quat::from_xyzw(idq_new.quat.i as f32, idq_new.quat.j as f32, idq_new.quat.k as f32, idq_new.quat.w as f32),
        scale: Vec3::new(1.,1.,1.)
    };
    return t;
}

pub fn convert_z_up_idq_to_y_up_bevy_transform_with_offset(idq: &ImplicitDualQuaternion, offset: &ImplicitDualQuaternion) -> Transform {
    let idq0 = idq.multiply(offset);
    // let idq0 = offset.multiply(idq);
    let idq_new = ImplicitDualQuaternion::new_from_euler_angles(-std::f64::consts::FRAC_PI_2, 0., 0., Vector3::new(0.,0.,0.)).multiply(&idq0);
    let t = Transform {
        translation: Vec3::new( idq_new.translation.x as f32, idq_new.translation.y as f32, idq_new.translation.z as f32 ),
        rotation: Quat::from_xyzw(idq_new.quat.i as f32, idq_new.quat.j as f32, idq_new.quat.k as f32, idq_new.quat.w as f32),
        scale: Vec3::new(1.,1.,1.)
    };
    return t;
}

pub fn convert_z_up_idq_to_y_up_bevy_transform_with_visual_and_urdf_mesh_offset(idq: &ImplicitDualQuaternion, visual_offset: &ImplicitDualQuaternion, urdf_offset: &ImplicitDualQuaternion) -> Transform {
    let idq0 = idq.multiply(visual_offset).multiply(urdf_offset);
    // let idq0 = offset.multiply(idq);
    let idq_new = ImplicitDualQuaternion::new_from_euler_angles(-std::f64::consts::FRAC_PI_2, 0., 0., Vector3::new(0.,0.,0.)).multiply(&idq0);
    let t = Transform {
        translation: Vec3::new( idq_new.translation.x as f32, idq_new.translation.y as f32, idq_new.translation.z as f32 ),
        rotation: Quat::from_xyzw(idq_new.quat.i as f32, idq_new.quat.j as f32, idq_new.quat.k as f32, idq_new.quat.w as f32),
        scale: Vec3::new(1.,1.,1.)
    };
    return t;
}

