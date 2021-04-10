use nalgebra::{Vector3, UnitQuaternion, Quaternion, Point3};
use crate::utils::utils_se3::transformation_utils::*;
use crate::utils::utils_math::vector_utils::*;
use std::fmt;
use std::ops::Mul;
use serde::{Serialize, Deserialize};


/* The chaining of transformations using IDQs translates "first" then rotates, same as homogeneous matrices */

#[derive(Clone, Serialize, Deserialize)]
pub struct ImplicitDualQuaternion {
    pub quat: UnitQuaternion<f64>,
    pub translation: Vector3<f64>,
    pub is_identity: bool,
    pub quat_is_identity: bool,
    pub translation_is_zeros: bool
}

impl ImplicitDualQuaternion {
    pub fn new(quat: UnitQuaternion<f64>, translation: Vector3<f64>) -> Self {
        let mut quat_is_identity = false;
        if (quat.w.abs() - 1.0).abs() < 0.000001 && quat.i.abs() < 0.000001 && quat.j.abs() < 0.000001 && quat.k.abs() < 0.000001 {
            quat_is_identity = true;
        }
        let mut translation_is_zeros = false;
        if translation[0].abs() < 0.000001 && translation[1].abs() < 0.000001 && translation[2] < 0.000001 {
            translation_is_zeros = true;
        }

        let is_identity = quat_is_identity && translation_is_zeros;

        Self {quat, translation, is_identity, quat_is_identity, translation_is_zeros}
    }

    pub fn new_identity() -> Self {
        return Self::new_from_euler_angles(0.,0.,0., Vector3::zeros());
    }

    pub fn new_from_euler_angles(rx: f64, ry: f64, rz: f64, translation: Vector3<f64>) -> Self {
        let quat = UnitQuaternion::from_euler_angles(rx, ry, rz);
        return Self::new(quat, translation)
    }

    pub fn new_from_quat_vec(quat_vec: Vec<f64>, translation: Vector3<f64>) -> Self {
        let tmp = Quaternion::new(quat_vec[0], quat_vec[1], quat_vec[2], quat_vec[3]);
        let quat = UnitQuaternion::from_quaternion(tmp);
        return Self::new(quat, translation);
    }

    pub fn multiply(&self, other: &ImplicitDualQuaternion) -> ImplicitDualQuaternion {
        let mut out_quat = self.quat * &other.quat;
        let mut out_translation = self.quat * &other.translation + &self.translation;
        return ImplicitDualQuaternion::new(out_quat, out_translation);
    }

    pub fn multiply_shortcircuit(&self, other: &ImplicitDualQuaternion) -> ImplicitDualQuaternion {
        if self.is_identity { return other.clone(); }
        if other.is_identity { return self.clone(); }

        let mut out_quat = self.quat.clone();
        if self.quat_is_identity { out_quat = other.quat.clone(); }
        else if other.quat_is_identity {
            // do nothing
        }
        else {
            out_quat *= &other.quat;
        }

        let mut out_translation = self.translation.clone();
        if self.quat_is_identity && !other.translation_is_zeros {
            out_translation += &other.translation;
        } else if self.quat_is_identity && other.translation_is_zeros {
            // do nothing
        } else {
            out_translation += &self.quat * &other.translation;
        }

        return ImplicitDualQuaternion::new(out_quat, out_translation);
    }

    pub fn multiply_by_vector3(&self, point: &Vector3<f64>) -> Vector3<f64> {
        return self.quat * point + self.translation;
    }

    pub fn multiply_by_vector3_shortcircuit(&self, point: &Vector3<f64>) -> Vector3<f64> {
        if self.is_identity { return point.clone(); }
        if self.quat_is_identity { return point + self.translation; }
        if self.translation_is_zeros { return self.quat * point; }
        return self.quat * point + self.translation;
    }

    pub fn multiply_by_point3(&self, point: &Point3<f64>) -> Point3<f64> {
        return self.quat * point + self.translation;
    }

    pub fn multiply_by_point3_shortcircuit(&self, point: &Point3<f64>) -> Point3<f64> {
        if self.is_identity { return point.clone(); }
        if self.quat_is_identity { return point + self.translation; }
        if self.translation_is_zeros { return self.quat * point; }
        return self.quat * point + self.translation;
    }

    pub fn multiply_mut(&mut self, other: &ImplicitDualQuaternion) {
        self.quat = self.quat * &other.quat;
        self.translation = self.quat * &other.translation + &self.translation;
    }

    pub fn inverse(&self) -> ImplicitDualQuaternion {
        let mut new_quat = self.quat.inverse();
        let mut new_translation = &new_quat * -self.translation.clone();
        return ImplicitDualQuaternion::new(new_quat, new_translation);
    }
}

impl fmt::Debug for ImplicitDualQuaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ImplicitDualQuaternion")
         .field("quat", &self.quat)
         .field("translation", &self.translation)
         .finish()
    }
}