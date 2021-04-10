use crate::utils::utils_parsing::urdf_joint::*;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use nalgebra::{Vector3, Matrix3, UnitQuaternion, Quaternion, DVector, Unit};
use serde::{Serialize, Deserialize};
use crate::utils::utils_math::vector_utils::get_orthogonal_vector;

#[derive(Clone, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    pub urdf_joint: URDFJoint,
    pub joint_idx: usize,
    pub preceding_link_idx: usize,
    pub child_link_idx: usize,
    pub origin_offset: ImplicitDualQuaternion,
    pub has_origin_offset: bool,
    pub dof_rotation_axes: Vec<Vector3<f64>>,
    pub dof_translation_axes: Vec<Vector3<f64>>,
    pub dof_rotation_axes_as_units: Vec<Unit<Vector3<f64>>>,
    pub num_dofs: usize,
    pub active: bool
}

impl Joint {
    pub fn new(urdf_joint: URDFJoint, joint_idx: usize, preceding_link_idx: usize, child_link_idx: usize) -> Self {
        let name = urdf_joint.name.clone();

        let origin_offset = ImplicitDualQuaternion::new_from_euler_angles( urdf_joint.origin_rpy[0], urdf_joint.origin_rpy[1], urdf_joint.origin_rpy[2],
                                                                           Vector3::new(urdf_joint.origin_xyz[0], urdf_joint.origin_xyz[1], urdf_joint.origin_xyz[2]));
        let mut dof_rotation_axes = Vec::new();
        let mut dof_translation_axes = Vec::new();
        let mut dof_rotation_axes_as_units = Vec::new();

        let mut has_origin_offset = true;
        if urdf_joint.origin_rpy[0] == 0.0 && urdf_joint.origin_rpy[1] == 0.0 && urdf_joint.origin_rpy[2] == 0.0 &&
            urdf_joint.origin_xyz[0] == 0.0 && urdf_joint.origin_xyz[1] == 0.0 && urdf_joint.origin_xyz[2] == 0.0 {
            has_origin_offset = false;
        }

        let mut out_joint = Self { name, urdf_joint, joint_idx, preceding_link_idx, child_link_idx, origin_offset, has_origin_offset, dof_rotation_axes, dof_translation_axes, dof_rotation_axes_as_units, num_dofs: 0, active: true };

        out_joint._set_dof_axes();
        out_joint.set_dof_rotation_axes_as_units();

        return out_joint
    }

    pub fn new_without_urdf_joint(joint_idx: usize, preceding_link_idx: usize, child_link_idx: usize) -> Self {
        let urdf_joint = URDFJoint::new_empty();
        return Self::new(urdf_joint, joint_idx, preceding_link_idx, child_link_idx);
    }

    fn _set_dof_axes(&mut self) -> Result<(), String> {
        let joint_type = self.urdf_joint.joint_type.clone();
        let axis = self.urdf_joint.axis.clone();

        if joint_type == "Revolute".to_string() {
            self.dof_rotation_axes.push(axis);
        } else if joint_type == "Continuous".to_string() {
            self.dof_rotation_axes.push(axis);
        } else if joint_type == "Prismatic".to_string() {
            self.dof_translation_axes.push(axis);
        } else if joint_type == "Floating".to_string() {
            self.dof_translation_axes.push( Vector3::new(1., 0., 0.) );
            self.dof_translation_axes.push( Vector3::new(0., 1., 0.) );
            self.dof_translation_axes.push( Vector3::new(0., 0., 1.) );

            self.dof_rotation_axes.push( Vector3::new(1., 0., 0.) );
            self.dof_rotation_axes.push( Vector3::new(0., 1., 0.) );
            self.dof_rotation_axes.push( Vector3::new(0., 0., 1.) );
        } else if joint_type == "Planar".to_string() {
            let mut v = DVector::from_element(3, 0.0);
            v[0] = axis[0]; v[1] = axis[1]; v[2] = axis[2];

            let v1 = get_orthogonal_vector(&v)?;
            let v2 = v.cross(&v1);

            self.dof_translation_axes.push( Vector3::new(v1[0], v1[1], v1[2]) );
            self.dof_translation_axes.push( Vector3::new(v2[0], v2[1], v2[2]) );
        }

        self.num_dofs = self.dof_rotation_axes.len() + self.dof_translation_axes.len();

        Ok(())
    }

    pub fn set_dof_rotation_axes_as_units(&mut self) {
        let l = self.dof_rotation_axes.len();
        if l == 0 { return; }

        for i in 0..l {
            self.dof_rotation_axes_as_units.push( Unit::new_normalize( self.dof_rotation_axes[i].clone() ) );
        }
    }
}