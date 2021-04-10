use urdf_rs::*;
use nalgebra::{Vector3, Matrix3};
use serde::{Serialize, Deserialize};

#[derive(Clone, Serialize, Deserialize)]
pub struct URDFJoint {
    pub name: String,
    pub joint_type: String,
    pub origin_xyz: Vector3<f64>,
    pub origin_rpy: Vector3<f64>,
    pub parent_link: String,
    pub child_link: String,
    pub axis: Vector3<f64>,
    pub includes_limits: bool,
    pub limits_lower: f64,
    pub limits_upper: f64,
    pub limits_effort: f64,
    pub limits_velocity: f64,
    pub includes_dynamics: bool,
    pub dynamics_damping: f64,
    pub dynamics_friction: f64,
    pub includes_mimic: bool,
    pub mimic_joint: String,
    pub mimic_multiplier: f64,
    pub mimic_offset: f64,
    pub includes_safety: bool,
    pub safety_soft_lower_limit: f64,
    pub safety_soft_upper_limit: f64,
    pub safety_k_position: f64,
    pub safety_k_velocity: f64
}

impl URDFJoint {
    pub fn new_from_urdf_joint(joint: &Joint) -> Self {
        let name = joint.name.clone();

        let mut joint_type = "".to_string();
        let jt = joint.joint_type.clone();
        match jt {
            JointType::Revolute => { joint_type = "Revolute".to_string() },
            JointType::Continuous => { joint_type = "Continuous".to_string() },
            JointType::Prismatic => { joint_type = "Prismatic".to_string() },
            JointType::Fixed => { joint_type = "Fixed".to_string() },
            JointType::Floating => { joint_type = "Floating".to_string() },
            JointType::Planar => { joint_type = "Planar".to_string() },
            JointType::Spherical => { joint_type = "Spherical".to_string() },
        }

        let origin_xyz = Vector3::new( joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2] );
        let origin_rpy = Vector3::new( joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2] );

        let parent_link = joint.parent.link.clone();
        let child_link = joint.child.link.clone();

        let axis = Vector3::new( joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2] );

        let mut includes_limits = true;
        let limits_lower = joint.limit.lower.clone();
        let limits_upper = joint.limit.upper.clone();
        let limits_effort = joint.limit.effort.clone();
        let limits_velocity = joint.limit.velocity.clone();

        if limits_lower == 0.0 && limits_upper == 0.0 {
            includes_limits = false;
        }

        let mut includes_dynamics = true;
        let dynamics_damping = joint.dynamics.damping.clone();
        let dynamics_friction = joint.dynamics.friction.clone();

        if dynamics_damping == 0.0 && dynamics_friction == 0.0 {
            includes_dynamics = false;
        }

        let mut includes_mimic = true;
        let mimic_joint = joint.mimic.joint.clone();
        let mimic_multiplier = joint.mimic.multiplier.clone();
        let mimic_offset = joint.mimic.offset.clone();

        if mimic_joint == "".to_string() {
            includes_mimic = false;
        }

        let mut includes_safety = true;
        let safety_soft_lower_limit = joint.safety_controller.soft_lower_limit;
        let safety_soft_upper_limit = joint.safety_controller.soft_upper_limit;
        let safety_k_position = joint.safety_controller.k_position;
        let safety_k_velocity = joint.safety_controller.k_velocity;

        if safety_soft_lower_limit == 0.0 && safety_soft_upper_limit == 0.0 {
            includes_safety = false;
        }

        Self {name, joint_type, origin_xyz, origin_rpy, parent_link, child_link, axis,
            includes_limits, limits_lower, limits_upper, limits_effort, limits_velocity,
            includes_dynamics, dynamics_damping, dynamics_friction, includes_mimic,
            mimic_joint, mimic_multiplier, mimic_offset,
            includes_safety, safety_soft_lower_limit, safety_soft_upper_limit, safety_k_position, safety_k_velocity}
    }

    pub fn new_empty() -> Self {
        return Self{ name: "".to_string(), joint_type: "".to_string(), origin_xyz: Vector3::zeros(), origin_rpy: Vector3::zeros(),
            parent_link: "".to_string(), child_link: "".to_string(), axis: Vector3::zeros(),
            includes_limits: false, limits_lower: 0.0, limits_upper: 0.0, limits_effort: 0.0, limits_velocity: 0.0,
            includes_dynamics: false, dynamics_damping: 0.0, dynamics_friction: 0.0,
            includes_mimic: false, mimic_joint: "".to_string(), mimic_multiplier: 0.0, mimic_offset: 0.0,
            includes_safety: false, safety_soft_lower_limit: 0.0, safety_soft_upper_limit: 0.0, safety_k_position: 0.0, safety_k_velocity: 0.0 };
    }

}