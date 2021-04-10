use crate::utils::utils_parsing::urdf_link::*;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use serde::{Serialize, Deserialize};


#[derive(Clone, Serialize, Deserialize)]
pub struct Link {
    pub name: String,
    pub urdf_link: URDFLink,
    pub link_idx: usize,
    pub preceding_link_idx: Option<usize>,
    pub children_link_idxs: Vec<usize>,
    pub preceding_joint_idx: Option<usize>,
    pub children_joint_idxs: Vec<usize>,
    pub active: bool
}

impl Link {
    pub fn new(urdf_link: URDFLink, link_idx: usize, preceding_link_idx: Option<usize>, children_link_idxs: Vec<usize>, preceding_joint_idx: Option<usize>, children_joint_idxs: Vec<usize>) -> Self {
        let name = urdf_link.name.clone();
        let mut active = Self::_decide_on_active(&urdf_link);

        Self { name, urdf_link, link_idx, preceding_link_idx, children_link_idxs, preceding_joint_idx, children_joint_idxs, active }
    }

    pub fn new_without_urdf_link(link_idx: usize, preceding_link_idx: Option<usize>, children_link_idxs: Vec<usize>, preceding_joint_idx: Option<usize>, children_joint_idxs: Vec<usize>) -> Self {
        let urdf_link = URDFLink::new_empty();
        return Self::new( urdf_link, link_idx, preceding_link_idx, children_link_idxs, preceding_joint_idx, children_joint_idxs );
    }

    pub fn has_mesh_filename(&self, mesh_filename: String) -> Option< MeshFilenameType > {
        if self.urdf_link.includes_collision_info {
            if self.urdf_link.collision[0].filename.is_some() {
                if self.urdf_link.collision[0].filename.as_ref().unwrap().clone() == mesh_filename.clone() { return Some(MeshFilenameType::Collision); }
            }
        }

        if self.urdf_link.includes_visual_info {
            if self.urdf_link.visual[0].filename.is_some() {
                if self.urdf_link.visual[0].filename.as_ref().unwrap().clone() == mesh_filename.clone() { return Some(MeshFilenameType::Visual); }
            }
        }

        return None;
    }

    fn _decide_on_active(urdf_link: &URDFLink) -> bool {
        return true;
    }
}


pub enum MeshFilenameType {
    Visual,
    Collision
}