use crate::prelude::LynxVarsGeneric;
use crate::robot_modules::prelude::*;
use crate::utils::utils_math::prelude::vec_to_dvec;
use nalgebra::DVector;

pub use crate::app::app_states::perpetual::robot_management_systems::robot_set_entity_and_info_server::*;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SpawnNewPhysicalRobot(pub bool);

#[derive(Clone, Debug, PartialEq)]
pub enum RobotLinkSpawnType {
    Physical, Visualization
}

#[derive(Clone, Debug)]
pub enum RobotLinkMeshType {
    VislbleGlb,
    StandardMaterial,
    InvisibleMaterial
}

#[derive(Clone, Debug, PartialEq)]
pub enum RobotSetType {
    Physical, Visualization
}

#[derive(Clone, Debug)]
pub struct RobotSetJointValues{pub joint_values: DVector<f64>, pub robot_set_type: RobotSetType}
impl RobotSetJointValues {
    pub fn new_from_lynx_vars(lynx_vars: &mut LynxVarsGeneric, robot_set_type: RobotSetType) -> Result<Self, String> {
        let robot_world = get_lynx_var_ref_generic!(lynx_vars, RobotWorld, "robot_world")?;
        let num_dof = robot_world.get_robot_set_ref().get_total_num_dofs();
        return Ok(Self{ joint_values: vec_to_dvec(&vec![0.0; num_dof]), robot_set_type});
    }
}

#[derive(Clone, Debug)]
pub struct RobotLinkInfoContainer {
    pub robot_name: String,
    pub robot_link_spawn_type: RobotLinkSpawnType,
    pub robot_link_mesh_type: RobotLinkMeshType,
    pub robot_server_vector_idx: usize,
    pub robot_set_idx: usize,
    pub robot_link_idx: usize,
}



