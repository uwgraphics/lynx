use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use crate::robot_modules::joint::Joint;
use crate::utils::utils_math::nalgebra_utils::*;
use nalgebra::{DVector, UnitQuaternion, Vector3, Unit};
use termion::{style, color};

/*
all about mapping an input vector x to the right DOFs on the right joints
*/

#[derive(Clone)]
pub struct RobotDOFModule {
    _num_dofs: usize,
    _joint_idx_to_input_x_start_idx: Vec<usize>, // mapping from joint idx to the start idx of an input joint state vector
    _input_x_idx_to_joint_idx_and_subidx: Vec<(usize, usize)>,
    _input_x_idx_to_joint_idx_and_subidx_and_subjoint_type: Vec<(usize, String, usize)>,
    _joints_copy: Vec<Joint>
}

impl RobotDOFModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule) -> Self {
        let _joint_idx_to_input_x_start_idx = Vec::new();
        let _input_x_idx_to_joint_idx_and_subidx = Vec::new();
        let _input_x_idx_to_joint_idx_and_subidx_and_subjoint_type = Vec::new();
        let _joints_copy = robot_configuration_module.robot_model_module.joints.clone();

        let mut out_self = Self {
            _num_dofs: 0,
            _joint_idx_to_input_x_start_idx, _input_x_idx_to_joint_idx_and_subidx,
            _input_x_idx_to_joint_idx_and_subidx_and_subjoint_type, _joints_copy
        };

        out_self._set_num_dofs(robot_configuration_module);
        out_self._set_joint_idx_to_input_x_start_idx_vector(robot_configuration_module);
        out_self._set_input_x_idx_to_joint_idx_and_subidx(robot_configuration_module);
        out_self._set_input_x_idx_to_joint_idx_and_subidx_and_subjoint_type(robot_configuration_module);

        return out_self;
    }

    fn _set_num_dofs(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        let mut num_dofs = 0 as usize;

        let l = robot_configuration_module.robot_model_module.joints.len();
        for i in 0..l {
            if robot_configuration_module.robot_model_module.joints[i].active {
                num_dofs += robot_configuration_module.robot_model_module.joints[i].num_dofs;
            }
        }

        self._num_dofs = num_dofs;
    }

    fn _set_joint_idx_to_input_x_start_idx_vector(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        let num_joints = robot_configuration_module.robot_model_module.joints.len();
        let mut count = 0;
        for i in 0..num_joints {
            if robot_configuration_module.robot_model_module.joints[i].active && robot_configuration_module.robot_model_module.joints[i].num_dofs > 0 {
                self._joint_idx_to_input_x_start_idx.push( count );
                count += robot_configuration_module.robot_model_module.joints[i].num_dofs;
            } else {
                self._joint_idx_to_input_x_start_idx.push( usize::max_value() );
            }
        }
    }

    fn _set_input_x_idx_to_joint_idx_and_subidx(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        self._input_x_idx_to_joint_idx_and_subidx = vec![ (0,0); self._num_dofs];

        let mut x_curr_idx = 0 as usize;
        let num_joints = robot_configuration_module.robot_model_module.joints.len();
        for i in 0..num_joints {
            if robot_configuration_module.robot_model_module.joints[i].active {

                let l = robot_configuration_module.robot_model_module.joints[i].num_dofs;
                for j in 0..l {
                    self._input_x_idx_to_joint_idx_and_subidx[x_curr_idx] = (i, j);
                    x_curr_idx += 1;
                }
            }
        }
    }

    fn _set_input_x_idx_to_joint_idx_and_subidx_and_subjoint_type(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        // must be run after _set_input_x_idx_to_joint_idx_and_subidx

        let num_dofs = self._num_dofs;
        for i in 0..num_dofs {
            let (joint_idx, sub_idx) = self._input_x_idx_to_joint_idx_and_subidx[i];
            let num_translation_dofs = robot_configuration_module.robot_model_module.joints[joint_idx].dof_translation_axes.len();
            let num_rotation_dofs = robot_configuration_module.robot_model_module.joints[joint_idx].dof_rotation_axes.len();

            if sub_idx >= num_translation_dofs {
                let joint_type = "rotation".to_string();
                let rotation_joint_idx = sub_idx - num_translation_dofs;
                self._input_x_idx_to_joint_idx_and_subidx_and_subjoint_type.push( (joint_idx, joint_type, rotation_joint_idx) );
            } else {
                let joint_type = "translation".to_string();
                let rotation_joint_idx = sub_idx;
                self._input_x_idx_to_joint_idx_and_subidx_and_subjoint_type.push( (joint_idx, joint_type, rotation_joint_idx) );
            }

        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_num_dofs(&self) -> usize { return self._num_dofs; }

    pub fn get_input_x_starting_idx_from_joint_idx(&self, joint_idx: usize) -> usize {
        return self._joint_idx_to_input_x_start_idx[joint_idx];
    }

    pub fn get_joint_idx_type_and_subidx_from_input_x_idx(&self, x_idx: usize) -> (usize, String, usize) {
        return self._input_x_idx_to_joint_idx_and_subidx_and_subjoint_type[x_idx].clone();
    }

    pub fn print_joint_dof_order(&self) {
        let num_dofs = self._num_dofs;
        for i in 0..num_dofs {
            let res = self.get_joint_idx_type_and_subidx_from_input_x_idx(i);
            let mut axis = Vector3::zeros();
            if res.1 == "rotation".to_string() {
                axis = self._joints_copy[res.0].dof_rotation_axes[ res.2 ].clone();
            } else {
                axis = self._joints_copy[res.0].dof_translation_axes[ res.2 ].clone();
            }
            println!("{}{}joint dof {} ---> {}{} ({} sub-joint axis local [{}, {}, {}])", style::Bold, color::Fg(color::Blue), i, style::Reset, self._joints_copy[res.0].name, res.1, axis[0], axis[1], axis[2]);
        }
        println!();
    }

    pub fn print_input_x_vals_next_to_joint_dof_names(&self, x: &DVector<f64>) {
        let num_dofs = self._num_dofs;
        for i in 0..num_dofs {
            let res = self.get_joint_idx_type_and_subidx_from_input_x_idx(i);
            let mut axis = Vector3::zeros();
            if res.1 == "rotation".to_string() {
                axis = self._joints_copy[res.0].dof_rotation_axes[ res.2 ].clone();
            } else {
                axis = self._joints_copy[res.0].dof_translation_axes[ res.2 ].clone();
            }
            println!("{}{}joint dof {} ---> {}{} ({} sub-joint axis local [{}, {}, {}]) --> {}", style::Bold, color::Fg(color::Blue), i, style::Reset,self._joints_copy[res.0].name, res.1, axis[0], axis[1], axis[2], x[i]);
        }
        println!();
    }

    pub fn print_input_x_vals_next_to_joint_dof_names_vec(&self, x: &Vec<f64>) {
        self.print_input_x_vals_next_to_joint_dof_names( &vec_to_dvec(x));
    }

    pub fn copy_joint_idx_to_input_x_start_idx(&self) -> Vec<usize> {
        return self._joint_idx_to_input_x_start_idx.clone();
    }
}