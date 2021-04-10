use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use crate::robot_modules::robot_dof_module::RobotDOFModule;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use crate::robot_modules::{joint::Joint, link::Link};
use nalgebra::{DVector, UnitQuaternion, Vector3, Unit};
use crate::utils::utils_math::nalgebra_utils::*;
use termion::{color, style};

#[derive(Clone)]
pub struct RobotFKModule {
    _predecessor_link_idxs: Vec<usize>,
    _predecessor_joint_idxs: Vec<usize>,
    _num_links: usize,
    _link_tree_traversal_layers_copy: Vec<Vec<usize>>,
    _links_copy: Vec<Link>,
    _joints_copy: Vec<Joint>,
    _num_dofs: usize,
    _joint_idx_to_input_x_start_idx_copy: Vec<usize>,
    _default_starting_out_vec: Vec<Option<ImplicitDualQuaternion>>
}

impl RobotFKModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule, robot_dof_module: &RobotDOFModule) -> Self {
        let _predecessor_link_idxs = Vec::new();
        let _predecessor_joint_idxs = Vec::new();
        let _num_links = robot_configuration_module.robot_model_module.links.len();
        let _link_tree_traversal_layers_copy = robot_configuration_module.robot_model_module.link_tree_traversal_layers.clone();
        let _links_copy = robot_configuration_module.robot_model_module.links.clone();
        let _joints_copy = robot_configuration_module.robot_model_module.joints.clone();
        let _num_dofs = robot_dof_module.get_num_dofs().clone();
        let _joint_idx_to_input_x_start_idx_copy = robot_dof_module.copy_joint_idx_to_input_x_start_idx();

        let mut _default_starting_out_vec = vec![ None; _num_links ];
        let world_idx = robot_configuration_module.robot_model_module.world_link_idx;
        _default_starting_out_vec[world_idx] = Some(ImplicitDualQuaternion::new_identity());

        let mut out_self = Self { _predecessor_link_idxs,
            _predecessor_joint_idxs, _num_links, _link_tree_traversal_layers_copy, _links_copy,
            _joints_copy, _num_dofs, _joint_idx_to_input_x_start_idx_copy, _default_starting_out_vec };

        out_self._set_predecessor_links_and_joints(robot_configuration_module);

        return out_self;
    }

    fn _set_predecessor_links_and_joints(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        let l = self._num_links;
        for i in 0..l {
            let predecessor_link_idx = robot_configuration_module.robot_model_module.links[i].preceding_link_idx;
            let predecessor_joint_idx = robot_configuration_module.robot_model_module.links[i].preceding_joint_idx;

            if predecessor_link_idx.is_none() { self._predecessor_link_idxs.push( usize::max_value() ); }
            else { self._predecessor_link_idxs.push( predecessor_link_idx.unwrap() ); }

            if predecessor_joint_idx.is_none() { self._predecessor_joint_idxs.push( usize::max_value() ); }
            else { self._predecessor_joint_idxs.push( predecessor_joint_idx.unwrap() ); }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn compute_fk(&self, x: &DVector<f64> ) -> Result<RobotFKResult, String> {
        if x.len() != self._num_dofs {
            println!("{}{}ERROR: robot state vector x does not have expected number of dofs ({} instead of {}).  Cannot compute FK. {}", color::Fg(color::Red), style::Bold, x.len(), self._num_dofs, style::Reset);
            return Err( format!("robot state vector x does not have expected number of dofs ({} instead of {}).  Cannot compute FK.", x.len(), self._num_dofs) );
        }

        let mut out_vec = self._default_starting_out_vec.clone();

        let num_layers = self._link_tree_traversal_layers_copy.len();
        if num_layers == 1 { return Ok( RobotFKResult::new(x, out_vec) ); }

        for i in 1..num_layers {
            let l = self._link_tree_traversal_layers_copy[i].len();
            for j in 0..l {
                let curr_link_idx = self._link_tree_traversal_layers_copy[i][j];
                let predecessor_link_idx = self._predecessor_link_idxs[curr_link_idx];
                let predecessor_joint_idx = self._predecessor_joint_idxs[curr_link_idx];
                self._compute_fk_on_single_link(x, curr_link_idx, predecessor_link_idx, predecessor_joint_idx, &mut out_vec);
            }
        }

        return Ok( RobotFKResult::new(x, out_vec) );
    }

    pub fn compute_fk_vec(&self, x: &Vec<f64> ) -> Result<RobotFKResult, String>  {
        return self.compute_fk( &vec_to_dvec(x) );
    }

    pub fn compute_fk_on_all_zeros_config(&self) -> Result<RobotFKResult, String> {
        let x = vec![ 0.0; self._num_dofs ];
        return self.compute_fk_vec(&x);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn compute_fk_gradient_perturbations(&self, x: &DVector<f64>) -> Result<RobotFKGradientPerturbationsResult, String> {
        let mut perturbation_fk_results = Vec::new();
        let perturbation = 0.000001;

        let mut x_h = x.clone();

        let l = x.len();
        for i in 0..l {
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            perturbation_fk_results.push( self.compute_fk(&x_h)? );
        }

        return Ok( RobotFKGradientPerturbationsResult::new(x, perturbation, perturbation_fk_results) );
    }

    pub fn compute_fk_gradient_perturbations_vec(&self, x: &Vec<f64>) -> Result<RobotFKGradientPerturbationsResult, String> {
        return self.compute_fk_gradient_perturbations(&vec_to_dvec(x));
    }

    pub fn compute_fk_gradient_perturbations_on_all_zeros_config(&self) -> Result<RobotFKGradientPerturbationsResult, String> {
        let x = vec![ 0.0; self._num_dofs ];
        return self.compute_fk_gradient_perturbations_vec(&x);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _compute_fk_on_single_link(&self, x: &DVector<f64>, curr_link_idx: usize, predecessor_link_idx: usize, predecessor_joint_idx: usize, out_vec: &mut Vec<Option<ImplicitDualQuaternion>>) {
        if !self._links_copy[curr_link_idx].active { return; }


        let mut out_pose = out_vec[predecessor_link_idx].as_ref().unwrap().clone();
        if self._joints_copy[predecessor_joint_idx].has_origin_offset {
            out_pose = out_pose.multiply_shortcircuit(  &self._joints_copy[predecessor_joint_idx].origin_offset  );
        }

        if !self._joints_copy[predecessor_joint_idx].active || self._joints_copy[predecessor_joint_idx].num_dofs == 0 {
            out_vec[curr_link_idx] = Some( out_pose.clone() );
            return;
        }

        let x_dof_start_idx = self._get_input_x_starting_idx_from_joint_idx( predecessor_joint_idx );
        let mut count = 0 as usize;

        let l = self._joints_copy[predecessor_joint_idx].dof_translation_axes.len();
        for i in 0..l {
            if x[x_dof_start_idx + count] == 0.0 { count += 1; continue; }
            else {
                let dof_translation = x[x_dof_start_idx + count] * &self._joints_copy[predecessor_joint_idx].dof_translation_axes[i];
                let idq = ImplicitDualQuaternion::new_from_euler_angles(0., 0., 0., dof_translation);
                out_pose = out_pose.multiply_shortcircuit( &idq );

                count += 1;
            }
        }

        let l = self._joints_copy[predecessor_joint_idx].dof_rotation_axes.len();
        for i in 0..l {
            if x[x_dof_start_idx + count] == 0.0 { count += 1; continue; }
            else {
                let dof_quat = UnitQuaternion::from_axis_angle( &self._joints_copy[predecessor_joint_idx].dof_rotation_axes_as_units[i], x[x_dof_start_idx + count]);
                let idq = ImplicitDualQuaternion::new( dof_quat, Vector3::zeros() );
                out_pose = out_pose.multiply_shortcircuit( &idq );

                count += 1;
            }
        }

        out_vec[curr_link_idx] = Some( out_pose );
    }

    fn _get_input_x_starting_idx_from_joint_idx(&self, joint_idx: usize) -> usize {
        // functionality from from dof_module
        return self._joint_idx_to_input_x_start_idx_copy[joint_idx];
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_results_next_to_link_names(&self, res: &RobotFKResult, robot_configuration_module: &RobotConfigurationModule) {
        println!("fk x config: {:?}", res.get_x_ref().data.as_vec());
        let l = robot_configuration_module.robot_model_module.links.len();
        for i in 0..l {
            println!("   {:?} {:?}: {:?}", i, robot_configuration_module.robot_model_module.links[i].name, res.get_link_frames_ref()[i]);
        }
        println!();
    }

    pub fn print_results_from_all_zeros_config_next_to_link_names(&self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let res = self.compute_fk_on_all_zeros_config()?;
        self.print_results_next_to_link_names(&res, robot_configuration_module);
        Ok(())
    }
}


#[derive(Clone, Debug)]
pub struct RobotFKResult {
    _x: DVector<f64>,
    _link_frames: Vec<Option<ImplicitDualQuaternion>>
}

impl RobotFKResult {
    pub fn new(x: &DVector<f64>, link_frames: Vec<Option<ImplicitDualQuaternion>>) -> Self {
        return Self { _x: x.clone(), _link_frames: link_frames };
    }

    pub fn get_x_ref(&self) -> &DVector<f64> {
        return &self._x;
    }

    pub fn get_link_frames_ref(&self) -> &Vec<Option<ImplicitDualQuaternion>> {
        return &self._link_frames;
    }
}


#[derive(Clone, Debug)]
pub struct RobotFKGradientPerturbationsResult {
    _x: DVector<f64>,
    _perturbation: f64,
    _perturbation_fk_results: Vec<RobotFKResult>
}

impl RobotFKGradientPerturbationsResult {
    pub fn new(x: &DVector<f64>, perturbation: f64, perturbation_fk_results: Vec<RobotFKResult>) -> Self {
        return Self { _x: x.clone(), _perturbation: perturbation, _perturbation_fk_results: perturbation_fk_results };
    }

    pub fn get_x_ref(&self) -> &DVector<f64> {
        return &self._x;
    }

    pub fn get_perturbation(&self) -> f64 {
        return self._perturbation;
    }

    pub fn get_perturbation_fk_results_ref(&self) -> &Vec<RobotFKResult> {
        return &self._perturbation_fk_results;
    }

    pub fn print_summary(&self) {
        let r = self.get_perturbation_fk_results_ref();
        let l = r.len();
        for i in 0..l {
            println!("{:?}", r[i]);
        }
    }
}