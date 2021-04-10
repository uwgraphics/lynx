use crate::robot_modules::robot_fk_module::*;
use crate::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_optimization::loss_function::*;
use crate::utils::utils_math::nalgebra_utils::{vec_to_dvec, vector3_to_dvec};
use crate::utils::utils_robot_objective_specification::link_kinematic_objective_specification::{*};
use crate::utils::utils_se3::transformation_utils::*;
use crate::utils::utils_collisions::collision_object_utils::*;
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;
use crate::utils::utils_collisions::collision_environment::CollisionEnvironment;
use crate::utils::utils_robot_objective_specification::link_info::*;
use crate::utils::utils_math::geometry_utils::*;
use crate::utils::utils_recorders::prelude::*;
use nalgebra::{DVector, Vector3, Point3, UnitQuaternion};
use std::time::Instant;
use termion::{color, style};

/* objective function without loss function at the end.  low values are generally assumed to be "better" */
pub trait IsolatedObjectiveTerm : LynxVarsUser + AsLynxVarsUser + IsolatedObjectiveTermClone + Send + Sync {
    fn name(&self) -> String {
        return "".to_string();
    }
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String>;
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<DVector<f64>, String> {
        return self.gradient_finite_differencing(x, lynx_vars, recorder);
    }
    fn gradient_finite_differencing(&self, x: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<DVector<f64>, String> {
        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, recorder)?;

        let mut x_h = x.clone();

        let p = 0.000001;
        for i in 0..l {
            x_h[i] += p;
            if i > 0 { x_h[i-1] -= p; }
            let f_h = self.call(&x_h, lynx_vars, recorder)?;
            out_gradient[i] = ( (-f_0 + f_h) / p);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> { return Box::new( IdentityLoss ) }
    fn print_diagnostics_information(&self, lynx_vars: &mut LynxVarsGeneric, x: &Option<Vec<f64>>) {
        let recorder_none = &RecorderArcMutexOption::new_none();

        println!("\n{}{} Diagnostics information for isolated objective term {:?} {}", color::Fg(color::LightCyan), style::Bold, self.name(), style::Reset);
        println!("{}-------------------------------------------------------------{}", color::Fg(color::LightCyan), style::Reset);
        println!("{}{} 1. lynx vars validity check ---> {}", color::Fg(color::Blue), style::Bold, style::Reset);
        let valid = LynxVarsValidityChecker::check_valid_generic(lynx_vars,  self.as_lynx_vars_user(), true);

        if !valid {
            println!("{}{} Could not run any more diagnostics on an invalid objective.  Exiting. {}", color::Fg(color::Blue), style::Bold, style::Reset);
            println!("{}-------------------------------------------------------------{}", color::Fg(color::LightCyan), style::Reset);
            return;
        }

        if x.is_none() {
            println!("{}{} Could not run any more diagnostics because x is None.  Feel free to provide an x next time to run further diagnostics on.  Exiting. {}", color::Fg(color::Blue), style::Bold, style::Reset);
            println!("{}-------------------------------------------------------------{}", color::Fg(color::LightCyan), style::Reset);
            return;
        }

        let _x = vec_to_dvec(&x.as_ref().unwrap());

        println!("{}{} 2. call check ---> {}", color::Fg(color::Blue), style::Bold, style::Reset);

        let start = Instant::now();
        let res = self.call(&_x, lynx_vars, recorder_none);
        let mut elapsed = start.elapsed();

        if res.is_err() {
            println!("{}{} There was an error on self.call(...).  That error was {:?}.  Exiting.  {}", color::Fg(color::Blue), style::Bold, res.err().unwrap(), style::Reset);
            println!("{}-------------------------------------------------------------{}", color::Fg(color::LightCyan), style::Reset);
            return;
        }

        println!("{}    self.call(...) seemed to work correctly.  {}", color::Fg(color::Green), style::Reset);
        println!("{}    First returned value was {} and took {:?}.  {}", color::Fg(color::Green), res.ok().unwrap(), elapsed, style::Reset);
        for i in 0..99 {
            let start = Instant::now();
            let res = self.call(&_x, lynx_vars, recorder_none);
            elapsed += start.elapsed();
        }
        elapsed /= 100;
        println!("{}    Average run-time over 100 calls was {:?}.  {}", color::Fg(color::Green), elapsed, style::Reset);

        println!("{}{} 3. gradient check ---> {}", color::Fg(color::Blue), style::Bold, style::Reset);

        let start = Instant::now();
        let res = self.gradient(&_x, lynx_vars, recorder_none);
        let mut elapsed = start.elapsed();
        if res.is_err() {
            println!("{}{} There was an error on self.gradient(...).  That error was {:?}.  Exiting.  {}", color::Fg(color::Blue), style::Bold, res.err().unwrap(), style::Reset);
            println!("{}-------------------------------------------------------------{}", color::Fg(color::LightCyan), style::Reset);
            return;
        }

        let start = Instant::now();
        let res_fd = self.gradient_finite_differencing(&_x, lynx_vars, recorder_none);
        let mut elapsed_fd = start.elapsed();
        if res.is_err() {
            println!("{}{} There was an error on self.gradient_finite_differencing(...).  That error was {:?}.  Exiting.  {}", color::Fg(color::Blue), style::Bold, res_fd.err().unwrap(), style::Reset);
            println!("{}-------------------------------------------------------------{}", color::Fg(color::LightCyan), style::Reset);
            return;
        }

        let grad = res.unwrap();
        let grad_fd = res_fd.unwrap();

        let l = grad.len();
        let mut correct = true;
        for i in 0..l {
            let diff = (grad[i] - grad_fd[i]).abs();
            if diff > 0.0001 { correct = false; }
        }

        let mut elapsed_average = elapsed.clone();
        let mut elapsed_fd_average = elapsed_fd.clone();
        for i in 0..99 {
            let start = Instant::now();
            let res = self.gradient(&_x, lynx_vars, recorder_none);
            elapsed_average += start.elapsed();

            let start = Instant::now();
            let res = self.gradient_finite_differencing(&_x, lynx_vars, recorder_none);
            elapsed_fd_average += start.elapsed();
        }
        elapsed_average /= 100;
        elapsed_fd_average /= 100;

        if correct {
            println!("{}    self.gradient(...) seems to match output from self.gradient_finite_differencing(...). {}", color::Fg(color::Green), style::Reset);
            println!("{}    First returned gradient was -- {:?} and took {:?}. {}", color::Fg(color::Green), grad.data.as_vec(), elapsed, style::Reset);
            println!("{}    First returned gradient FD was {:?} and took {:?}. {}", color::Fg(color::Green), grad_fd.data.as_vec(), elapsed_fd, style::Reset);
            println!("{}    Average run-time of gradient over 100 calls was -- {:?}. {}", color::Fg(color::Green), elapsed_average, style::Reset);
            println!("{}    Average run-time of gradient FD over 100 calls was {:?}. {}", color::Fg(color::Green), elapsed_fd_average, style::Reset);
        } else {
            println!("{}    self.gradient(...) does not seem to match output from self.gradient_finite_differencing(...). {}", color::Fg(color::Red), style::Reset);
            println!("{}    First returned gradient    {:?} and took {:?}. {}", color::Fg(color::Red), grad.data.as_vec(), elapsed, style::Reset);
            println!("{}    First returned gradient FD {:?} and took {:?}. {}", color::Fg(color::Red), grad_fd.data.as_vec(), elapsed_fd, style::Reset);
            println!("{}    Average run-time of gradient over 100 calls was -- {:?}. {}", color::Fg(color::Red), elapsed_average, style::Reset);
            println!("{}    Average run-time of gradient FD over 100 calls was {:?}. {}", color::Fg(color::Red), elapsed_fd_average, style::Reset);
        }

        println!("{}-------------------------------------------------------------{}", color::Fg(color::LightCyan), style::Reset);

    }
}

pub trait IsolatedObjectiveTermClone {
    fn clone_box(&self) -> Box<dyn IsolatedObjectiveTerm>;
}
impl<T> IsolatedObjectiveTermClone for T where T: 'static + IsolatedObjectiveTerm + Clone {
    fn clone_box(&self) -> Box<dyn IsolatedObjectiveTerm> {
        Box::new(self.clone())
    }
}

pub struct IsolatedObjectiveTermBox(Box<dyn IsolatedObjectiveTerm>);
impl IsolatedObjectiveTermBox {
    pub fn name(&self) -> String { return self.0.name(); }
    pub fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<f64, String> {
        return self.0.call(x, lynx_vars, recorder);
    }
    pub fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<DVector<f64>, String> {
        return self.0.gradient(x, lynx_vars, recorder);
    }
    pub fn gradient_finite_differencing(&self, x: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption) -> Result<DVector<f64>, String> {
        return self.0.gradient_finite_differencing(x, lynx_vars, recorder);
    }
    pub fn get_default_loss_function(&self) -> Box<dyn LossFunction> { return self.0.get_default_loss_function(); }
    pub fn print_diagnostics_information(&self, lynx_vars: &mut LynxVarsGeneric, x: &Option<Vec<f64>>) {
        self.0.print_diagnostics_information(lynx_vars, x);
    }
    pub fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return self.0.get_lynx_vars_types();
    }
}
impl Clone for IsolatedObjectiveTermBox {
    fn clone(&self) -> Self {
        return Self(self.0.clone_box());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/*
#[derive(Clone)]
pub struct Test;
impl IsolatedObjectiveTerm for Test {
    fn name(&self) -> String {return "test".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        // let a = lynx_vars.get_f64_variable_ref("a")?;
        // let b = lynx_vars.get_f64_variable_ref("b")?;

        let a = get_lynx_var_ref!(lynx_vars, f64, "a")?;
        let b = get_lynx_var_ref!(lynx_vars, f64, "b")?;

        return Ok(a * x[0].powi(2) + b * x[1].powi(2));
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        // let a = lynx_vars.get_f64_variable_ref("a")?;
        // let b = lynx_vars.get_f64_variable_ref("b")?;

        let a = get_lynx_var_ref!(lynx_vars, f64, "a")?;
        let b = get_lynx_var_ref!(lynx_vars, f64, "b")?;

        let mut output = DVector::from_element(x.len(), 0.0);
        output[0] = a * 2.0 * x[0];
        output[1] = b * 2.0 * x[1];

        return Ok(output);
    }
}
impl LynxVarsUser for Test {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        // return vec![ LynxVarsType::F64("a"),  LynxVarsType::F64("b") ];
        return vec![ ("f64", "a"), ("f64", "b") ];
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct RobotLinkPoseGoalMatching;
impl IsolatedObjectiveTerm for RobotLinkPoseGoalMatching {
    fn name(&self) -> String {return "robot_link_pose_goal_matching".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        type LinkPoseMatchingSpecificationVec = Vec<LinkPoseMatchingSpecification>;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let pose_goals = get_lynx_var_ref!(lynx_vars, LinkPoseMatchingSpecificationVec, "link_pose_matching_specifications")?;
        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;

        if !(fk_res.get_x_ref() == x) {
            let l = x.len();
            for i in 0..l {
                if x[i].is_nan() { return Err("NaN was in x, could not continue optimizing".to_string()); }
            }

            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let mut error_sum = 0.0;

        let l = pose_goals.len();
        for i in 0..l {
            let link_idx = robot_salient_link_description.get_salient_links()[pose_goals[i].salient_link_idx].link_idx;
            let frame_option = &fk_res.get_link_frames_ref()[link_idx];
            if frame_option.is_none() {
                return Err("Salient link idx is None in RobotLinkPoseGoalMatching".to_string());
            }

            let frame_unwrap = frame_option.as_ref().unwrap();

            let error = implicit_dual_quaternion_disp_l2_magnitude(frame_unwrap, &pose_goals[i].goal_pose);
            error_sum += error;
        }

        return Ok(error_sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            // println!("{:?}, {:?}, {:?}", x_h, f_h, robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 1.0, 2) )
    }
}
impl LynxVarsUser for RobotLinkPoseGoalMatching {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotSalientLinkDescription", "robot_salient_link_description"),
                     ("LinkPoseMatchingSpecificationVec", "link_pose_matching_specifications"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")];
    }
}

#[derive(Clone)]
pub struct RobotLinkPositionGoalMatching;
impl IsolatedObjectiveTerm for RobotLinkPositionGoalMatching {
    fn name(&self) -> String {return "robot_link_position_goal_matching".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        type LinkPositionMatchingSpecificationVec = Vec<LinkPositionMatchingSpecification>;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let position_goals = get_lynx_var_ref!(lynx_vars, LinkPositionMatchingSpecificationVec, "link_position_matching_specifications")?;
        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;

        if !(fk_res.get_x_ref() == x) {
            let l = x.len();
            for i in 0..l {
                if x[i].is_nan() { return Err("NaN was in x, could not continue optimizing".to_string()); }
            }

            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let mut error_sum = 0.0;

        let l = position_goals.len();
        for i in 0..l {
            let link_idx = robot_salient_link_description.get_salient_links()[position_goals[i].salient_link_idx].link_idx;
            let frame_option = &fk_res.get_link_frames_ref()[link_idx];
            if frame_option.is_none() {
                return Err("Salient link idx is None in RobotLinkPoseGoalMatching".to_string());
            }

            let frame_unwrap = frame_option.as_ref().unwrap();

            let error = (&frame_unwrap.translation - &position_goals[i].goal_position).norm();
            error_sum += error;
        }

        return Ok(error_sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPositionGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            // println!("{:?}, {:?}, {:?}", x_h, f_h, robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 1.0, 2) )
    }
}
impl LynxVarsUser for RobotLinkPositionGoalMatching {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotSalientLinkDescription", "robot_salient_link_description"),
                     ("LinkPositionMatchingSpecificationVec", "link_position_matching_specifications"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")];
    }
}

#[derive(Clone)]
pub struct RobotLinkOrientationGoalMatching;
impl IsolatedObjectiveTerm for RobotLinkOrientationGoalMatching {
    fn name(&self) -> String {return "robot_link_orientation_goal_matching".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        type LinkOrientationMatchingSpecificationVec = Vec<LinkOrientationMatchingSpecification>;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let orientation_goals = get_lynx_var_ref!(lynx_vars, LinkOrientationMatchingSpecificationVec, "link_orientation_matching_specifications")?;
        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;

        if !(fk_res.get_x_ref() == x) {
            let l = x.len();
            for i in 0..l {
                if x[i].is_nan() { return Err("NaN was in x, could not continue optimizing".to_string()); }
            }

            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let mut error_sum = 0.0;

        let l = orientation_goals.len();
        for i in 0..l {
            let link_idx = robot_salient_link_description.get_salient_links()[orientation_goals[i].salient_link_idx].link_idx;
            let frame_option = &fk_res.get_link_frames_ref()[link_idx];
            if frame_option.is_none() {
                return Err("Salient link idx is None in RobotLinkOrientationGoalMatching".to_string());
            }

            let frame_unwrap = frame_option.as_ref().unwrap();

            let error1 = quaternion_disp(frame_unwrap.quat.clone(), orientation_goals[i].goal_orientation.clone()).norm();
            let error2 = quaternion_disp(UnitQuaternion::from_quaternion(frame_unwrap.quat.neg()), orientation_goals[i].goal_orientation.clone()).norm();
            error_sum += error1.min(error2);
        }

        return Ok(error_sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkOrientationGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            // println!("{:?}, {:?}, {:?}", x_h, f_h, robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 1.0, 2) )
    }
}
impl LynxVarsUser for RobotLinkOrientationGoalMatching {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotSalientLinkDescription", "robot_salient_link_description"),
                     ("LinkOrientationMatchingSpecificationVec", "link_orientation_matching_specifications"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")];
    }
}

#[derive(Clone)]
pub struct RobotLinkLookat;
impl IsolatedObjectiveTerm for RobotLinkLookat {
    fn name(&self) -> String {return "robot_link_lookat".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let l = x.len();
            for i in 0..l {
                if x[i].is_nan() { return Err("NaN was in x, could not continue optimizing".to_string()); }
            }

            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }


        type LinkLookAtSpecificationVec = Vec<LinkLookAtSpecification>;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let link_lookat_specifications = get_lynx_var_ref!(lynx_vars, LinkLookAtSpecificationVec, "link_lookat_specifications")?;

        let mut error_sum = 0.0;

        let l = link_lookat_specifications.len();
        for i in 0..l {
            let link_idx = robot_salient_link_description.get_salient_links()[link_lookat_specifications[i].salient_link_idx].link_idx;
            let forward_axis_option = &robot_salient_link_description.get_salient_links()[link_lookat_specifications[i].salient_link_idx].link_local_forward_axis;
            if forward_axis_option.is_none() {
                return Err(format!("link_local_forward_axis must be set in RobotLinkLookat"));
            }
            let forward_axis = forward_axis_option.as_ref().unwrap();
            let lookat_target = &link_lookat_specifications[i].lookat_target;

            let frame_option = &fk_res.get_link_frames_ref()[link_idx];
            if frame_option.is_none() {
                return Err("Salient link idx is None in RobotLinkLookat".to_string());
            }

            let frame_unwrap = frame_option.as_ref().unwrap();

            let frame_position = &frame_unwrap.translation;
            let frame_orientation_quat = &frame_unwrap.quat;
            let frame_orientation_rot_mat = frame_orientation_quat.to_rotation_matrix();
            let mut lookat_vector = forward_axis.to_vector_from_rotation_matrix(&frame_orientation_rot_mat);
            let forward_cast_position = frame_position + 1000.0 * &lookat_vector;
            let proj_res = pt_dis_to_line_seg_dvecs(&vector3_to_dvec(lookat_target), &vector3_to_dvec(frame_position), &vector3_to_dvec(&forward_cast_position));

            error_sum += proj_res.0;
        }

        return Ok(error_sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkOrientationGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            // println!("{:?}, {:?}, {:?}", x_h, f_h, robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 1.0, 2) )
    }
}
impl LynxVarsUser for RobotLinkLookat {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotSalientLinkDescription", "robot_salient_link_description"),
                     ("LinkLookAtSpecificationVec", "link_lookat_specifications"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")];
    }
}

#[derive(Clone)]
pub struct RobotLinkLookatAnotherLink;
impl IsolatedObjectiveTerm for RobotLinkLookatAnotherLink {
    fn name(&self) -> String {return "robot_link_lookat".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let l = x.len();
            for i in 0..l {
                if x[i].is_nan() { return Err("NaN was in x, could not continue optimizing".to_string()); }
            }

            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }


        type LinkLookAtAnotherLinkSpecificationVec = Vec<LinkLookAtAnotherLinkSpecification>;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let link_lookat_another_link_specifications = get_lynx_var_ref!(lynx_vars, LinkLookAtAnotherLinkSpecificationVec, "link_lookat_another_link_specifications")?;

        let mut error_sum = 0.0;

        let l = link_lookat_another_link_specifications.len();
        for i in 0..l {
            let link_idx = robot_salient_link_description.get_salient_links()[link_lookat_another_link_specifications[i].salient_link_idx].link_idx;
            let forward_axis_option = &robot_salient_link_description.get_salient_links()[link_lookat_another_link_specifications[i].salient_link_idx].link_local_forward_axis;
            if forward_axis_option.is_none() {
                return Err(format!("link_local_forward_axis must be set in RobotLinkLookatAnotherLink"));
            }
            let forward_axis = forward_axis_option.as_ref().unwrap();
            let lookat_link_idx = robot_salient_link_description.get_salient_links()[link_lookat_another_link_specifications[i].lookat_salient_link_idx].link_idx;

            let frame_option = &fk_res.get_link_frames_ref()[link_idx];
            if frame_option.is_none() {
                return Err("Salient link idx is None in RobotLinkLookat".to_string());
            }

            let frame_lookat_option = &fk_res.get_link_frames_ref()[lookat_link_idx];
            if frame_lookat_option.is_none() {
                return Err("Salient link idx is None in RobotLinkLookat".to_string());
            }

            let frame_unwrap = frame_option.as_ref().unwrap();
            let frame_lookat_unwrap = frame_lookat_option.as_ref().unwrap();

            let frame_position = &frame_unwrap.translation;
            let frame_orientation_quat = &frame_unwrap.quat;
            let frame_orientation_rot_mat = frame_orientation_quat.to_rotation_matrix();
            let mut lookat_vector = forward_axis.to_vector_from_rotation_matrix(&frame_orientation_rot_mat);
            let forward_cast_position = frame_position + 1000.0 * &lookat_vector;
            let lookat_target = &frame_lookat_unwrap.translation;
            let proj_res = pt_dis_to_line_seg_dvecs(&vector3_to_dvec(lookat_target), &vector3_to_dvec(frame_position), &vector3_to_dvec(&forward_cast_position));

            error_sum += proj_res.0;
        }

        return Ok(error_sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkOrientationGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            // println!("{:?}, {:?}, {:?}", x_h, f_h, robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 1.0, 2) )
    }
}
impl LynxVarsUser for RobotLinkLookatAnotherLink {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotSalientLinkDescription", "robot_salient_link_description"),
                     ("LinkLookAtAnotherLinkSpecificationVec", "link_lookat_another_link_specifications"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")];
    }
}

#[derive(Clone)]
pub struct RobotLinkUpright;
impl IsolatedObjectiveTerm for RobotLinkUpright {
    fn name(&self) -> String {return "robot_link_upright".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkUpright objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let l = x.len();
            for i in 0..l {
                if x[i].is_nan() { return Err("NaN was in x, could not continue optimizing".to_string()); }
            }

            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }


        type LinkUprightSpecificationVec = Vec<LinkUprightSpecification>;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let link_upright_specifications = get_lynx_var_ref!(lynx_vars, LinkUprightSpecificationVec, "link_upright_specifications")?;

        let mut error_sum = 0.0;

        let l = link_upright_specifications.len();
        for i in 0..l {
            let link_idx = robot_salient_link_description.get_salient_links()[link_upright_specifications[i].salient_link_idx].link_idx;
            let left_axis_option = &robot_salient_link_description.get_salient_links()[link_upright_specifications[i].salient_link_idx].link_local_left_axis;
            if left_axis_option.is_none() {
                return Err(format!("link_local_left_axis must be set in RobotLinkUpright"));
            }
            let left_axis = left_axis_option.as_ref().unwrap();

            let frame_option = &fk_res.get_link_frames_ref()[link_idx];
            if frame_option.is_none() {
                return Err("Salient link idx is None in RobotLinkUpright".to_string());
            }

            let frame_unwrap = frame_option.as_ref().unwrap();

            let frame_orientation_quat = &frame_unwrap.quat;
            let frame_orientation_rot_mat = frame_orientation_quat.to_rotation_matrix();
            let left_axis_vector = left_axis.to_vector_from_rotation_matrix(&frame_orientation_rot_mat);
            let up_vector = Vector3::new(0.,0.,1.);

            error_sum += left_axis_vector.dot(&up_vector);
        }

        return Ok(error_sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkOrientationGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            // println!("{:?}, {:?}, {:?}", x_h, f_h, robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 1.0, 2) )
    }
}
impl LynxVarsUser for RobotLinkUpright {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotSalientLinkDescription", "robot_salient_link_description"),
                     ("LinkUprightSpecificationVec", "link_upright_specifications"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")];
    }
}


#[derive(Clone)]
pub struct RobotTwoLinkMatchGivenDistance;
impl IsolatedObjectiveTerm for RobotTwoLinkMatchGivenDistance {
    fn name(&self) -> String {return "robot_two_link_match_given_distance".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkUpright objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let l = x.len();
            for i in 0..l {
                if x[i].is_nan() { return Err("NaN was in x, could not continue optimizing".to_string()); }
            }

            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }


        type TwoLinkMatchGivenDistanceSpecificationVec = Vec<TwoLinkMatchGivenDistanceSpecification>;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let two_link_match_given_distance_specifications = get_lynx_var_ref!(lynx_vars, TwoLinkMatchGivenDistanceSpecificationVec, "two_link_match_given_distance_specifications")?;

        let mut error_sum = 0.0;

        let l = two_link_match_given_distance_specifications.len();
        for i in 0..l {
            if robot_salient_link_description.get_salient_links().len() <= two_link_match_given_distance_specifications[i].salient_link_idx1 {
                return Err(format!("salient link idx {:?} is too high for the number of salient links ({:?})", two_link_match_given_distance_specifications[i].salient_link_idx1, robot_salient_link_description.get_salient_links().len()));
            }

            if robot_salient_link_description.get_salient_links().len() <= two_link_match_given_distance_specifications[i].salient_link_idx2 {
                return Err(format!("salient link idx {:?} is too high for the number of salient links ({:?})", two_link_match_given_distance_specifications[i].salient_link_idx1, robot_salient_link_description.get_salient_links().len()));
            }

            let link1_idx = robot_salient_link_description.get_salient_links()[two_link_match_given_distance_specifications[i].salient_link_idx1].link_idx;
            let link2_idx = robot_salient_link_description.get_salient_links()[two_link_match_given_distance_specifications[i].salient_link_idx2].link_idx;

            let frame1_option = &fk_res.get_link_frames_ref()[link1_idx];
            if frame1_option.is_none() {
                return Err("Salient link idx is None in RobotTwoLinkMatchGivenDistance".to_string());
            }

            let frame2_option = &fk_res.get_link_frames_ref()[link2_idx];
            if frame2_option.is_none() {
                return Err("Salient link idx is None in RobotTwoLinkMatchGivenDistance".to_string());
            }

            let frame1_unwrap = frame1_option.as_ref().unwrap();
            let frame2_unwrap = frame2_option.as_ref().unwrap();

            let dis = (&frame1_unwrap.translation - &frame2_unwrap.translation).norm();
            let goal_dis = two_link_match_given_distance_specifications[i].distance;

            error_sum += (dis - goal_dis).powi(2);

        }

        return Ok(error_sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkOrientationGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            // println!("{:?}, {:?}, {:?}", x_h, f_h, robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 1.0, 2) )
    }
}
impl LynxVarsUser for RobotTwoLinkMatchGivenDistance {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotSalientLinkDescription", "robot_salient_link_description"),
                     ("TwoLinkMatchGivenDistanceSpecificationVec", "two_link_match_given_distance_specifications"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")];
    }
}


#[derive(Clone)]
pub struct RobotTwoLinkMatchOrientation;
impl IsolatedObjectiveTerm for RobotTwoLinkMatchOrientation {
    fn name(&self) -> String {return "robot_two_link_match_orientation".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkUpright objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let l = x.len();
            for i in 0..l {
                if x[i].is_nan() { return Err("NaN was in x, could not continue optimizing".to_string()); }
            }

            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }


        type TwoLinkMatchOrientationSpecificationVec = Vec<TwoLinkMatchOrientationSpecification>;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let two_link_match_orientation_specifications = get_lynx_var_ref!(lynx_vars, TwoLinkMatchOrientationSpecificationVec, "two_link_match_orientation_specifications")?;

        let mut error_sum = 0.0;

        let l = two_link_match_orientation_specifications.len();
        for i in 0..l {
            if robot_salient_link_description.get_salient_links().len() <= two_link_match_orientation_specifications[i].salient_link_idx1 {
                return Err(format!("salient link idx {:?} is too high for the number of salient links ({:?})", two_link_match_orientation_specifications[i].salient_link_idx1, robot_salient_link_description.get_salient_links().len()));
            }

            if robot_salient_link_description.get_salient_links().len() <= two_link_match_orientation_specifications[i].salient_link_idx2 {
                return Err(format!("salient link idx {:?} is too high for the number of salient links ({:?})", two_link_match_orientation_specifications[i].salient_link_idx1, robot_salient_link_description.get_salient_links().len()));
            }

            let link1_idx = robot_salient_link_description.get_salient_links()[two_link_match_orientation_specifications[i].salient_link_idx1].link_idx;
            let link2_idx = robot_salient_link_description.get_salient_links()[two_link_match_orientation_specifications[i].salient_link_idx2].link_idx;

            let frame1_option = &fk_res.get_link_frames_ref()[link1_idx];
            if frame1_option.is_none() {
                return Err("Salient link idx is None in RobotTwoLinkMatchGivenDistance".to_string());
            }

            let frame2_option = &fk_res.get_link_frames_ref()[link2_idx];
            if frame2_option.is_none() {
                return Err("Salient link idx is None in RobotTwoLinkMatchGivenDistance".to_string());
            }

            let frame1_unwrap = frame1_option.as_ref().unwrap();
            let frame2_unwrap = frame2_option.as_ref().unwrap();


            error_sum += quaternion_disp(frame1_unwrap.quat.clone(), frame2_unwrap.quat.clone()).norm();

        }

        return Ok(error_sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkOrientationGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            // println!("{:?}, {:?}, {:?}", x_h, f_h, robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 1.0, 2) )
    }
}
impl LynxVarsUser for RobotTwoLinkMatchOrientation {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotSalientLinkDescription", "robot_salient_link_description"),
                     ("TwoLinkMatchOrientationSpecificationVec", "two_link_match_orientation_specifications"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")];
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct PullToCSpaceGoal;
impl IsolatedObjectiveTerm for PullToCSpaceGoal {
    fn name(&self) -> String {return "pull_to_cspace_goal".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        type DVectorF64 = DVector<f64>;
        let cspace_goal = get_lynx_var_ref!(lynx_vars, DVectorF64, "cspace_goal")?;
        let lambda = get_lynx_var_ref!(lynx_vars, f64, "lambda")?;
        let previous_states_container = get_lynx_var_ref!(lynx_vars, PreviousStatesContainer, "previous_states_container")?;

        let mut dir = cspace_goal - &previous_states_container.prev_state;
        let mut dir_n = (&dir / dir.norm());
        let incremental_cspace_goal = &previous_states_container.prev_state + (*lambda * dir_n);

        return Ok( (incremental_cspace_goal - x).norm() );
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.05, 2.0, 2) )
    }
}
impl LynxVarsUser for PullToCSpaceGoal {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("DVectorF64", "cspace_goal"),
                     ("f64", "lambda"),
                     ("PreviousStatesContainer", "previous_states_container") ];
    }
}

#[derive(Clone)]
pub struct RobotMinimizeJointVelocity;
impl IsolatedObjectiveTerm for RobotMinimizeJointVelocity {
    fn name(&self) -> String {return "robot_minimize_joint_velocity".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        let previous_states_container = get_lynx_var_ref!(lynx_vars, PreviousStatesContainer, "previous_states_container")?;

        let v = (x - &previous_states_container.prev_state);

        return Ok(v.norm());
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 3.0, 2) )
    }
}
impl LynxVarsUser for RobotMinimizeJointVelocity {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("PreviousStatesContainer", "previous_states_container") ];
    }
}

#[derive(Clone)]
pub struct RobotMinimizeJointAcceleration;
impl IsolatedObjectiveTerm for RobotMinimizeJointAcceleration {
    fn name(&self) -> String {return "robot_minimize_joint_acceleration".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        let previous_states_container = get_lynx_var_ref!(lynx_vars, PreviousStatesContainer, "previous_states_container")?;

        let v1 = (x - &previous_states_container.prev_state);
        let v2 = (&previous_states_container.prev_state - &previous_states_container.prev_state2);

        let a = (&v1 - &v2);

        return Ok(a.norm());
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 3.0, 2) )
    }
}
impl LynxVarsUser for RobotMinimizeJointAcceleration {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("PreviousStatesContainer", "previous_states_container") ];
    }
}

#[derive(Clone)]
pub struct RobotMinimizeJointJerk;
impl IsolatedObjectiveTerm for RobotMinimizeJointJerk {
    fn name(&self) -> String {return "robot_minimize_joint_jerk".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        let previous_states_container = get_lynx_var_ref!(lynx_vars, PreviousStatesContainer, "previous_states_container")?;

        let v1 = (x - &previous_states_container.prev_state);
        let v2 = (&previous_states_container.prev_state - &previous_states_container.prev_state2);
        let v3 = (&previous_states_container.prev_state2 - &previous_states_container.prev_state3);

        let a1 = (&v1 - &v2);
        let a2 = (&v2 - &v3);

        let j = &a1 - &a2;

        return Ok(j.norm());
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 3.0, 2) )
    }
}
impl LynxVarsUser for RobotMinimizeJointJerk {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("PreviousStatesContainer", "previous_states_container") ];
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct RobotMinimizeEEAcceleration;
impl IsolatedObjectiveTerm for RobotMinimizeEEAcceleration {
    fn name(&self) -> String {return "robot_minimize_ee_acceleration".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotMinimizeEEAcceleration objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let previous_states_container = get_lynx_var_ref!(lynx_vars, PreviousStatesContainer, "previous_states_container")?;

        let frames = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
        let frames_prev = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(&previous_states_container.prev_state)?;
        let frames_prev2 = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(&previous_states_container.prev_state2)?;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let ee_link_idx = robot_salient_link_description.get_salient_links()[0].link_idx;

        let idq_option = frames.get_link_frames_ref()[ee_link_idx].as_ref().unwrap();
        let idq_prev_option = frames_prev.get_link_frames_ref()[ee_link_idx].as_ref().unwrap();
        let idq_prev2_option = frames_prev2.get_link_frames_ref()[ee_link_idx].as_ref().unwrap();

        let pos = &idq_option.translation;
        let pos_prev = &idq_prev_option.translation;
        let pos_prev2 = &idq_prev2_option.translation;

        let v1 = (pos - pos_prev);
        let v2 = (pos_prev - pos_prev2);
        let a = (&v1 - &v2);

        return Ok(a.norm());
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 3.0, 2) )
    }
}
impl LynxVarsUser for RobotMinimizeEEAcceleration {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("PreviousStatesContainer", "previous_states_container"),
                     ("RobotSalientLinkDescription", "robot_salient_link_description") ];
    }
}

#[derive(Clone)]
pub struct RobotMinimizeEEJerk;
impl IsolatedObjectiveTerm for RobotMinimizeEEJerk {
    fn name(&self) -> String {return "robot_minimize_ee_jerk".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotMinimizeEEJerk objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let previous_states_container = get_lynx_var_ref!(lynx_vars, PreviousStatesContainer, "previous_states_container")?;

        let frames = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
        let frames_prev = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(&previous_states_container.prev_state)?;
        let frames_prev2 = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(&previous_states_container.prev_state2)?;
        let frames_prev3 = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(&previous_states_container.prev_state3)?;

        let robot_salient_link_description = get_lynx_var_ref!(lynx_vars, RobotSalientLinkDescription, "robot_salient_link_description")?;

        let ee_link_idx = robot_salient_link_description.get_salient_links()[0].link_idx;

        let idq_option = frames.get_link_frames_ref()[ee_link_idx].as_ref().unwrap();
        let idq_prev_option = frames_prev.get_link_frames_ref()[ee_link_idx].as_ref().unwrap();
        let idq_prev2_option = frames_prev2.get_link_frames_ref()[ee_link_idx].as_ref().unwrap();
        let idq_prev3_option = frames_prev3.get_link_frames_ref()[ee_link_idx].as_ref().unwrap();

        let pos = &idq_option.translation;
        let pos_prev = &idq_prev_option.translation;
        let pos_prev2 = &idq_prev2_option.translation;
        let pos_prev3 = &idq_prev3_option.translation;

        let v1 = (pos - pos_prev);
        let v2 = (pos_prev - pos_prev2);
        let v3 = (pos_prev2 - pos_prev3);

        let a1 = (&v1 - &v2);
        let a2 = (&v2 - &v3);

        let j = &a1 - &a2;

        return Ok(j.norm());
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.1, 3.0, 2) )
    }
}
impl LynxVarsUser for RobotMinimizeEEJerk {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("PreviousStatesContainer", "previous_states_container"),
                     ("RobotSalientLinkDescription", "robot_salient_link_description") ];
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
#[derive(Clone)]
pub struct RobotSelfCollisionAvoidance;
impl IsolatedObjectiveTerm for RobotSelfCollisionAvoidance {
    fn name(&self) -> String {return "robot_self_collision_avoidance".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotSelfCollisionAvoidance objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        type DVectorF64 = DVector<f64>;
        let initial_condition = get_lynx_var_ref!(lynx_vars, DVectorF64, "initial_condition")?.clone();
        let fk_res_from_initial_condition = get_lynx_var_ref!(lynx_vars, RobotFKResult, "fk_res_from_initial_condition")?;
        if !(fk_res_from_initial_condition.get_x_ref() == &initial_condition) {
            let new_fk_res_from_contact_calculation = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(&initial_condition)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "fk_res_from_initial_condition", new_fk_res_from_contact_calculation);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let self_collision_contact_state = get_lynx_var_ref!(lynx_vars, DVectorF64, "self_collision_contact_state")?;
        if !(self_collision_contact_state == &initial_condition) {
            let self_collision_link_geometry_type = get_lynx_var_ref!(lynx_vars, LinkGeometryType, "self_collision_link_geometry_type")?;
            let new_contact_calculation = robot_module_toolbox_unwrap.get_core_collision_module_mut_ref().self_contact_check(&fk_res_from_initial_condition, self_collision_link_geometry_type.clone(), false, Some(0.5))?;
            set_lynx_var!(lynx_vars, ContactCheckMultipleResult, "self_collision_contact_check_multiple_result", new_contact_calculation);
            set_lynx_var!(lynx_vars, DVectorF64, "self_collision_contact_state", initial_condition.clone());
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let self_collision_contact_check_multiple_result = get_lynx_var_ref!(lynx_vars, ContactCheckMultipleResult, "self_collision_contact_check_multiple_result")?;

        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let mut sum = 0.0;

        match self_collision_contact_check_multiple_result {
            ContactCheckMultipleResult::NoIntersectionsFound(info) => {
                let l = info.get_contact_check_contacts().len();
                for i in 0..l {
                    let depth_compared_to_average = info.get_contact_check_depths_with_respect_to_average()[i];

                    if depth_compared_to_average > 0.9 { continue; }
                    let world1 = &info.get_contact_check_contacts()[i].world1;
                    let world1_vec = Vector3::new(world1[0], world1[1], world1[2]);
                    let world2 = &info.get_contact_check_contacts()[i].world2;
                    let world2_vec = Vector3::new(world2[0], world2[1], world2[2]);
                    let normal = &info.get_contact_check_contacts()[i].normal;
                    let normal_vec = Vector3::new(normal[0], normal[1], normal[2]);
                    let link1_idx = info.get_contact_check_idxs()[i][0][0];
                    let link2_idx = info.get_contact_check_idxs()[i][1][0];

                    let multiplier = 0.5 * (1.0 / (1.0 + (4.0 * depth_compared_to_average).exp()));

                    let world1_goal = &world1_vec + multiplier * -&normal_vec;
                    let world2_goal = &world2_vec + multiplier * &normal_vec;

                    if fk_res.get_link_frames_ref()[link1_idx].is_none() {
                        return Err(format!("link idx {:?} is none in RobotSelfCollisionAvoidance objective", link1_idx));
                    }

                    if fk_res.get_link_frames_ref()[link2_idx].is_none() {
                        return Err(format!("link idx {:?} is none in RobotSelfCollisionAvoidance objective", link2_idx));
                    }

                    let world1_transformed_vec = implicit_dual_quaternion_vector3_displacement_transform(&fk_res_from_initial_condition.get_link_frames_ref()[link1_idx].as_ref().unwrap(), &fk_res.get_link_frames_ref()[link1_idx].as_ref().unwrap(), &world1_vec);
                    let world2_transformed_vec = implicit_dual_quaternion_vector3_displacement_transform(&fk_res_from_initial_condition.get_link_frames_ref()[link2_idx].as_ref().unwrap(), &fk_res.get_link_frames_ref()[link2_idx].as_ref().unwrap(), &world2_vec);

                    let dis1 = (&world1_transformed_vec - &world1_goal).norm();
                    let dis2 = (&world2_transformed_vec - &world2_goal).norm();

                    sum += (dis1 + dis2);
                    /*
                    println!("x: {:?}", x);
                    println!("link idx 1: {:?}", link1_idx);
                    println!("link idx 2: {:?}", link2_idx);
                    println!("world1: {:?}", world1);
                    println!("world2: {:?}", world2);
                    println!("world1 transformed: {:?}", world1_transformed_vec);
                    println!("world2 transformed: {:?}", world2_transformed_vec);
                    println!("world1 goal: {:?}", world1_goal);
                    println!("world2 goal: {:?}", world2_goal);
                    println!("fk_res_from_contact_calculation link 1: {:?}", fk_res_from_contact_calculation.get_link_frames_ref()[link1_idx].as_ref());
                    println!("fk_res link 1: {:?}", fk_res.get_link_frames_ref()[link1_idx].as_ref());
                    println!("fk_res_from_contact_calculation link 2: {:?}", fk_res_from_contact_calculation.get_link_frames_ref()[link2_idx].as_ref());
                    println!("fk_res link 2: {:?}", fk_res.get_link_frames_ref()[link2_idx].as_ref());
                    println!("normal: {:?}", normal);
                    println!("depth_compared_to_average: {:?}", depth_compared_to_average);
                    println!("multiplier: {:?}", multiplier);
                    println!("dis1: {:?}", dis1);
                    println!("dis2: {:?}", dis2);
                    println!("sum: {:?}", sum);
                    println!();
                    */

                }
            },
            ContactCheckMultipleResult::IntersectionFound(info) => {
                let l = info.get_contact_check_contacts().len();
                for i in 0..l {
                    let depth_compared_to_average = info.get_contact_check_depths_with_respect_to_average()[i];

                    if depth_compared_to_average > 0.9 { continue; }
                    let world1 = &info.get_contact_check_contacts()[i].world1;
                    let world1_vec = Vector3::new(world1[0], world1[1], world1[2]);
                    let world2 = &info.get_contact_check_contacts()[i].world2;
                    let world2_vec = Vector3::new(world2[0], world2[1], world2[2]);
                    let normal = &info.get_contact_check_contacts()[i].normal;
                    let normal_vec = Vector3::new(normal[0], normal[1], normal[2]);
                    let link1_idx = info.get_contact_check_idxs()[i][0][0];
                    let link2_idx = info.get_contact_check_idxs()[i][1][0];

                    let multiplier = 0.5 * (1.0 / (1.0 + (4.0 * depth_compared_to_average).exp()));

                    let world1_goal = &world1_vec + multiplier * -&normal_vec;
                    let world2_goal = &world2_vec + multiplier * &normal_vec;

                    if fk_res.get_link_frames_ref()[link1_idx].is_none() {
                        return Err(format!("link idx {:?} is none in RobotSelfCollisionAvoidance objective", link1_idx));
                    }

                    if fk_res.get_link_frames_ref()[link2_idx].is_none() {
                        return Err(format!("link idx {:?} is none in RobotSelfCollisionAvoidance objective", link2_idx));
                    }

                    let world1_transformed_vec = implicit_dual_quaternion_vector3_displacement_transform(&fk_res_from_initial_condition.get_link_frames_ref()[link1_idx].as_ref().unwrap(), &fk_res.get_link_frames_ref()[link1_idx].as_ref().unwrap(), &world1_vec);
                    let world2_transformed_vec = implicit_dual_quaternion_vector3_displacement_transform(&fk_res_from_initial_condition.get_link_frames_ref()[link2_idx].as_ref().unwrap(), &fk_res.get_link_frames_ref()[link2_idx].as_ref().unwrap(), &world2_vec);

                    let dis1 = (&world1_transformed_vec - &world1_goal).norm();
                    let dis2 = (&world2_transformed_vec - &world2_goal).norm();

                    sum += (dis1 + dis2);
                    /*
                    println!("x: {:?}", x);
                    println!("link idx 1: {:?}", link1_idx);
                    println!("link idx 2: {:?}", link2_idx);
                    println!("world1: {:?}", world1);
                    println!("world2: {:?}", world2);
                    println!("world1 transformed: {:?}", world1_transformed_vec);
                    println!("world2 transformed: {:?}", world2_transformed_vec);
                    println!("world1 goal: {:?}", world1_goal);
                    println!("world2 goal: {:?}", world2_goal);
                    println!("fk_res_from_contact_calculation link 1: {:?}", fk_res_from_contact_calculation.get_link_frames_ref()[link1_idx].as_ref());
                    println!("fk_res link 1: {:?}", fk_res.get_link_frames_ref()[link1_idx].as_ref());
                    println!("fk_res_from_contact_calculation link 2: {:?}", fk_res_from_contact_calculation.get_link_frames_ref()[link2_idx].as_ref());
                    println!("fk_res link 2: {:?}", fk_res.get_link_frames_ref()[link2_idx].as_ref());
                    println!("normal: {:?}", normal);
                    println!("depth_compared_to_average: {:?}", depth_compared_to_average);
                    println!("multiplier: {:?}", multiplier);
                    println!("dis1: {:?}", dis1);
                    println!("dis2: {:?}", dis2);
                    println!("sum: {:?}", sum);
                    println!();
                    */
                }
            }
        }


        Ok(sum)
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
}
impl LynxVarsUser for RobotSelfCollisionAvoidance {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("DVectorF64", "initial_condition"),
                     ("DVectorF64", "self_collision_contact_state"),
                     ("RobotFKResult", "fk_res_from_initial_condition"),
                     ("LinkGeometryType", "self_collision_link_geometry_type"),
                     ("ContactCheckMultipleResult", "self_collision_contact_check_multiple_result"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result")  ];
    }
}

#[derive(Clone)]
pub struct RobotEnvironmentCollisionAvoidance;
impl IsolatedObjectiveTerm for RobotEnvironmentCollisionAvoidance {
    fn name(&self) -> String {return "robot_environment_collision_avoidance".to_string()}
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotEnvironmentCollisionAvoidance objective".to_string());
        }

        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        type CollisionEnvironmentOption = Option<CollisionEnvironment>;
        let collision_environment_option = get_lynx_var_ref!(lynx_vars, CollisionEnvironmentOption, "collision_environment_option")?;
        if collision_environment_option.is_none() { return Ok(0.0); }

        let collision_environment_unwrap = collision_environment_option.as_ref().unwrap();

        type DVectorF64 = DVector<f64>;
        let initial_condition = get_lynx_var_ref!(lynx_vars, DVectorF64, "initial_condition")?.clone();
        let fk_res_from_initial_condition = get_lynx_var_ref!(lynx_vars, RobotFKResult, "fk_res_from_initial_condition")?;
        if !(fk_res_from_initial_condition.get_x_ref() == &initial_condition) {
            let new_fk_res_from_contact_calculation = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(&initial_condition)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "fk_res_from_initial_condition", new_fk_res_from_contact_calculation);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let environment_collision_contact_state = get_lynx_var_ref!(lynx_vars, DVectorF64, "environment_collision_contact_state")?;
        if !(environment_collision_contact_state == &initial_condition) {
            let environment_collision_link_geometry_type = get_lynx_var_ref!(lynx_vars, LinkGeometryType, "environment_collision_link_geometry_type")?;
            let new_contact_calculation = robot_module_toolbox_unwrap.get_core_collision_module_mut_ref().environment_contact_check(&fk_res_from_initial_condition, environment_collision_link_geometry_type.clone(), collision_environment_unwrap, false, Some(0.5))?;
            set_lynx_var!(lynx_vars, ContactCheckMultipleResult, "environment_collision_contact_check_multiple_result", new_contact_calculation);
            set_lynx_var!(lynx_vars, DVectorF64, "environment_collision_contact_state", initial_condition.clone());
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let environment_collision_contact_check_multiple_result = get_lynx_var_ref!(lynx_vars, ContactCheckMultipleResult, "environment_collision_contact_check_multiple_result")?;

        // environment_collision_contact_check_multiple_result.print_summary();

        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        let mut sum = 0.0;

        // environment_collision_contact_check_multiple_result.print_summary();
        // println!("{:?}", environment_collision_contact_check_multiple_result.);

        match environment_collision_contact_check_multiple_result {
            ContactCheckMultipleResult::NoIntersectionsFound(info) => {
                let l = info.get_contact_check_contacts().len();
                for i in 0..l {
                    let depth_objective_value = info.get_contact_check_depths_with_respect_to_average()[i];

                    let world2 = &info.get_contact_check_contacts()[i].world2;
                    let world2_vec = Vector3::new(world2[0], world2[1], world2[2]);
                    let normal = &info.get_contact_check_contacts()[i].normal;
                    let normal_vec = Vector3::new(normal[0], normal[1], normal[2]);
                    let link_idx = info.get_contact_check_idxs()[i][1][0];

                    let multiplier = 0.4 * (1.0 / (1.0 + (4.0 * depth_objective_value).exp()));

                    let world2_goal = &world2_vec + multiplier * &normal_vec;

                    if fk_res.get_link_frames_ref()[link_idx].is_none() {
                        return Err(format!("link idx {:?} is none in RobotEnvironmentCollisionAvoidance objective", link_idx));
                    }

                    let world2_transformed_vec = implicit_dual_quaternion_vector3_displacement_transform(&fk_res_from_initial_condition.get_link_frames_ref()[link_idx].as_ref().unwrap(), &fk_res.get_link_frames_ref()[link_idx].as_ref().unwrap(), &world2_vec);

                    let dis = (&world2_transformed_vec - &world2_goal).norm();

                    sum += dis;
                    /*
                    println!("x: {:?}", x);
                    println!("link idx 1: {:?}", link1_idx);
                    println!("link idx 2: {:?}", link2_idx);
                    println!("world1: {:?}", world1);
                    println!("world2: {:?}", world2);
                    println!("world1 transformed: {:?}", world1_transformed_vec);
                    println!("world2 transformed: {:?}", world2_transformed_vec);
                    println!("world1 goal: {:?}", world1_goal);
                    println!("world2 goal: {:?}", world2_goal);
                    println!("fk_res_from_contact_calculation link 1: {:?}", fk_res_from_contact_calculation.get_link_frames_ref()[link1_idx].as_ref());
                    println!("fk_res link 1: {:?}", fk_res.get_link_frames_ref()[link1_idx].as_ref());
                    println!("fk_res_from_contact_calculation link 2: {:?}", fk_res_from_contact_calculation.get_link_frames_ref()[link2_idx].as_ref());
                    println!("fk_res link 2: {:?}", fk_res.get_link_frames_ref()[link2_idx].as_ref());
                    println!("normal: {:?}", normal);
                    println!("depth_compared_to_average: {:?}", depth_compared_to_average);
                    println!("multiplier: {:?}", multiplier);
                    println!("dis1: {:?}", dis1);
                    println!("dis2: {:?}", dis2);
                    println!("sum: {:?}", sum);
                    println!();
                    */

                }
            },
            ContactCheckMultipleResult::IntersectionFound(info) => {
                let l = info.get_contact_check_contacts().len();
                for i in 0..l {
                    let depth_objective_value = info.get_contact_check_depths_with_respect_to_average()[i];

                    let world2 = &info.get_contact_check_contacts()[i].world2;
                    let world2_vec = Vector3::new(world2[0], world2[1], world2[2]);
                    let normal = &info.get_contact_check_contacts()[i].normal;
                    let normal_vec = Vector3::new(normal[0], normal[1], normal[2]);
                    let link_idx = info.get_contact_check_idxs()[i][1][0];

                    let multiplier = 0.5 * (1.0 / (1.0 + (4.0 * depth_objective_value).exp()));

                    let world2_goal = &world2_vec + multiplier * &normal_vec;

                    if fk_res.get_link_frames_ref()[link_idx].is_none() {
                        return Err(format!("link idx {:?} is none in RobotEnvironmentCollisionAvoidance objective", link_idx));
                    }

                    let world2_transformed_vec = implicit_dual_quaternion_vector3_displacement_transform(&fk_res_from_initial_condition.get_link_frames_ref()[link_idx].as_ref().unwrap(), &fk_res.get_link_frames_ref()[link_idx].as_ref().unwrap(), &world2_vec);

                    let dis = (&world2_transformed_vec - &world2_goal).norm();

                    sum += dis;
                    /*
                    println!("x: {:?}", x);
                    println!("link idx 1: {:?}", link1_idx);
                    println!("link idx 2: {:?}", link2_idx);
                    println!("world1: {:?}", world1);
                    println!("world2: {:?}", world2);
                    println!("world1 transformed: {:?}", world1_transformed_vec);
                    println!("world2 transformed: {:?}", world2_transformed_vec);
                    println!("world1 goal: {:?}", world1_goal);
                    println!("world2 goal: {:?}", world2_goal);
                    println!("fk_res_from_contact_calculation link 1: {:?}", fk_res_from_contact_calculation.get_link_frames_ref()[link1_idx].as_ref());
                    println!("fk_res link 1: {:?}", fk_res.get_link_frames_ref()[link1_idx].as_ref());
                    println!("fk_res_from_contact_calculation link 2: {:?}", fk_res_from_contact_calculation.get_link_frames_ref()[link2_idx].as_ref());
                    println!("fk_res link 2: {:?}", fk_res.get_link_frames_ref()[link2_idx].as_ref());
                    println!("normal: {:?}", normal);
                    println!("depth_compared_to_average: {:?}", depth_compared_to_average);
                    println!("multiplier: {:?}", multiplier);
                    println!("dis1: {:?}", dis1);
                    println!("dis2: {:?}", dis2);
                    println!("sum: {:?}", sum);
                    println!();
                    */
                }
            }
        }

        Ok(sum)
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        type CollisionEnvironmentOption = Option<CollisionEnvironment>;
        let collision_environment_option = get_lynx_var_ref!(lynx_vars, CollisionEnvironmentOption, "collision_environment_option")?;
        if collision_environment_option.is_none() { return Ok( DVector::from_element(x.len(), 0.0) ); }

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
}
impl LynxVarsUser for RobotEnvironmentCollisionAvoidance {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("CollisionEnvironmentOption", "collision_environment_option"),
                     ("DVectorF64", "initial_condition"),
                     ("DVectorF64", "environment_collision_contact_state"),
                     ("RobotFKResult", "fk_res_from_initial_condition"),
                     ("LinkGeometryType", "environment_collision_link_geometry_type"),
                     ("ContactCheckMultipleResult", "environment_collision_contact_check_multiple_result"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result") ];
    }
}
*/

#[derive(Clone)]
pub struct RobotSelfCollisionAvoidance;
impl IsolatedObjectiveTerm for RobotSelfCollisionAvoidance {
    fn name(&self) -> String { return "robot_self_collision_avoidance".to_string(); }
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {

        /// load robot toolbox
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotEnvironmentCollisionAvoidance objective".to_string());
        }
        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        /// load fk result
        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        /// load environment contact focus idxs
        type DVectorF64 = DVector<f64>;
        let state_where_self_contact_focus_idxs_were_last_computed = get_lynx_var_ref!(lynx_vars, DVectorF64, "state_where_self_contact_focus_idxs_were_last_computed")?;
        let norm_threshold_to_recalculate_new_self_contact_focus_idxs = get_lynx_var_ref!(lynx_vars, f64, "norm_threshold_to_recalculate_new_self_contact_focus_idxs")?;
        let contact_margin = get_lynx_var_ref!(lynx_vars, f64, "contact_margin")?;
        let max_number_of_self_contact_focus_idxs = get_lynx_var_ref!(lynx_vars, usize, "max_number_of_self_contact_focus_idxs")?;
        let self_collision_link_geometry_type = get_lynx_var_ref!(lynx_vars, LinkGeometryType, "self_collision_link_geometry_type")?;
        if state_where_self_contact_focus_idxs_were_last_computed.len() != x.len() || (x - state_where_self_contact_focus_idxs_were_last_computed).norm() > *norm_threshold_to_recalculate_new_self_contact_focus_idxs {
            let contact_res = robot_module_toolbox_unwrap.get_core_collision_module_mut_ref().self_contact_check(&fk_res, self_collision_link_geometry_type.clone(), false, Some(*contact_margin))?;
            let self_contact_focus_idxs = contact_res.get_closest_n_contact_idxs(*max_number_of_self_contact_focus_idxs);

            set_lynx_var!(lynx_vars, DVectorF64, "state_where_self_contact_focus_idxs_were_last_computed", x.clone())?;
            type UsizePairPairVec = Vec< [[usize; 2]; 2] >;
            set_lynx_var!(lynx_vars, UsizePairPairVec, "self_contact_focus_idxs", self_contact_focus_idxs)?;

            return self.call(x, lynx_vars, robot_module_toolbox);
        }
        type UsizePairPairVec = Vec< [[usize; 2]; 2] >;
        let self_contact_focus_idxs = get_lynx_var_ref!(lynx_vars, UsizePairPairVec, "self_contact_focus_idxs")?;

        /// compute objective
        let curr_contact_res = robot_module_toolbox_unwrap.get_core_collision_module_mut_ref().self_contact_check_subset(self_contact_focus_idxs, fk_res, self_collision_link_geometry_type.clone(),  false, Some(*contact_margin))?;
        let curr_contact_info = curr_contact_res.get_contact_check_multiple_info_ref();

        let mut sum = 0.0;

        let depth_objective_values = curr_contact_info.get_contact_check_depths_with_respect_to_average();

        let l = depth_objective_values.len();
        for i in 0..l {
            if depth_objective_values[i] >= 0.0 {
                sum += 1.0 / (1.0 + (100.0 * depth_objective_values[i]).exp());
                // sum += 1.0 / (1.0 + (5.0 * depth_objective_values[i]).exp());
            } else {
                sum += -3.0 * depth_objective_values[i] + 0.5;
            }
        }

        return Ok(sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.3, 0.5, 2) )
    }
}
impl LynxVarsUser for RobotSelfCollisionAvoidance {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotFKResult", "robot_fk_result"),
                     ("f64", "norm_threshold_to_recalculate_new_self_contact_focus_idxs"),
                     ("DVectorF64", "state_where_self_contact_focus_idxs_were_last_computed"),
                     ("f64", "contact_margin"),
                     ("usize", "max_number_of_self_contact_focus_idxs"),
                     ("LinkGeometryType", "self_collision_link_geometry_type"),
                     ("UsizePairPairVec", "self_contact_focus_idxs"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result") ];
    }
}

#[derive(Clone)]
pub struct RobotEnvironmentCollisionAvoidance;
impl IsolatedObjectiveTerm for RobotEnvironmentCollisionAvoidance {
    fn name(&self) -> String { return "robot_environment_collision_avoidance".to_string(); }
    fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<f64, String> {

        /// load robot toolbox
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotEnvironmentCollisionAvoidance objective".to_string());
        }
        let mut robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        /// load collision environment
        type CollisionEnvironmentOption = Option<CollisionEnvironment>;
        let collision_environment_option = get_lynx_var_ref!(lynx_vars, CollisionEnvironmentOption, "collision_environment_option")?;
        if collision_environment_option.is_none() { return Ok(0.0); }
        let collision_environment_unwrap = collision_environment_option.as_ref().unwrap();

        /// load fk result
        let fk_res = get_lynx_var_ref!(lynx_vars, RobotFKResult, "robot_fk_result")?;
        if !(fk_res.get_x_ref() == x) {
            let fk_res_new = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk(x)?;
            set_lynx_var!(lynx_vars, RobotFKResult, "robot_fk_result", fk_res_new);
            return self.call(x, lynx_vars, robot_module_toolbox);
        }

        /// load environment contact focus idxs
        type DVectorF64 = DVector<f64>;
        let state_where_contact_focus_idxs_were_last_computed = get_lynx_var_ref!(lynx_vars, DVectorF64, "state_where_environment_contact_focus_idxs_were_last_computed")?;
        let norm_threshold_to_recalculate_new_environment_contact_focus_idxs = get_lynx_var_ref!(lynx_vars, f64, "norm_threshold_to_recalculate_new_environment_contact_focus_idxs")?;
        let contact_margin = get_lynx_var_ref!(lynx_vars, f64, "contact_margin")?;
        let max_number_of_environment_contact_focus_idxs = get_lynx_var_ref!(lynx_vars, usize, "max_number_of_environment_contact_focus_idxs")?;
        let environment_collision_link_geometry_type = get_lynx_var_ref!(lynx_vars, LinkGeometryType, "environment_collision_link_geometry_type")?;
        if state_where_contact_focus_idxs_were_last_computed.len() != x.len() || (x - state_where_contact_focus_idxs_were_last_computed).norm() > *norm_threshold_to_recalculate_new_environment_contact_focus_idxs {
            let contact_res = robot_module_toolbox_unwrap.get_core_collision_module_mut_ref().environment_contact_check(&fk_res, environment_collision_link_geometry_type.clone(), collision_environment_unwrap, false, Some(*contact_margin))?;
            let environment_contact_focus_idxs = contact_res.get_closest_n_contact_idxs(*max_number_of_environment_contact_focus_idxs);

            set_lynx_var!(lynx_vars, DVectorF64, "state_where_environment_contact_focus_idxs_were_last_computed", x.clone())?;
            type UsizePairPairVec = Vec< [[usize; 2]; 2] >;
            set_lynx_var!(lynx_vars, UsizePairPairVec, "environment_contact_focus_idxs", environment_contact_focus_idxs)?;

            return self.call(x, lynx_vars, robot_module_toolbox);
        }
        type UsizePairPairVec = Vec< [[usize; 2]; 2] >;
        let environment_contact_focus_idxs = get_lynx_var_ref!(lynx_vars, UsizePairPairVec, "environment_contact_focus_idxs")?;

        /// compute objective
        let curr_contact_res = robot_module_toolbox_unwrap.get_core_collision_module_mut_ref().environment_contact_check_subset(environment_contact_focus_idxs, fk_res, environment_collision_link_geometry_type.clone(), collision_environment_unwrap, false, Some(*contact_margin))?;
        let curr_contact_info = curr_contact_res.get_contact_check_multiple_info_ref();

        let mut sum = 0.0;

        let depth_objective_values = curr_contact_info.get_contact_check_depths_with_respect_to_average();

        let l = depth_objective_values.len();
        for i in 0..l {
            if depth_objective_values[i] >= 0.0 {
                sum += 1.0 / (1.0 + (100.0 * depth_objective_values[i]).exp());
                // sum += 1.0 / (1.0 + (15.0 * depth_objective_values[i]).exp());
            } else {
                sum += -3.0 * depth_objective_values[i] + 0.5;
            }
        }

        return Ok(sum);
    }
    fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVars, robot_module_toolbox: &mut Option<RobotModuleToolbox>) -> Result<DVector<f64>, String> {
        if robot_module_toolbox.is_none() {
            return Err("robot_module_toolbox must be Some in RobotLinkPoseGoalMatching objective".to_string());
        }

        let robot_module_toolbox_unwrap = robot_module_toolbox.as_mut().unwrap();

        type CollisionEnvironmentOption = Option<CollisionEnvironment>;
        let collision_environment_option = get_lynx_var_ref!(lynx_vars, CollisionEnvironmentOption, "collision_environment_option")?;
        if collision_environment_option.is_none() { return Ok( DVector::from_element(x.len(), 0.0) ); }

        let robot_fk_gradient_perturbations_result = get_lynx_var_ref!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result")?.clone();
        if !(x == robot_fk_gradient_perturbations_result.get_x_ref()) {
            let new_robot_fk_gradient_perturbations_result = robot_module_toolbox_unwrap.get_fk_module_ref().compute_fk_gradient_perturbations(x)?;
            set_lynx_var!(lynx_vars, RobotFKGradientPerturbationsResult, "robot_fk_gradient_perturbations_result", new_robot_fk_gradient_perturbations_result)?;
            return self.gradient(x, lynx_vars, robot_module_toolbox);
        }

        let l = x.len();

        let mut out_gradient = DVector::from_element(l, 0.0);

        let f_0 = self.call(x, lynx_vars, robot_module_toolbox)?;

        let mut x_h = x.clone();

        let perturbation = robot_fk_gradient_perturbations_result.get_perturbation();

        for i in 0..l {
            set_lynx_var!( lynx_vars, RobotFKResult, "robot_fk_result", robot_fk_gradient_perturbations_result.get_perturbation_fk_results_ref()[i].clone());
            x_h[i] += perturbation;
            if i > 0 { x_h[i-1] -= perturbation; }
            let f_h = self.call(&x_h, lynx_vars, robot_module_toolbox)?;
            out_gradient[i] = ( (-f_0 + f_h) / perturbation);
        }

        return Ok(out_gradient);
    }
    fn get_default_loss_function(&self) -> Box<dyn LossFunction> {
        return Box::new( GrooveLoss::new(0.0, 2, 0.3, 0.5, 2) )
    }
}
impl LynxVarsUser for RobotEnvironmentCollisionAvoidance {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("CollisionEnvironmentOption", "collision_environment_option"),
                     ("RobotFKResult", "robot_fk_result"),
                     ("f64", "norm_threshold_to_recalculate_new_environment_contact_focus_idxs"),
                     ("DVectorF64", "state_where_environment_contact_focus_idxs_were_last_computed"),
                     ("f64", "contact_margin"),
                     ("usize", "max_number_of_environment_contact_focus_idxs"),
                     ("LinkGeometryType", "environment_collision_link_geometry_type"),
                     ("UsizePairPairVec", "environment_contact_focus_idxs"),
                     ("RobotFKGradientPerturbationsResult", "robot_fk_gradient_perturbations_result") ];
    }
}

 */