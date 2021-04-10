use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use crate::robot_modules::robot_dof_module::RobotDOFModule;
use crate::utils::utils_files_and_strings::robot_folder_utils::*;
use crate::utils::utils_parsing::yaml_parsing_utils::get_yaml_obj;
use crate::utils::utils_files_and_strings::file_utils::check_if_path_exists;
use nalgebra::{DVector};
use termion::{color, style};
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use crate::robot_modules::robot_bounds_module::BoundsCheckResult::{InBounds, OutOfBounds, Error};
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_vars::prelude::*;

#[derive(Clone)]
pub struct RobotBoundsModule {
    _upper_bounds: Vec<f64>,
    _lower_bounds: Vec<f64>,
    _bounds: Vec<(f64, f64)>,
    _robot_name_copy: String,
    _mobile_base_mode_copy: String,
    _num_dofs: usize
}

impl RobotBoundsModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule, robot_dof_module: &RobotDOFModule, mobile_base_bounds_filename: Option<&str>) -> Self {
        let upper_bounds = Vec::new();
        let lower_bounds = Vec::new();
        let bounds = Vec::new();
        let _robot_name_copy = robot_configuration_module.robot_model_module.robot_name.clone();
        let _mobile_base_mode_copy = robot_configuration_module.mobile_base_mode.clone();
        let _num_dofs = robot_dof_module.get_num_dofs();

        let mut out_self = Self { _upper_bounds: upper_bounds, _lower_bounds: lower_bounds, _bounds: bounds, _robot_name_copy, _mobile_base_mode_copy, _num_dofs };

        out_self._create_mobile_base_bounds_folder_and_default_file_if_need_be(robot_configuration_module);
        out_self._set_bounds(robot_configuration_module, robot_dof_module);
        out_self.set_bounds_for_mobile_base_from_file(mobile_base_bounds_filename);

        return out_self;
    }

    fn _set_bounds(&mut self, robot_configuration_module: &RobotConfigurationModule, robot_dof_module: &RobotDOFModule) {
        let l = robot_dof_module.get_num_dofs();
        for i in 0..l {
            let c = robot_dof_module.get_joint_idx_type_and_subidx_from_input_x_idx(i);
            let has_bounds = robot_configuration_module.robot_model_module.joints[c.0].urdf_joint.includes_limits;
            let mut upper_bound = std::f64::INFINITY;
            let mut lower_bound = -std::f64::INFINITY;

            if has_bounds {
                upper_bound = robot_configuration_module.robot_model_module.joints[c.0].urdf_joint.limits_upper;
                lower_bound = robot_configuration_module.robot_model_module.joints[c.0].urdf_joint.limits_lower;
            }

            self._upper_bounds.push( upper_bound );
            self._lower_bounds.push( lower_bound );
            self._bounds.push( (lower_bound, upper_bound) )
        }
    }

    fn _create_mobile_base_bounds_folder_and_default_file_if_need_be(&self, robot_configuration_module: &RobotConfigurationModule) {
        // let fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/mobile_base_bounds";
        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let partial_fp = "mobile_base_bounds/default.yaml".to_string();
        let res = check_if_path_exists_relative_to_robot_directory(robot_name.clone(), partial_fp.clone());

        if !res {
            println!("{}{}mobile_base_bounds directory has not been created yet.  Creating that now.{}", color::Fg(color::Blue), style::Bold, style::Reset);
            let mut out_str = "# bounds can either be left as empty brackets or can be filled with two floating point values (lower and upper bounds)\n".to_string();
            out_str += "#    e.g., bounds_x: [-1.0, 1.0]\n";
            out_str += "#    e.g., bounds_y: [-2.0, 0.0]  (note that 0.0 is inputted as floating point number with a decimal rather than just 0) \n";
            out_str += "# all rotation bounds should be specified in radians. \n";

            out_str += "bounds_x: [-10., 10.]\n";
            out_str += "bounds_y: [-10., 10.]\n";
            out_str += "bounds_z: [-10., 10.]\n";
            out_str += "bounds_rx: [-3.14, 3.14]\n";
            out_str += "bounds_ry: [-3.14, 3.14]\n";
            out_str += "bounds_rz: [-3.14, 3.14]";

            write_string_to_file_relative_to_robot_directory(robot_name.clone(), "mobile_base_bounds".to_string(), "default.yaml".to_string(), out_str, true);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn check_if_state_is_within_bounds(&self, state: &DVector<f64>) -> BoundsCheckResult {
        let l = state.len();
        if !(l == self._num_dofs) { return Error(format!("given state length ({:?}) and num dofs ({:?}) are not the same", state.len(), self._num_dofs)); }

        for i in 0..l {
            if state[i] > self._upper_bounds[i] || state[i] < self._lower_bounds[i] {
                return OutOfBounds(format!("state idx {:?} is out of bounds with value {:?} (bounds are {:?} - {:?}", i, state[i], self._lower_bounds[i], self._upper_bounds[i]));
            }
        }

        return InBounds;
    }

    pub fn set_bounds_for_mobile_base_manual(&mut self, bounds_x: Option<(f64, f64)>, bounds_y: Option<(f64, f64)>, bounds_z: Option<(f64, f64)>, bounds_rx: Option<(f64, f64)>, bounds_ry: Option<(f64, f64)>, bounds_rz: Option<(f64, f64)>) {
        let mobile_base_mode = self._mobile_base_mode_copy.clone();
        if mobile_base_mode == "static".to_string()  { return; }

        let l = self._bounds.len();

        if mobile_base_mode == "floating".to_string() {
            if bounds_x.is_some() { self._bounds[l - 6] =  (bounds_x.unwrap().clone());  self._lower_bounds[l - 6] = bounds_x.unwrap().0;  self._upper_bounds[l - 6] = bounds_x.unwrap().1;}
            if bounds_y.is_some() { self._bounds[l - 5] =  (bounds_y.unwrap().clone());  self._lower_bounds[l - 5] = bounds_y.unwrap().0;  self._upper_bounds[l - 5] = bounds_y.unwrap().1;}
            if bounds_z.is_some() { self._bounds[l - 4] =  (bounds_z.unwrap().clone());  self._lower_bounds[l - 4] = bounds_z.unwrap().0;  self._upper_bounds[l - 4] = bounds_z.unwrap().1;}
            if bounds_rx.is_some() { self._bounds[l - 3] = (bounds_rx.unwrap().clone()); self._lower_bounds[l - 3] = bounds_rx.unwrap().0; self._upper_bounds[l - 3] = bounds_rx.unwrap().1;}
            if bounds_ry.is_some() { self._bounds[l - 2] = (bounds_ry.unwrap().clone()); self._lower_bounds[l - 2] = bounds_ry.unwrap().0; self._upper_bounds[l - 2] = bounds_ry.unwrap().1;}
            if bounds_rz.is_some() { self._bounds[l - 1] = (bounds_rz.unwrap().clone()); self._lower_bounds[l - 1] = bounds_rz.unwrap().0; self._upper_bounds[l - 1] = bounds_rz.unwrap().1;}
        } else if mobile_base_mode == "translation".to_string() {
            if bounds_x.is_some() { self._bounds[l - 3] =  (bounds_x.unwrap().clone());  self._lower_bounds[l - 3] = bounds_x.unwrap().0;  self._upper_bounds[l - 3] = bounds_x.unwrap().1;}
            if bounds_y.is_some() { self._bounds[l - 2] =  (bounds_y.unwrap().clone());  self._lower_bounds[l - 2] = bounds_y.unwrap().0;  self._upper_bounds[l - 2] = bounds_y.unwrap().1;}
            if bounds_z.is_some() { self._bounds[l - 1] =  (bounds_z.unwrap().clone());  self._lower_bounds[l - 1] = bounds_z.unwrap().0;  self._upper_bounds[l - 1] = bounds_z.unwrap().1;}
        } else if mobile_base_mode == "rotation".to_string() {
            if bounds_rx.is_some() { self._bounds[l - 3] = (bounds_rx.unwrap().clone()); self._lower_bounds[l - 3] = bounds_rx.unwrap().0; self._upper_bounds[l - 3] = bounds_rx.unwrap().1;}
            if bounds_ry.is_some() { self._bounds[l - 2] = (bounds_ry.unwrap().clone()); self._lower_bounds[l - 2] = bounds_ry.unwrap().0; self._upper_bounds[l - 2] = bounds_ry.unwrap().1;}
            if bounds_rz.is_some() { self._bounds[l - 1] = (bounds_rz.unwrap().clone()); self._lower_bounds[l - 1] = bounds_rz.unwrap().0; self._upper_bounds[l - 1] = bounds_rz.unwrap().1;}
        } else if mobile_base_mode == "planar_translation".to_string() {
            if bounds_x.is_some() { self._bounds[l - 2] =  (bounds_x.unwrap().clone());  self._lower_bounds[l - 2] = bounds_x.unwrap().0;  self._upper_bounds[l - 2] = bounds_x.unwrap().1;}
            if bounds_y.is_some() { self._bounds[l - 1] =  (bounds_y.unwrap().clone());  self._lower_bounds[l - 1] = bounds_y.unwrap().0;  self._upper_bounds[l - 1] = bounds_y.unwrap().1;}
        } else if mobile_base_mode == "planar_rotation".to_string() {
            if bounds_rz.is_some() { self._bounds[l - 1] = (bounds_rz.unwrap().clone()); self._lower_bounds[l - 1] = bounds_rz.unwrap().0; self._upper_bounds[l - 1] = bounds_rz.unwrap().1;}
        } else if mobile_base_mode == "planar_translation_and_rotation".to_string() {
            if bounds_x.is_some() { self._bounds[l - 3] =  (bounds_x.unwrap().clone());  self._lower_bounds[l - 3] = bounds_x.unwrap().0;  self._upper_bounds[l - 3] = bounds_x.unwrap().1;}
            if bounds_y.is_some() { self._bounds[l - 2] =  (bounds_y.unwrap().clone());  self._lower_bounds[l - 2] = bounds_y.unwrap().0;  self._upper_bounds[l - 2] = bounds_y.unwrap().1;}
            if bounds_rz.is_some() { self._bounds[l - 1] = (bounds_rz.unwrap().clone()); self._lower_bounds[l - 1] = bounds_rz.unwrap().0; self._upper_bounds[l - 1] = bounds_rz.unwrap().1;}
        }
    }

    pub fn set_bounds_for_mobile_base_from_file(&mut self, mobile_base_bounds_filename: Option<&str>) -> Result<(), String> {
        // do not need to put .yaml at the end of mobile base bounds filename
        let mut mobile_base_bounds_file_ = "default".to_string();
        if mobile_base_bounds_filename.is_some() { mobile_base_bounds_file_ = mobile_base_bounds_filename.unwrap().to_string(); }

        let robot_name = self._robot_name_copy.clone();
        let fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/mobile_base_bounds/" + mobile_base_bounds_file_.as_str() + ".yaml";

        let mut bounds_x = None;
        let mut bounds_y = None;
        let mut bounds_z = None;
        let mut bounds_rx = None;
        let mut bounds_ry = None;
        let mut bounds_rz = None;

        let y = get_yaml_obj(fp.clone())?;
        if y.len() == 0 {
            println!("{}{}WARNING: mobile_base_bounds_filename {}.yaml was not found.  Defaulting to no bounds on mobile base. {}", color::Fg(color::Yellow), style::Bold, mobile_base_bounds_file_, style::Reset);
            return Err(format!("WARNING: mobile_base_bounds_filename {}.yaml was not found.  Defaulting to no bounds on mobile base.", mobile_base_bounds_file_));
        }

        let bounds_x_ = y[0]["bounds_x"].clone();
        if bounds_x_.as_vec().is_some() && bounds_x_.as_vec().unwrap().len() == 2 {
            if bounds_x_.as_vec().unwrap()[0].as_f64().is_some() && bounds_x_.as_vec().unwrap()[1].as_f64().is_some() {
                let mut lower_bound = bounds_x_.as_vec().unwrap()[0].as_f64().unwrap();
                let mut upper_bound = bounds_x_.as_vec().unwrap()[1].as_f64().unwrap();
                bounds_x = Some((lower_bound, upper_bound));
            } else {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_x from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        } else {
            if !(bounds_x_.as_vec().unwrap().len() == 0) {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_x from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        }

        let bounds_y_ = y[0]["bounds_y"].clone();
        if bounds_y_.as_vec().is_some() && bounds_y_.as_vec().unwrap().len() == 2 {
            if bounds_y_.as_vec().unwrap()[0].as_f64().is_some() && bounds_y_.as_vec().unwrap()[1].as_f64().is_some() {
                let mut lower_bound = bounds_y_.as_vec().unwrap()[0].as_f64().unwrap();
                let mut upper_bound = bounds_y_.as_vec().unwrap()[1].as_f64().unwrap();
                bounds_y = Some((lower_bound, upper_bound));
            } else {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_y from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        } else {
            if !(bounds_y_.as_vec().unwrap().len() == 0) {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_y from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        }

        let bounds_z_ = y[0]["bounds_z"].clone();
        if bounds_z_.as_vec().is_some() && bounds_z_.as_vec().unwrap().len() == 2 {
            if bounds_z_.as_vec().unwrap()[0].as_f64().is_some() && bounds_z_.as_vec().unwrap()[1].as_f64().is_some() {
                let mut lower_bound = bounds_z_.as_vec().unwrap()[0].as_f64().unwrap();
                let mut upper_bound = bounds_z_.as_vec().unwrap()[1].as_f64().unwrap();
                bounds_z = Some((lower_bound, upper_bound));
            } else {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_z from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        } else {
            if !(bounds_z_.as_vec().unwrap().len() == 0) {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_z from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        }



        let bounds_rx_ = y[0]["bounds_rx"].clone();
        if bounds_rx_.as_vec().is_some() && bounds_rx_.as_vec().unwrap().len() == 2 {
            if bounds_rx_.as_vec().unwrap()[0].as_f64().is_some() && bounds_rx_.as_vec().unwrap()[1].as_f64().is_some() {
                let mut lower_bound = bounds_rx_.as_vec().unwrap()[0].as_f64().unwrap();
                let mut upper_bound = bounds_rx_.as_vec().unwrap()[1].as_f64().unwrap();
                bounds_rx = Some((lower_bound, upper_bound));
            } else {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_rx from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        } else {
            if !(bounds_rx_.as_vec().unwrap().len() == 0) {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_rx from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        }

        let bounds_ry_ = y[0]["bounds_ry"].clone();
        if bounds_ry_.as_vec().is_some() && bounds_ry_.as_vec().unwrap().len() == 2 {
            if bounds_ry_.as_vec().unwrap()[0].as_f64().is_some() && bounds_ry_.as_vec().unwrap()[1].as_f64().is_some() {
                let mut lower_bound = bounds_ry_.as_vec().unwrap()[0].as_f64().unwrap();
                let mut upper_bound = bounds_ry_.as_vec().unwrap()[1].as_f64().unwrap();
                bounds_ry = Some((lower_bound, upper_bound));
            } else {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_ry from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        } else {
            if !(bounds_ry_.as_vec().unwrap().len() == 0) {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_ry from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        }

        let bounds_rz_ = y[0]["bounds_rz"].clone();
        if bounds_rz_.as_vec().is_some() && bounds_rz_.as_vec().unwrap().len() == 2 {
            if bounds_rz_.as_vec().unwrap()[0].as_f64().is_some() && bounds_rz_.as_vec().unwrap()[1].as_f64().is_some() {
                let mut lower_bound = bounds_rz_.as_vec().unwrap()[0].as_f64().unwrap();
                let mut upper_bound = bounds_rz_.as_vec().unwrap()[1].as_f64().unwrap();
                bounds_rz = Some((lower_bound, upper_bound));
            } else {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_rz from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        } else {
            if !(bounds_rz_.as_vec().unwrap().len() == 0) {
                println!("{}{}WARNING: there seems to have been some problem parsing bounds_rz from file.  Make sure this is a valid list of two floating point values.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            }
        }

        self.set_bounds_for_mobile_base_manual(bounds_x, bounds_y, bounds_z, bounds_rx, bounds_ry, bounds_rz);

        Ok(())
    }

    pub fn uniform_sample_from_bounds(&self) -> DVector<f64> {
        let mut v =  DVector::from_element(self._num_dofs, 0.0);
        let mut rng = rand::thread_rng();
        for i in 0..self._num_dofs {
            v[i] = rng.gen_range(self._lower_bounds[i].max(-100.0), self._upper_bounds[i].min(100.0));
        }
        return v;
    }

    pub fn to_float_vec_sampler_box(&self) -> FloatVecSamplerBox {
        return FloatVecSamplerBox::new(self);
    }

    pub fn to_lynx_float_vec_sampler_box(&self) -> LynxFloatVecSamplerBox {
        return LynxFloatVecSamplerBox::new(self);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_upper_bounds(&self) -> &Vec<f64> {
        return &self._upper_bounds;
    }

    pub fn get_lower_bounds(&self) -> &Vec<f64> {
        return &self._lower_bounds;
    }

    pub fn get_bounds(&self) -> &Vec<(f64, f64)> { return &self._bounds; }

}

impl FloatVecSampler for RobotBoundsModule {
    fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String> {
        return Ok(self.uniform_sample_from_bounds());
    }
}
impl LynxFloatVecSampler for RobotBoundsModule {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        return Ok(self.uniform_sample_from_bounds());
    }
}
impl LynxVarsUser for RobotBoundsModule { }

#[derive(Clone, Debug)]
pub enum BoundsCheckResult {
    InBounds,
    OutOfBounds(String),
    Error(String)
}
