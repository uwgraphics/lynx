use crate::utils::utils_vars::{lynx_vars::LynxVars, lynx_vars_parallel::LynxVarsParallel};
use crate::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use crate::robot_modules::robot_world::RobotWorld;
use rayon::prelude::*;

#[derive(Debug)]
pub enum LynxVarsGeneric<'a> {
    SingleThreaded(LynxVars),
    SingleThreadedMutRef(&'a mut LynxVars),
    Parallel(LynxVarsParallel),
}

impl<'a> LynxVarsGeneric<'a> {
    pub fn new_empty_single_threaded() -> Self {
        return LynxVarsGeneric::SingleThreaded(LynxVars::new_empty());
    }

    pub fn new_empty_parallel(num_threads: Option<usize>) -> Self {
        return LynxVarsGeneric::Parallel(LynxVarsParallel::new_empty(num_threads));
    }

    pub fn new_empty_single_threaded_packaged_with_robot_module_toolbox(robot_name: &str, configuration_name: Option<&str>, mobile_base_bounds_filename: Option<&str>) -> Result<Self, String> {
        let robot_module_toolbox = RobotModuleToolbox::new_lite(robot_name, configuration_name, mobile_base_bounds_filename)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
        set_or_add_lynx_var_generic!(&mut lynx_vars, RobotModuleToolbox, "robot_module_toolbox", robot_module_toolbox);
        return Ok(lynx_vars);
    }

    pub fn new_empty_parallel_packaged_with_robot_module_toolbox(num_threads: Option<usize>, robot_name: &str, configuration_name: Option<&str>, mobile_base_bounds_filename: Option<&str>) -> Result<Self, String> {
        let robot_module_toolbox = RobotModuleToolbox::new_lite(robot_name, configuration_name, mobile_base_bounds_filename)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_parallel(num_threads);
        set_or_add_lynx_var_generic!(&mut lynx_vars, RobotModuleToolbox, "robot_module_toolbox", robot_module_toolbox);
        return Ok(lynx_vars);
    }

    pub fn new_empty_single_threaded_packaged_with_robot_world(robot_name: &str, configuration_name: Option<&str>, mobile_base_bounds_filename: Option<&str>, environment_name: Option<&str>) -> Result<Self, String> {
        let robot_world = RobotWorld::new(robot_name, configuration_name, mobile_base_bounds_filename, environment_name)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
        set_or_add_lynx_var_generic!(&mut lynx_vars, RobotWorld, "robot_world", robot_world);
        return Ok(lynx_vars);
    }

    pub fn new_empty_parallel_packaged_with_robot_world(num_threads: Option<usize>, robot_name: &str, configuration_name: Option<&str>, mobile_base_bounds_filename: Option<&str>, environment_name: Option<&str>) -> Result<Self, String> {
        let robot_world = RobotWorld::new(robot_name, configuration_name, mobile_base_bounds_filename, environment_name)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_parallel(num_threads);
        set_or_add_lynx_var_generic!(&mut lynx_vars, RobotWorld, "robot_world", robot_world);
        return Ok(lynx_vars);
    }

    pub fn new_parallel(lynx_vars: &LynxVars, num_threads: Option<usize>) -> Self {
        return LynxVarsGeneric::Parallel(LynxVarsParallel::new(lynx_vars, num_threads));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn convert_to_parallel(&mut self, num_threads: Option<usize>, in_parallel: bool) -> Result<(), String> {
        let mut _num_threads = num_cpus::get();
        if num_threads.is_some() { _num_threads = num_threads.unwrap(); }
        if _num_threads == 0 { _num_threads = 1; }

        if _num_threads == 1 {
            return Err("requested number of threads was 1.  Did not convert to parallel".to_string());
        }

        match self {
            LynxVarsGeneric::SingleThreaded(l) => {
                let mut lynx_vars_parallel = LynxVarsParallel::new_with_parallel_option(l, Some(_num_threads), in_parallel);
                let mut out_self = LynxVarsGeneric::Parallel(lynx_vars_parallel);
                *self = out_self;
            }
            LynxVarsGeneric::SingleThreadedMutRef(l) => {
                let mut lynx_vars_parallel = LynxVarsParallel::new_with_parallel_option(l, Some(_num_threads), in_parallel);
                let mut out_self = LynxVarsGeneric::Parallel(lynx_vars_parallel);
                *self = out_self;
            }
            LynxVarsGeneric::Parallel(l) => {
                if l.get_num_threads() == _num_threads { return Ok(()); }
                else {
                    let mut lynx_vars = l.get_first_ref();
                    let mut lynx_vars_parallel = LynxVarsParallel::new_with_parallel_option(lynx_vars, Some(_num_threads), in_parallel);
                    let mut out_self = LynxVarsGeneric::Parallel(lynx_vars_parallel);
                    *self = out_self;
                }
            }
        }

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn contains_variable(&self, var_type: &'static str, var_name: &'static str) -> bool {
        match self {
            LynxVarsGeneric::SingleThreaded(v) => v.contains_variable(var_type, var_name),
            LynxVarsGeneric::SingleThreadedMutRef(v) => v.contains_variable(var_type, var_name),
            LynxVarsGeneric::Parallel(v) => v.all_contain_variable(var_type, var_name)
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_robot_module_toolbox_ref(&self, name: Option<&str>) -> Result<&RobotModuleToolbox, String> {
        return match name {
            None => { get_lynx_var_ref_generic!(self, RobotModuleToolbox, "robot_module_toolbox") }
            Some(n) => { get_lynx_var_ref_generic!(self, RobotModuleToolbox, n) }
        }
    }

    pub fn get_robot_module_toolbox_mut_ref(&mut self, name: Option<&str>) -> Result<&mut RobotModuleToolbox, String> {
        return match name {
            None => { get_lynx_var_mut_ref_generic!(self, RobotModuleToolbox, "robot_module_toolbox") }
            Some(n) => { get_lynx_var_mut_ref_generic!(self, RobotModuleToolbox, n) }
        }
    }

    pub fn get_robot_world_ref(&self, name: Option<&str>) -> Result<&RobotWorld, String> {
        return match name {
            None => { get_lynx_var_ref_generic!(self, RobotWorld, "robot_world") }
            Some(n) => { get_lynx_var_ref_generic!(self, RobotWorld, n) }
        }
    }

    pub fn get_robot_world_mut_ref(&mut self, name: Option<&str>) -> Result<&mut RobotWorld, String> {
        return match name {
            None => { get_lynx_var_mut_ref_generic!(self, RobotWorld, "robot_world") }
            Some(n) => { get_lynx_var_mut_ref_generic!(self, RobotWorld, n) }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_num_threads(&self) -> usize {
        return match self {
            LynxVarsGeneric::SingleThreaded(_) => { 1 }
            LynxVarsGeneric::SingleThreadedMutRef(_) => { 1 }
            LynxVarsGeneric::Parallel(l) => { l.get_num_threads() }
        }
    }

    pub fn print(&self) {
        match self {
            LynxVarsGeneric::SingleThreaded(v) => v.print(),
            LynxVarsGeneric::SingleThreadedMutRef(v) => v.print(),
            LynxVarsGeneric::Parallel(v) => v.print()
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
}
impl<'a> Clone for LynxVarsGeneric<'a> {
    fn clone(&self) -> Self {
        match self {
            LynxVarsGeneric::SingleThreaded(v) => return LynxVarsGeneric::SingleThreaded(v.clone()),
            LynxVarsGeneric::SingleThreadedMutRef(v) => return LynxVarsGeneric::SingleThreaded((*v).clone()),
            LynxVarsGeneric::Parallel(v) => return LynxVarsGeneric::Parallel(v.clone())
        }
    }
}