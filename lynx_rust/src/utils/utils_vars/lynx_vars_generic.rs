use crate::utils::utils_vars::{lynx_vars::LynxVars, lynx_vars_parallel::LynxVarsParallel};
use crate::robot_modules::robot::Robot;
use crate::robot_modules::robot_world::RobotWorld;
use crate::robot_modules::prelude::RobotSet;
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

    pub fn new_single_threaded_packaged_with_robot(robot_name: &str, configuration_name: Option<&str>) -> Result<Self, String> {
        let robot = Robot::new(robot_name, configuration_name)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
        set_or_add_lynx_var_generic!(&mut lynx_vars, Robot, "robot", robot);
        return Ok(lynx_vars);
    }

    pub fn new_parallel_packaged_with_robot(num_threads: Option<usize>, robot_name: &str, configuration_name: Option<&str>) -> Result<Self, String> {
        let robot = Robot::new(robot_name, configuration_name)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_parallel(num_threads);
        set_or_add_lynx_var_generic!(&mut lynx_vars, Robot, "robot", robot);
        return Ok(lynx_vars);
    }

    pub fn new_single_threaded_packaged_with_robot_world(robot_names: Vec<&str>, configuration_names: Vec<Option<&str>>, environment_name: Option<&str>) -> Result<Self, String> {
        let robot_world = RobotWorld::new(robot_names, configuration_names, environment_name)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
        set_or_add_lynx_var_generic!(&mut lynx_vars, RobotWorld, "robot_world", robot_world);
        return Ok(lynx_vars);
    }

    pub fn new_parallel_packaged_with_robot_world(num_threads: Option<usize>, robot_names: Vec<&str>, configuration_names: Vec<Option<&str>>, environment_name: Option<&str>) -> Result<Self, String> {
        let robot_world = RobotWorld::new(robot_names, configuration_names, environment_name)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_parallel(num_threads);
        set_or_add_lynx_var_generic!(&mut lynx_vars, RobotWorld, "robot_world", robot_world);
        return Ok(lynx_vars);
    }

    pub fn new_single_threaded_packaged_with_robot_world_from_set_name(robot_set_name: &str, environment_name: Option<&str>) -> Result<Self, String> {
        let robot_world = RobotWorld::new_from_set_name(robot_set_name, environment_name)?;
        let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
        set_or_add_lynx_var_generic!(&mut lynx_vars, RobotWorld, "robot_world", robot_world);
        return Ok(lynx_vars);
    }

    pub fn new_parallel_packaged_with_robot_world_from_set_name(num_threads: Option<usize>, robot_set_name: &str, environment_name: Option<&str>) -> Result<Self, String> {
        let robot_world = RobotWorld::new_from_set_name(robot_set_name, environment_name)?;
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

    pub fn get_robot_ref(&self, name: Option<&str>) -> Result<&Robot, String> {
        return match name {
            None => { get_lynx_var_ref_generic!(self, Robot, "robot_module_toolbox") }
            Some(n) => { get_lynx_var_ref_generic!(self, Robot, n) }
        }
    }

    pub fn get_robot_mut_ref(&mut self, name: Option<&str>) -> Result<&mut Robot, String> {
        return match name {
            None => { get_lynx_var_mut_ref_generic!(self, Robot, "robot_module_toolbox") }
            Some(n) => { get_lynx_var_mut_ref_generic!(self, Robot, n) }
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

    pub fn get_robot_set_ref_via_robot_world(&self, robot_world_name: Option<&str>) -> Result<&RobotSet, String> {
        let robot_world = self.get_robot_world_ref(robot_world_name)?;
        return Ok(robot_world.get_robot_set_ref());
    }

    pub fn get_robot_set_mut_ref_via_robot_world(&mut self, robot_world_name: Option<&str>) -> Result<&mut RobotSet, String> {
        let robot_world = self.get_robot_world_mut_ref(robot_world_name)?;
        return Ok(robot_world.get_robot_set_mut_ref());
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