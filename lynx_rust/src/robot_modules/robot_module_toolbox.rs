use crate::robot_modules::robot_model_module::RobotModelModule;
use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use crate::robot_modules::robot_dof_module::RobotDOFModule;
use crate::robot_modules::robot_fk_module::RobotFKModule;
use crate::robot_modules::robot_bounds_module::RobotBoundsModule;
use crate::robot_modules::robot_core_collision_module::RobotCoreCollisionModule;
use crate::robot_modules::robot_triangle_mesh_collision_module::RobotTriangleMeshCollisionModule;
use crate::robot_modules::robot_core_collision_parallel_module::RobotCoreCollisionParallelModule;
use crate::robot_modules::robot_triangle_mesh_collision_parallel_module::RobotTriangleMeshCollisionParallelModule;
use crate::robot_modules::robot_salient_links_module::RobotSalientLinksModule;
use termion::{style, color};
use std::fmt;


#[derive(Clone)]
pub struct RobotModuleToolbox {
    _robot_configuration_module: RobotConfigurationModule,
    _robot_dof_module: RobotDOFModule,
    _robot_bounds_module: RobotBoundsModule,
    _robot_fk_module: RobotFKModule,
    _robot_salient_links_module: RobotSalientLinksModule,
    _robot_core_collision_module: RobotCoreCollisionModule,
    _robot_triangle_mesh_collision_module: Option<RobotTriangleMeshCollisionModule>,
    // _robot_core_collision_parallel_module: Option<RobotCoreCollisionParallelModule>,
    // _robot_triangle_mesh_collision_parallel_module: Option<RobotTriangleMeshCollisionParallelModule>
}

impl RobotModuleToolbox {
    pub fn new( robot_name: &str, configuration_name: Option<&str>, mobile_base_bounds_filename: Option<&str>) -> Result<Self, String> {
        let _robot_configuration_module = RobotConfigurationModule::new(robot_name, configuration_name)?;
        let _robot_dof_module = RobotDOFModule::new(&_robot_configuration_module);
        let _robot_bounds_module = RobotBoundsModule::new(&_robot_configuration_module, &_robot_dof_module, mobile_base_bounds_filename);
        let _robot_fk_module = RobotFKModule::new(&_robot_configuration_module, &_robot_dof_module);
        let _robot_salient_links_module = RobotSalientLinksModule::new(&_robot_configuration_module);
        let _robot_core_collision_module = RobotCoreCollisionModule::new(&_robot_configuration_module, &_robot_fk_module, &_robot_bounds_module)?;

        return Ok( Self { _robot_configuration_module, _robot_dof_module, _robot_bounds_module, _robot_fk_module,
            _robot_salient_links_module, _robot_core_collision_module, _robot_triangle_mesh_collision_module: None } );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_configuration_module_ref(&self) -> &RobotConfigurationModule {
        return &self._robot_configuration_module;
    }

    pub fn get_dof_module_ref(&self) -> &RobotDOFModule {
        return &self._robot_dof_module;
    }

    pub fn get_bounds_module_ref(&self) -> &RobotBoundsModule { return &self._robot_bounds_module; }

    pub fn get_bounds_module_mut_ref(&mut self) -> &RobotBoundsModule { return &mut self._robot_bounds_module; }

    pub fn get_fk_module_ref(&self) -> &RobotFKModule {
        return &self._robot_fk_module;
    }

    pub fn get_salient_links_module_ref(&self) -> &RobotSalientLinksModule { return &self._robot_salient_links_module; }

    pub fn get_core_collision_module_ref(&self) -> &RobotCoreCollisionModule {
        return &self._robot_core_collision_module;
    }

    pub fn get_core_collision_module_mut_ref(&mut self) -> &mut RobotCoreCollisionModule {
        return &mut self._robot_core_collision_module;
    }

    pub fn get_triangle_mesh_collision_module_ref(&mut self) -> Result<&RobotTriangleMeshCollisionModule, String> {
        if self._robot_triangle_mesh_collision_module.is_none() {
            println!("{}{} creating a triangle_mesh_collision_module in robot_module_toolbox.  May take a few seconds...{}", color::Fg(color::Blue), style::Bold, style::Reset);
            let _robot_triangle_mesh_collision_module = RobotTriangleMeshCollisionModule::new( &self._robot_configuration_module, &self._robot_fk_module, &self._robot_bounds_module)?;
            self._robot_triangle_mesh_collision_module = Some( _robot_triangle_mesh_collision_module );
        }

        return Ok(&self._robot_triangle_mesh_collision_module.as_ref().unwrap());
    }

    pub fn get_triangle_mesh_collision_module_mut_ref(&mut self) -> Result<&mut RobotTriangleMeshCollisionModule, String> {
        if self._robot_triangle_mesh_collision_module.is_none() {
            println!("{}{} creating a triangle_mesh_collision_module in robot_module_toolbox.  May take a few seconds...{}", color::Fg(color::Blue), style::Bold, style::Reset);
            let _robot_triangle_mesh_collision_module = RobotTriangleMeshCollisionModule::new( &self._robot_configuration_module, &self._robot_fk_module, &self._robot_bounds_module)?;
            self._robot_triangle_mesh_collision_module = Some( _robot_triangle_mesh_collision_module );
        }

        match &mut self._robot_triangle_mesh_collision_module {
            Some(m) => return Ok(m),
            None => return Err("for some reason, triangle_mesh_collision_module was not set.".to_string())
        }
    }

    /*
    pub fn get_core_collision_parallel_module_ref(&mut self) -> Result<&RobotCoreCollisionParallelModule, String> {
        if self._robot_core_collision_parallel_module.is_none() {
            let _robot_core_collision_parallel_module = RobotCoreCollisionParallelModule::new_from_robot_core_collision_module(self.get_core_collision_module_ref(), None)?;
            self._robot_core_collision_parallel_module = Some( _robot_core_collision_parallel_module );
        }

        return Ok(&self._robot_core_collision_parallel_module.as_ref().unwrap());
    }

    pub fn get_core_collision_parallel_module_mut_ref(&mut self) -> Result<&mut RobotCoreCollisionParallelModule, String> {
        if self._robot_core_collision_parallel_module.is_none() {
            let _robot_core_collision_parallel_module = RobotCoreCollisionParallelModule::new_from_robot_core_collision_module(self.get_core_collision_module_ref(), None)?;
            self._robot_core_collision_parallel_module = Some( _robot_core_collision_parallel_module );
        }

        match &mut self._robot_core_collision_parallel_module {
            Some(m) => return Ok(m),
            None => return Err("for some reason, robot_core_collision_parallel_module was not set.".to_string())
        }

    }

    pub fn get_triangle_mesh_collision_parallel_module_ref(&mut self) -> Result<&RobotTriangleMeshCollisionParallelModule, String> {
        if self._robot_triangle_mesh_collision_parallel_module.is_none() {
            let _robot_triangle_mesh_collision_parallel_module = RobotTriangleMeshCollisionParallelModule::new_from_robot_triangle_mesh_collision_module(self.get_triangle_mesh_collision_module_ref()?, None)?;
            self._robot_triangle_mesh_collision_parallel_module = Some( _robot_triangle_mesh_collision_parallel_module );
        }

        return Ok(&self._robot_triangle_mesh_collision_parallel_module.as_ref().unwrap());
    }

    pub fn get_triangle_mesh_collision_parallel_module_mut_ref(&mut self) -> Result<&mut RobotTriangleMeshCollisionParallelModule, String> {
        if self._robot_triangle_mesh_collision_parallel_module.is_none() {
            let _robot_triangle_mesh_collision_parallel_module = RobotTriangleMeshCollisionParallelModule::new_from_robot_triangle_mesh_collision_module(self.get_triangle_mesh_collision_module_ref()?, None)?;
            self._robot_triangle_mesh_collision_parallel_module = Some( _robot_triangle_mesh_collision_parallel_module );
        }

        match &mut self._robot_triangle_mesh_collision_parallel_module {
            Some(m) => return Ok(m),
            None => return Err("for some reason, robot_triangle_mesh_collision_parallel_module was not set.".to_string())
        }

    }
    */

    ////////////////////////////////////////////////////////////////////////////////////////////////
}

impl fmt::Debug for RobotModuleToolbox {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Ok(())
    }
}