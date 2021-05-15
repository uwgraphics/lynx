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
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use crate::utils::utils_files_and_strings::prelude::*;
use crate::utils::utils_preprocessing::mesh_preprocessing_utils::save_all_links_as_triangle_meshes;
use termion::{style, color};
use std::fmt;


#[derive(Clone)]
pub struct Robot {
    _robot_name: String,
    _robot_configuration_module: RobotConfigurationModule,
    _robot_dof_module: RobotDOFModule,
    _robot_bounds_module: RobotBoundsModule,
    _robot_fk_module: RobotFKModule,
    _robot_salient_links_module: RobotSalientLinksModule,
    _robot_core_collision_module: RobotCoreCollisionModule,
    _robot_triangle_mesh_collision_module: Option<RobotTriangleMeshCollisionModule>,
    // _robot_mesh_info_module: RobotMeshInfoModule
    // _robot_core_collision_parallel_module: Option<RobotCoreCollisionParallelModule>,
    // _robot_triangle_mesh_collision_parallel_module: Option<RobotTriangleMeshCollisionParallelModule>
}

impl Robot {
    pub fn new(robot_name: &str, configuration_name: Option<&str>) -> Result<Self, String> {
        if !check_if_robot_is_valid_choice(robot_name) { return Err(format!("Robot {} is not a valid choice as there is not a folder in the robots directory that has this name.", robot_name)) }

        Robot::_create_link_triangle_meshes_if_need_be(&robot_name.to_string())?;

        let _robot_configuration_module = RobotConfigurationModule::new(robot_name, configuration_name)?;
        let _robot_name = _robot_configuration_module.robot_model_module.robot_name.clone();
        let _robot_dof_module = RobotDOFModule::new(&_robot_configuration_module);
        let _robot_bounds_module = RobotBoundsModule::new(&_robot_configuration_module, &_robot_dof_module);
        let _robot_fk_module = RobotFKModule::new(&_robot_configuration_module, &_robot_dof_module);
        let _robot_salient_links_module = RobotSalientLinksModule::new(&_robot_configuration_module);
        let _robot_core_collision_module = RobotCoreCollisionModule::new(&_robot_configuration_module, &_robot_fk_module, &_robot_bounds_module)?;
        // let _robot_mesh_info_module = RobotMeshInfoModule::new(&_robot_configuration_module);

        return Ok( Self { _robot_name, _robot_configuration_module, _robot_dof_module, _robot_bounds_module, _robot_fk_module,
            _robot_salient_links_module, _robot_core_collision_module, _robot_triangle_mesh_collision_module: None } );
    }

    pub fn new_from_manual_inputs(robot_name: &str, configuration_name: &str, base_offset: ImplicitDualQuaternion, dead_end_link_names: Vec<String>, inactive_joint_names: Vec<String>, mobile_base_mode: String, mobile_base_bounds_filename: Option<&str>) -> Result<Self, String> {
        if !check_if_robot_is_valid_choice(robot_name) { return Err(format!("Robot {} is not a valid choice as there is not a folder in the robots directory that has this name.", robot_name)) }

        Robot::_create_link_triangle_meshes_if_need_be(&robot_name.to_string())?;

        let _robot_configuration_module = RobotConfigurationModule::new_manual_inputs(robot_name, configuration_name, base_offset, dead_end_link_names, inactive_joint_names, mobile_base_mode, str_option_to_string_option(mobile_base_bounds_filename));
        let _robot_name = _robot_configuration_module.robot_model_module.robot_name.clone();
        let _robot_dof_module = RobotDOFModule::new(&_robot_configuration_module);
        let _robot_bounds_module = RobotBoundsModule::new(&_robot_configuration_module, &_robot_dof_module);
        let _robot_fk_module = RobotFKModule::new(&_robot_configuration_module, &_robot_dof_module);
        let _robot_salient_links_module = RobotSalientLinksModule::new(&_robot_configuration_module);
        let _robot_core_collision_module = RobotCoreCollisionModule::new(&_robot_configuration_module, &_robot_fk_module, &_robot_bounds_module)?;
        // let _robot_mesh_info_module = RobotMeshInfoModule::new(&_robot_configuration_module);

        return Ok( Self { _robot_name, _robot_configuration_module, _robot_dof_module, _robot_bounds_module, _robot_fk_module,
            _robot_salient_links_module, _robot_core_collision_module, _robot_triangle_mesh_collision_module: None } );
    }

    pub fn new_from_configuration_module(robot_configuration_module: &RobotConfigurationModule) -> Result<Self, String> {
        Robot::_create_link_triangle_meshes_if_need_be(&robot_configuration_module.robot_model_module.robot_name)?;

        let _robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let _robot_dof_module = RobotDOFModule::new(robot_configuration_module);
        let _robot_bounds_module = RobotBoundsModule::new(robot_configuration_module, &_robot_dof_module);
        let _robot_fk_module = RobotFKModule::new(robot_configuration_module, &_robot_dof_module);
        let _robot_salient_links_module = RobotSalientLinksModule::new(robot_configuration_module);
        let _robot_core_collision_module = RobotCoreCollisionModule::new(robot_configuration_module, &_robot_fk_module, &_robot_bounds_module)?;
        // let _robot_mesh_info_module = RobotMeshInfoModule::new(robot_configuration_module);

        return Ok( Self { _robot_name, _robot_configuration_module: robot_configuration_module.clone(), _robot_dof_module, _robot_bounds_module, _robot_fk_module,
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
    pub fn get_mesh_info_module_ref(&self) -> &RobotMeshInfoModule {
        return &self._robot_mesh_info_module;
    }
    */

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_robot_name_ref(&self) -> &String {
        return &self._robot_name;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _create_link_triangle_meshes_if_need_be(robot_name: &String) -> Result<(), String> {
        let exists1 = check_if_path_exists_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_triangle_meshes_visual".to_string());
        let exists2 = check_if_path_exists_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_triangle_meshes_collision".to_string());
        if !(exists1 || exists2) {
            save_all_links_as_triangle_meshes(robot_name.clone())?;
        }
        Ok(())
    }

}

impl fmt::Debug for Robot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Ok(())
    }
}