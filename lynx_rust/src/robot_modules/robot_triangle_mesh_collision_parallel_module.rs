use crate::robot_modules::{robot_triangle_mesh_collision_module::*, robot_configuration_module::*, robot_fk_module::*, robot_bounds_module::*};
use rayon::prelude::*;
use num_cpus::*;
use crate::robot_modules::robot_triangle_mesh_collision_module::RobotTriangleMeshCollisionModule;


#[derive(Clone)]
pub struct RobotTriangleMeshCollisionParallelModule {
    _robot_triangle_mesh_collision_modules: Vec<RobotTriangleMeshCollisionModule>,
    _num_threads: usize
}

impl RobotTriangleMeshCollisionParallelModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule, robot_fk_module: &RobotFKModule, robot_bounds_module: &RobotBoundsModule, num_threads: Option<usize>) -> Result<Self, String> {
        let mut _num_threads = num_cpus::get();
        if num_threads.is_some() { _num_threads = num_threads.unwrap(); }
        if _num_threads == 0 {
            return Err("num threads of 0 is invalid in RobotCoreCollisionParallelModule".to_string());
        }

        let mut _robot_triangle_mesh_collision_modules = Vec::new();
        let robot_triangle_mesh_collision_module = RobotTriangleMeshCollisionModule::new(robot_configuration_module, robot_fk_module, robot_bounds_module)?;
        _robot_triangle_mesh_collision_modules.push(robot_triangle_mesh_collision_module);
        for i in 1.._num_threads {
            _robot_triangle_mesh_collision_modules.push( _robot_triangle_mesh_collision_modules[0].clone() );
        }

        return Ok(Self { _robot_triangle_mesh_collision_modules, _num_threads });
    }

    pub fn new_from_robot_triangle_mesh_collision_module(robot_triangle_mesh_collision_module: &RobotTriangleMeshCollisionModule, num_threads: Option<usize>) -> Result<Self, String> {
        let mut _num_threads = num_cpus::get();
        if num_threads.is_some() { _num_threads = num_threads.unwrap(); }
        if _num_threads == 0 {
            return Err("num threads of 0 is invalid in RobotCoreCollisionParallelModule".to_string());
        }

        let mut _robot_triangle_mesh_collision_modules = Vec::new();
        for i in 0.._num_threads {
            _robot_triangle_mesh_collision_modules.push(robot_triangle_mesh_collision_module.clone());
        }

        return Ok(Self { _robot_triangle_mesh_collision_modules, _num_threads });
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_one(&mut self) -> &mut RobotTriangleMeshCollisionModule {
        return &mut self._robot_triangle_mesh_collision_modules[0];
    }

    pub fn get_all(&mut self) -> &mut Vec<RobotTriangleMeshCollisionModule> {
        return &mut self._robot_triangle_mesh_collision_modules;
    }

    pub fn get_num_threads(&self) -> usize {
        return self._num_threads;
    }
}