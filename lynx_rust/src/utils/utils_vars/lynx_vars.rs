use std::collections::HashMap;
use crate::utils::utils_collisions::{collision_environment::CollisionEnvironment, collision_multiple_results::*};
use crate::robot_modules::{robot_world::RobotWorld, robot_module_toolbox::RobotModuleToolbox};
use crate::utils::utils_image_environments::image_environment::ImageEnvironment;
use crate::robot_modules::robot_fk_module::*;
use crate::utils::utils_collisions::collision_object_group_queries::*;
use nalgebra::{DVector};
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;

#[derive(Clone, Debug)]
pub struct LynxVars {
    pub all_var_types_and_names: Vec<(&'static str, &'static str)>,
    pub usize: (Vec<usize>, HashMap<&'static str, usize>, usize ),
    pub usizeVec: (Vec<Vec<usize>>, HashMap<&'static str, usize>, usize ),
    pub f64: (Vec<f64>, HashMap<&'static str, usize>, usize ),
    pub f64Vec: (Vec<Vec<f64>>, HashMap<&'static str, usize>, usize ),
    pub f64VecOption: (Vec<Option<Vec<f64>>>, HashMap<&'static str, usize>, usize ),
    pub DVectorF64: (Vec<DVector<f64>>, HashMap<&'static str, usize>, usize),
    pub DVectorF64Vec: (Vec<Vec<DVector<f64>>>, HashMap<&'static str, usize>, usize),
    pub RobotModuleToolbox: (Vec<RobotModuleToolbox>, HashMap<&'static str, usize>, usize),
    pub RobotWorld: (Vec<RobotWorld>, HashMap<&'static str, usize>, usize),
    pub CollisionEnvironment: (Vec<CollisionEnvironment>, HashMap<&'static str, usize>, usize),
    pub CollisionEnvironmentOption: (Vec<Option<CollisionEnvironment>>, HashMap<&'static str, usize>, usize),
    pub ImageEnvironment: (Vec<ImageEnvironment>, HashMap<&'static str, usize>, usize),
    pub RobotFKResult: (Vec<RobotFKResult>, HashMap<&'static str, usize>, usize),
    pub RobotFKResultVec: (Vec<Vec<RobotFKResult>>, HashMap<&'static str, usize>, usize),
    pub RobotFKGradientPerturbationsResult: (Vec<RobotFKGradientPerturbationsResult>, HashMap<&'static str, usize>, usize),
    pub RobotFKGradientPerturbationsResultVec: (Vec<Vec<RobotFKGradientPerturbationsResult>>, HashMap<&'static str, usize>, usize),
    pub IntersectCheckMultipleResult: (Vec<IntersectCheckMultipleResult>, HashMap<&'static str, usize>, usize),
    pub IntersectCheckMultipleResultVec: (Vec<Vec<IntersectCheckMultipleResult>>, HashMap<&'static str, usize>, usize),
    pub DistanceCheckMultipleResult: (Vec<DistanceCheckMultipleResult>, HashMap<&'static str, usize>, usize),
    pub DistanceCheckMultipleResultVec: (Vec<Vec<DistanceCheckMultipleResult>>, HashMap<&'static str, usize>, usize),
    pub ContactCheckMultipleResult: (Vec<ContactCheckMultipleResult>, HashMap<&'static str, usize>, usize),
    pub ContactCheckMultipleResultVec: (Vec<Vec<ContactCheckMultipleResult>>, HashMap<&'static str, usize>, usize),
    pub LinkGeometryType: (Vec<LinkGeometryType>, HashMap<&'static str, usize>, usize),
    pub UsizePairPair: (Vec<  [ [usize; 2]; 2 ]  >, HashMap<&'static str, usize>, usize),
    pub UsizePairPairVec: (Vec<Vec< [ [usize; 2]; 2 ] >>, HashMap<&'static str, usize>, usize),
}

impl LynxVars{
    pub fn new_empty() -> Self {
        return Self {
            all_var_types_and_names: Vec::new(),
            usize: (Vec::new(), HashMap::new(), 0),
            usizeVec: (Vec::new(), HashMap::new(), 0),
            f64: (Vec::new(), HashMap::new(), 0),
            f64Vec: (Vec::new(), HashMap::new(), 0),
            f64VecOption: (Vec::new(), HashMap::new(), 0),
            DVectorF64: (Vec::new(), HashMap::new(), 0),
            DVectorF64Vec: (Vec::new(), HashMap::new(), 0),
            RobotModuleToolbox: (Vec::new(), HashMap::new(), 0),
            RobotWorld: (Vec::new(), HashMap::new(), 0),
            CollisionEnvironment: (Vec::new(), HashMap::new(), 0),
            CollisionEnvironmentOption: (Vec::new(), HashMap::new(), 0),
            ImageEnvironment: (Vec::new(), HashMap::new(), 0),
            RobotFKResult: (Vec::new(), HashMap::new(), 0),
            RobotFKResultVec: (Vec::new(), HashMap::new(), 0),
            RobotFKGradientPerturbationsResult: (Vec::new(), HashMap::new(), 0),
            RobotFKGradientPerturbationsResultVec: (Vec::new(), HashMap::new(), 0),
            IntersectCheckMultipleResult: (Vec::new(), HashMap::new(), 0),
            IntersectCheckMultipleResultVec: (Vec::new(), HashMap::new(), 0),
            DistanceCheckMultipleResult: (Vec::new(), HashMap::new(), 0),
            DistanceCheckMultipleResultVec: (Vec::new(), HashMap::new(), 0),
            ContactCheckMultipleResult: (Vec::new(), HashMap::new(), 0),
            ContactCheckMultipleResultVec: (Vec::new(), HashMap::new(), 0),
            LinkGeometryType: (Vec::new(), HashMap::new(), 0),
            UsizePairPair: (Vec::new(), HashMap::new(), 0),
            UsizePairPairVec: (Vec::new(), HashMap::new(), 0),
        }
    }

    pub fn contains_variable(&self, var_type: &'static str, var_name: &'static str) -> bool {
        return self.all_var_types_and_names.contains( &(var_type, var_name) );
    }

    pub fn print(&self) {
        println!("{:?}", self);
    }
}
