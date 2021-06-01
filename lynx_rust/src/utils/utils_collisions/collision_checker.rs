use crate::robot_modules::{robot_core_collision_module::RobotCoreCollisionModule, robot_fk_module::RobotFKModule, robot_bounds_module::{RobotBoundsModule, BoundsCheckResult}, robot_dof_module::RobotDOFModule, robot_configuration_module::RobotConfigurationModule};
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;
use crate::robot_modules::robot::Robot;
use crate::robot_modules::robot_world::RobotWorld;
use crate::utils::utils_collisions::{collision_check_result_enum::*, collision_multiple_results::*};
use crate::utils::utils_math::nalgebra_utils::vec_to_dvec;
use crate::utils::utils_vars::lynx_vars_generic::LynxVarsGeneric;
use crate::utils::utils_vars::lynx_vars_user::*;
use crate::utils::utils_image_environments::image_environment::ImageEnvironment;
use nalgebra::{DVector};

pub trait CollisionChecker: Send + Sync + LynxVarsUser + AsLynxVarsUser + CollisionCheckerClone {
    fn in_collision(&self, state: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric) -> Result<CollisionCheckResult, String>;
    fn get_collision_environment_name(&self, lynx_vars: &LynxVarsGeneric) -> Result<String, String> { Ok("".to_string()) }
    fn to_collision_checker_box(&self) -> CollisionCheckerBox {
        return CollisionCheckerBox(self.clone_box());
    }
}

pub trait CollisionCheckerClone {
    fn clone_box(&self) -> Box<dyn CollisionChecker>;
}
impl<T> CollisionCheckerClone for T where T: 'static + CollisionChecker + Clone {
    fn clone_box(&self) -> Box<dyn CollisionChecker> {
        Box::new(self.clone())
    }
}

pub struct CollisionCheckerBox(Box<dyn CollisionChecker>);
impl CollisionCheckerBox {
    pub fn new(collision_checker: &dyn CollisionChecker) -> Self {
        return Self(collision_checker.clone_box());
    }

    pub fn in_collision(&self, state: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric) -> Result<CollisionCheckResult, String> {
        return self.0.in_collision(state, lynx_vars);
    }

    pub fn get_collision_environment_name(&self, lynx_vars: &LynxVarsGeneric) -> Result<String, String> {
        return self.0.get_collision_environment_name(lynx_vars);
    }
}
impl Clone for CollisionCheckerBox {
    fn clone(&self) -> Self {
        let c = self.0.clone_box();
        return Self(c);
    }
}
unsafe impl Send for CollisionCheckerBox { }
unsafe impl Sync for CollisionCheckerBox { }

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


#[derive(Clone)]
pub struct NullCollisionChecker;
impl CollisionChecker for NullCollisionChecker {
    fn in_collision(&self, state: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric) ->  Result<CollisionCheckResult, String> { return Ok(CollisionCheckResult::NotInCollision) }
}
impl LynxVarsUser for NullCollisionChecker { }

#[derive(Clone)]
pub struct AlwaysCollision;
impl CollisionChecker for AlwaysCollision {
    fn in_collision(&self, state: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric) -> Result<CollisionCheckResult, String>{ return Ok(CollisionCheckResult::InCollision("always collision".to_string())); }
}
impl LynxVarsUser for AlwaysCollision { }

#[derive(Clone)]
pub struct SphereCollisionChecker {
    pub radius: f64,
    pub center: DVector<f64>
}
impl SphereCollisionChecker {
    pub fn new(radius: f64, center: &Vec<f64>) -> Self {
        Self{radius, center: vec_to_dvec(center)}
    }
}
impl CollisionChecker for SphereCollisionChecker {
    fn in_collision(&self, state: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric) -> Result<CollisionCheckResult, String> {
        let dis = (state - &self.center).norm();
        if (state - &self.center).norm() < self.radius {
            return Ok(CollisionCheckResult::InCollision(format!("{:?} is within the radius", dis)));
        } else {
            return Ok(CollisionCheckResult::NotInCollision);
        }
    }
}
impl LynxVarsUser for SphereCollisionChecker { }

#[derive(Clone)]
pub struct RobotWorldCollisionChecker;
impl CollisionChecker for RobotWorldCollisionChecker {
    fn in_collision(&self, state: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric) -> Result<CollisionCheckResult, String> {
        let mut robot_world = get_lynx_var_mut_ref_generic!(lynx_vars, RobotWorld, "robot_world")?;

        let bounds_check = robot_world.get_robot_set_mut_ref().check_if_state_is_within_bounds(state)?;
        match bounds_check {
            BoundsCheckResult::InBounds => {}
            BoundsCheckResult::OutOfBounds(s) => { return Ok( CollisionCheckResult::InCollision(s) ) }
            BoundsCheckResult::Error(s) => { return Err(s) }
        }

        let fk_res = robot_world.get_robot_set_mut_ref().compute_fk(state)?;

        let self_collision_check = robot_world.get_robot_set_mut_ref().self_intersect_check(&fk_res, LinkGeometryType::OBBs, true)?;
        if self_collision_check.in_collision() { return Ok(CollisionCheckResult::InCollision("self collision".to_string())) }

        let environment_collision_check = robot_world.environment_intersect_check(&fk_res, LinkGeometryType::OBBs, true)?;
        if environment_collision_check.in_collision() { return Ok(CollisionCheckResult::InCollision("environment collision".to_string())) }

        let multi_robot_collision_check = robot_world.get_robot_set_mut_ref().multi_robot_intersect_check(&fk_res, LinkGeometryType::OBBs, true)?;
        if multi_robot_collision_check.in_collision() { return Ok(CollisionCheckResult::InCollision("multi robot collision".to_string())) }

        return Ok(CollisionCheckResult::NotInCollision);
    }
}
impl LynxVarsUser for RobotWorldCollisionChecker {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("RobotWorld", "robot_world") ];
    }
}

#[derive(Clone)]
pub struct ImageEnvironmentCollisionChecker {
    _image_environment: ImageEnvironment
}
impl ImageEnvironmentCollisionChecker {
    pub fn new(image_name: &str) -> Self {
        let image_environment = ImageEnvironment::new_collision_image(image_name);
        return Self { _image_environment: image_environment };
    }
}
impl CollisionChecker for ImageEnvironmentCollisionChecker {
    fn in_collision(&self, state: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric) -> Result<CollisionCheckResult, String> {
        // let image_environment = lynx_vars.get_image_environment_variable_ref("image_environment")?;
        // let image_environment = get_lynx_var_ref_generic!(lynx_vars, ImageEnvironment, "image_environment")?;

        if state[0] < 0.0 || state[1] < 0.0 || state[0] > self._image_environment.world_height || state[1] > self._image_environment.world_width {
            return Ok(CollisionCheckResult::InCollision("out of bounds on image".to_string()));
        }

        let image_val = self._image_environment.query_image(state[0], state[1], false);
        if image_val == 0.0 {
            return Ok(CollisionCheckResult::NotInCollision);
        } else {
            return Ok(CollisionCheckResult::InCollision("collision on image".to_string()));
        }
    }
    fn get_collision_environment_name(&self, lynx_vars: &LynxVarsGeneric) -> Result<String, String> {
        // let image_environment = get_lynx_var_ref_generic!(lynx_vars, ImageEnvironment, "image_environment")?;
        // return Ok(image_environment.image_name.clone());
        return Ok(self._image_environment.image_name.clone());
    }
}
impl LynxVarsUser for ImageEnvironmentCollisionChecker { }

