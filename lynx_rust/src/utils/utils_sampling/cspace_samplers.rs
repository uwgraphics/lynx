use crate::robot_modules::robot_world::RobotWorld;
use crate::utils::utils_vars::{lynx_vars_generic::LynxVarsGeneric, lynx_vars_user::*};
use crate::utils::utils_sampling::float_vec_sampler_traits::*;
use crate::utils::utils_sampling::multi_float_vec_sampler_traits::*;
use crate::utils::utils_collisions::collision_checker::*;
use crate::utils::utils_collisions::collision_check_result_enum::CollisionCheckResult;
use nalgebra::DVector;
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use std::marker::{Sync, Send};
use std::time::Instant;

#[derive(Clone)]
pub struct FreeSpaceSampler {
    _base_sampler: LynxFloatVecSamplerBox,
    _collision_checker: CollisionCheckerBox
}
impl FreeSpaceSampler {
    pub fn new(base_sampler: LynxFloatVecSamplerBox, collision_checker: CollisionCheckerBox) -> Self {
        Self { _base_sampler: base_sampler, _collision_checker: collision_checker }
    }
}
impl LynxFloatVecSampler for FreeSpaceSampler {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        let mut num_tries = 0;
        loop {
            let sample = self._base_sampler.lynx_float_vec_sampler_sample(lynx_vars)?;
            // let start = Instant::now();
            let in_collision_res = self._collision_checker.in_collision(&sample, lynx_vars)?;
            match in_collision_res {
                CollisionCheckResult::NotInCollision => {
                    // println!("{:?}", num_tries);
                    return Ok(sample.clone());
                }
                CollisionCheckResult::InCollision(s) => {
                    // println!("+++{:?}, {:?}", start.elapsed(), s);
                }
                CollisionCheckResult::Error(_) => { }
            }
            num_tries+=1;
        }
    }
}
impl LynxMultiFloatVecSampler for FreeSpaceSampler { }
impl LynxVarsUser for FreeSpaceSampler {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ];
    }
}

