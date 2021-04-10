use crate::utils::utils_sampling::{float_vec_sampler_traits::*, multi_float_vec_sampler_traits::*, int_vec_sampler_traits::*, multi_int_vec_sampler_traits::*};
use crate::utils::utils_vars::{lynx_vars_generic::LynxVarsGeneric, lynx_vars_user::*};
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use std::marker::{Sync, Send};
use nalgebra::DVector;

#[derive(Clone)]
pub struct NullSampler2D;
impl FloatVecSampler for NullSampler2D {
    fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String> {
        return Ok(DVector::from_element(2, 0.0));
    }
}
impl LynxFloatVecSampler for NullSampler2D {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        return Ok(DVector::from_element(2, 0.0));
    }
}
impl LynxVarsUser for NullSampler2D { }

#[derive(Clone)]
pub struct NullSamplerND {
    pub dim: usize
}
impl FloatVecSampler for NullSamplerND {
    fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String> {
        return Ok(DVector::from_element(self.dim, 0.0));
    }
}
impl LynxFloatVecSampler for NullSamplerND {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        return Ok(DVector::from_element(self.dim, 0.0));
    }
}
impl LynxVarsUser for NullSamplerND { }

#[derive(Clone)]
pub struct RangeFloatVecSampler {
    pub upper_bound: f64,
    pub lower_bound: f64,
    pub dim: usize
}
impl RangeFloatVecSampler {
    pub fn new(lower_bound: f64, upper_bound: f64, dim: usize) -> Self {
        Self{upper_bound, lower_bound, dim}
    }
}
impl FloatVecSampler for RangeFloatVecSampler {
    fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        let mut rng = rand::thread_rng();
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.lower_bound, self.upper_bound);
        }
        Ok(v)
    }
}
impl LynxFloatVecSampler for RangeFloatVecSampler {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        let mut rng = rand::thread_rng();
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.lower_bound, self.upper_bound);
        }
        return Ok(v);
    }
}
impl LynxVarsUser for RangeFloatVecSampler { }

#[derive(Clone)]
pub struct RangeFloatVecSamplerSeedable {
    pub upper_bound: f64,
    pub lower_bound: f64,
    pub dim: usize,
    pub seed: u64
}
impl RangeFloatVecSamplerSeedable {
    pub fn new(lower_bound: f64, upper_bound: f64, dim: usize, seed: u64) -> Self {
        Self{upper_bound, lower_bound, dim, seed}
    }

    pub fn reseed(&mut self, seed: u64) {
        self.seed = seed;
    }
}
impl FloatVecSampler for RangeFloatVecSamplerSeedable {
    fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        let mut rng = StdRng::seed_from_u64(self.seed);
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.lower_bound, self.upper_bound);
        }
        Ok(v)
    }
}
impl LynxFloatVecSampler for RangeFloatVecSamplerSeedable {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        let mut rng = StdRng::seed_from_u64(self.seed);
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.lower_bound, self.upper_bound);
        }
        return Ok(v);
    }
}
impl LynxVarsUser for RangeFloatVecSamplerSeedable { }

#[derive(Clone)]
pub struct RectangleFloatVecSampler {
    pub bounds: Vec<(f64, f64)>,
    pub dim: usize
}
impl RectangleFloatVecSampler {
    pub fn new(bounds: Vec<(f64, f64)>) -> Self {
        let dim = bounds.len();
        Self {bounds : bounds.clone(), dim}
    }
}
impl FloatVecSampler for RectangleFloatVecSampler {
    fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        let mut rng = rand::thread_rng();
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.bounds[i].0, self.bounds[i].1);
        }
        Ok(v)
    }
}
impl LynxFloatVecSampler for RectangleFloatVecSampler {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        let mut v =  DVector::from_element(self.dim, 0.0);
        let mut rng = rand::thread_rng();
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.bounds[i].0, self.bounds[i].1);
        }
        return Ok(v);
    }
}
impl LynxVarsUser for RectangleFloatVecSampler { }

#[derive(Clone)]
pub struct RangeIntVecSampler {
    pub upper_bound: i32,
    pub lower_bound: i32,
    pub dim: usize
}
impl RangeIntVecSampler {
    pub fn new(lower_bound: i32, upper_bound: i32, dim: usize) -> Self {
        Self{upper_bound, lower_bound, dim}
    }
}
impl IntVecSampler for RangeIntVecSampler {
    fn int_vec_sampler_sample(&self) -> Result<DVector<i32>, String> {
        let mut v =  DVector::from_element(self.dim, 0);
        let mut rng = rand::thread_rng();
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.lower_bound, self.upper_bound + 1) as i32;
        }
        Ok(v)
    }
}
impl LynxIntVecSampler for RangeIntVecSampler {
    fn lynx_int_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<i32>, String> {
        let mut v =  DVector::from_element(self.dim, 0);
        let mut rng = rand::thread_rng();
        for i in 0..self.dim {
            v[i] = rng.gen_range(self.lower_bound, self.upper_bound + 1) as i32;
        }
        return Ok(v);
    }
}
impl LynxVarsUser for RangeIntVecSampler { }




