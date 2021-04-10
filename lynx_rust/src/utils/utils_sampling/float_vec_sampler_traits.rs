use crate::utils::utils_vars::{lynx_vars_generic::LynxVarsGeneric, lynx_vars_user::*};
use nalgebra::DVector;
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use std::marker::{Sync, Send};


pub trait FloatVecSampler: Send + Sync + FloatVecSamplerClone {
    fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String>;
    fn to_float_vec_sampler_box(&self) -> FloatVecSamplerBox {
        return FloatVecSamplerBox(self.float_vec_sampler_clone_box());
    }
}

pub trait FloatVecSamplerClone {
    fn float_vec_sampler_clone_box(&self) -> Box<dyn FloatVecSampler>;
}
impl<T> FloatVecSamplerClone for T where T: 'static + FloatVecSampler + Clone {
    fn float_vec_sampler_clone_box(&self) -> Box<dyn FloatVecSampler> {
        Box::new(self.clone())
    }
}

pub trait AsFloatVecSampler {
    fn as_float_vec_sampler(&self) -> &dyn FloatVecSampler;
}
impl<T: FloatVecSampler> AsFloatVecSampler for T {
    fn as_float_vec_sampler(&self) -> &dyn FloatVecSampler {
        self
    }
}

pub struct FloatVecSamplerBox(Box<dyn FloatVecSampler>);
impl FloatVecSamplerBox {
    pub fn new(float_vec_sampler: &dyn FloatVecSampler) -> Self {
        return Self( float_vec_sampler.float_vec_sampler_clone_box() );
    }

    pub fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String> {
        return self.0.float_vec_sampler_sample();
    }
}
impl Clone for FloatVecSamplerBox {
    fn clone(&self) -> Self {
        let c = self.0.float_vec_sampler_clone_box();
        return Self(c);
    }
}
unsafe impl Send for FloatVecSamplerBox { }
unsafe impl Sync for FloatVecSamplerBox { }

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait LynxFloatVecSampler: Send + Sync + LynxFloatVecSamplerClone + LynxVarsUser + AsLynxVarsUser {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String>;
    fn to_lynx_float_vec_sampler_box(&self) -> LynxFloatVecSamplerBox {
        return LynxFloatVecSamplerBox(self.lynx_float_vec_sampler_clone_box());
    }
}

pub trait LynxFloatVecSamplerClone {
    fn lynx_float_vec_sampler_clone_box(&self) -> Box<dyn LynxFloatVecSampler>;
}
impl<T> LynxFloatVecSamplerClone for T where T: 'static + LynxFloatVecSampler + Clone {
    fn lynx_float_vec_sampler_clone_box(&self) -> Box<dyn LynxFloatVecSampler> {
        Box::new(self.clone())
    }
}

pub trait AsLynxFloatVecSampler {
    fn as_lynx_float_vec_sampler(&self) -> &LynxFloatVecSampler;
}
impl<T: LynxFloatVecSampler> AsLynxFloatVecSampler for T {
    fn as_lynx_float_vec_sampler(&self) -> &LynxFloatVecSampler {
        self
    }
}

pub struct LynxFloatVecSamplerBox(Box<dyn LynxFloatVecSampler>);
impl LynxFloatVecSamplerBox {
    pub fn new(lynx_float_vec_sampler: &dyn LynxFloatVecSampler) -> Self {
        return Self(lynx_float_vec_sampler.lynx_float_vec_sampler_clone_box());
    }

    pub fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        return self.0.lynx_float_vec_sampler_sample(lynx_vars);
    }
}
impl Clone for LynxFloatVecSamplerBox {
    fn clone(&self) -> Self {
        let c = self.0.lynx_float_vec_sampler_clone_box();
        return Self(c);
    }
}
unsafe impl Send for LynxFloatVecSamplerBox { }
unsafe impl Sync for LynxFloatVecSamplerBox { }