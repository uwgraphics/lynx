use crate::utils::utils_vars::{lynx_vars_generic::LynxVarsGeneric, lynx_vars_user::*};
use nalgebra::DVector;
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use std::marker::{Sync, Send};


pub trait IntVecSampler:  Send + Sync + IntVecSamplerClone {
    fn int_vec_sampler_sample(&self) -> Result<DVector<i32>, String>;
    fn to_int_vec_sampler_box(&self) -> IntVecSamplerBox {
        return IntVecSamplerBox(self.int_vec_sampler_clone_box());
    }
}

pub trait IntVecSamplerClone {
    fn int_vec_sampler_clone_box(&self) -> Box<dyn IntVecSampler>;
}
impl<T> IntVecSamplerClone for T where T: 'static + IntVecSampler + Clone {
    fn int_vec_sampler_clone_box(&self) -> Box<dyn IntVecSampler> {
        Box::new(self.clone())
    }
}

pub struct IntVecSamplerBox(Box<dyn IntVecSampler>);
impl IntVecSamplerBox {
    fn int_vec_sampler_sample(&self) -> Result<DVector<i32>, String> {
        return self.0.int_vec_sampler_sample();
    }
}
impl Clone for IntVecSamplerBox {
    fn clone(&self) -> Self {
        let c = self.0.int_vec_sampler_clone_box();
        return Self(c);
    }
}
unsafe impl Send for IntVecSamplerBox { }
unsafe impl Sync for IntVecSamplerBox { }

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait LynxIntVecSampler: Send + Sync + LynxIntVecSamplerClone + LynxVarsUser + AsLynxVarsUser {
    fn lynx_int_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<i32>, String>;
    fn to_lynx_int_vec_sampler_box(&self) -> LynxIntVecSamplerBox {
        return LynxIntVecSamplerBox(self.lynx_int_vec_sampler_clone_box());
    }
}

pub trait LynxIntVecSamplerClone {
    fn lynx_int_vec_sampler_clone_box(&self) -> Box<dyn LynxIntVecSampler>;
}
impl<T> LynxIntVecSamplerClone for T where T: 'static + LynxIntVecSampler + Clone {
    fn lynx_int_vec_sampler_clone_box(&self) -> Box<dyn LynxIntVecSampler> {
        Box::new(self.clone())
    }
}

pub trait AsLynxIntVecSampler {
    fn as_lynx_int_vec_sampler(&self) -> &LynxIntVecSampler;
}
impl<T: LynxIntVecSampler> AsLynxIntVecSampler for T {
    fn as_lynx_int_vec_sampler(&self) -> &LynxIntVecSampler {
        self
    }
}

pub struct LynxIntVecSamplerBox(Box<dyn LynxIntVecSampler>);
impl LynxIntVecSamplerBox {
    pub fn lynx_int_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<i32>, String> {
        return self.0.lynx_int_vec_sampler_sample(lynx_vars);
    }
}
impl Clone for LynxIntVecSamplerBox {
    fn clone(&self) -> Self {
        let c = self.0.lynx_int_vec_sampler_clone_box();
        return Self(c);
    }
}
unsafe impl Send for LynxIntVecSamplerBox { }
unsafe impl Sync for LynxIntVecSamplerBox { }