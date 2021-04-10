use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_sampling::{float_vec_sampler_traits::*};
use crate::utils::utils_runtime_management::termination_util::TerminationUtil;
use nalgebra::{DVector, max};
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use std::marker::{Sync, Send};
use std::sync::{Mutex, RwLock};
use std::time::Instant;

pub trait MultiFloatVecSampler: Send + Sync + MultiFloatVecSamplerClone + FloatVecSampler {
    fn multi_float_vec_sampler_sample(&self, target_num_samples: usize, max_sample_tries: usize, parallel: bool) -> Result<Vec<DVector<f64>>, String> {
        return if parallel {
            self.multi_float_vec_sampler_sample_default_parallel(target_num_samples, max_sample_tries)
        } else {
            self.multi_float_vec_sampler_sample_default_single_threaded(target_num_samples, max_sample_tries)
        }
    }
    fn multi_float_vec_sampler_sample_default_single_threaded(&self, target_num_samples: usize, max_sample_tries: usize) -> Result<Vec<DVector<f64>>, String> {
        let mut out_vec = Vec::new();

        for _ in 0..max_sample_tries {
            let s = self.float_vec_sampler_sample();
            if s.is_ok() {
                out_vec.push(s.ok().unwrap());
            }
            if out_vec.len() == target_num_samples { return Ok(out_vec); }
        }
        return Ok(out_vec);
    }
    fn multi_float_vec_sampler_sample_default_parallel(&self, target_num_samples: usize, max_sample_tries: usize) -> Result<Vec<DVector<f64>>, String> {
        let mut out_vec = RwLock::new(Vec::new());
        let mut terminate = TerminationUtil::new();

        let a : Vec<usize> = (0..num_cpus::get()).collect();
        a.par_iter().for_each(|x| {
            let mut terminate_local = terminate.clone();

            for _ in 0..max_sample_tries {
                if terminate_local.get_terminate() { return; }
                let s = self.float_vec_sampler_sample();
                if s.is_ok() {
                    let s_unwrap = s.unwrap();
                    let mut out_vec_write_unwrap = out_vec.write().unwrap();
                    out_vec_write_unwrap.push(s_unwrap);
                    drop(out_vec_write_unwrap);
                    let out_vec_read_unwrap = out_vec.read().unwrap();
                    if out_vec_read_unwrap.len() == target_num_samples {
                        terminate_local.set_to_terminate();
                    }
                }
            }
        });

        let mut out_vec_unwrap = out_vec.read().unwrap().to_vec();
        return Ok(out_vec_unwrap);
    }
    fn to_multi_float_vec_sampler_box(&self) -> MultiFloatVecSamplerBox {
        return MultiFloatVecSamplerBox(self.multi_float_vec_sampler_clone_box());
    }
}

pub trait MultiFloatVecSamplerClone {
    fn multi_float_vec_sampler_clone_box(&self) -> Box<dyn MultiFloatVecSampler>;
}
impl<T> MultiFloatVecSamplerClone for T where T: 'static + MultiFloatVecSampler + Clone {
    fn multi_float_vec_sampler_clone_box(&self) -> Box<dyn MultiFloatVecSampler> {
        Box::new(self.clone())
    }
}

pub trait AsMultiFloatVecSampler {
    fn as_multi_float_vec_sampler(&self) -> &MultiFloatVecSampler;
}
impl<T: MultiFloatVecSampler> AsMultiFloatVecSampler for T {
    fn as_multi_float_vec_sampler(&self) -> &MultiFloatVecSampler {
        self
    }
}

pub struct MultiFloatVecSamplerBox(Box<dyn MultiFloatVecSampler>);
impl MultiFloatVecSamplerBox {
    pub fn multi_float_vec_sampler_sample(&self, target_num_samples: usize, max_sample_tries: usize, parallel: bool) -> Result<Vec<DVector<f64>>, String> {
        return self.0.multi_float_vec_sampler_sample(target_num_samples, max_sample_tries, parallel);
    }
}
impl Clone for MultiFloatVecSamplerBox {
    fn clone(&self) -> Self {
        let c = self.0.multi_float_vec_sampler_clone_box();
        return Self(c);
    }
}
unsafe impl Send for MultiFloatVecSamplerBox { }
unsafe impl Sync for MultiFloatVecSamplerBox { }

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait LynxMultiFloatVecSampler: Send + Sync + LynxMultiFloatVecSamplerClone + LynxFloatVecSampler {
    fn lynx_multi_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric, target_num_samples: usize, max_sample_tries: usize, force_single_threaded: bool) -> Result<Vec<DVector<f64>>, String> {
        return match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => {
                self.lynx_multi_float_vec_sampler_sample_default_single_threaded(lynx_vars, target_num_samples, max_sample_tries)
            }
            LynxVarsGeneric::SingleThreadedMutRef(_) => {
                self.lynx_multi_float_vec_sampler_sample_default_single_threaded(lynx_vars, target_num_samples, max_sample_tries)
            }
            LynxVarsGeneric::Parallel(l) => {
                if force_single_threaded {
                    self.lynx_multi_float_vec_sampler_sample_default_single_threaded(lynx_vars, target_num_samples, max_sample_tries)
                } else {
                    self.lynx_multi_float_vec_sampler_sample_default_parallel(l, target_num_samples, max_sample_tries)
                }
            }
        }
    }
    fn lynx_multi_float_vec_sampler_sample_default_single_threaded(&self, lynx_vars: &mut LynxVarsGeneric, target_num_samples: usize, max_sample_tries: usize) -> Result<Vec<DVector<f64>>, String> {
        let mut out_vec = Vec::new();

        for _ in 0..max_sample_tries {
            let s = self.lynx_float_vec_sampler_sample(lynx_vars);
            if s.is_ok() {
                out_vec.push(s.ok().unwrap());
            }
            if out_vec.len() == target_num_samples { return Ok(out_vec); }
        }
        return Ok(out_vec);
    }
    fn lynx_multi_float_vec_sampler_sample_default_parallel(&self, lynx_vars: &mut LynxVarsParallel, target_num_samples: usize, max_sample_tries: usize) -> Result<Vec<DVector<f64>>, String> {
        let mut out_vec = RwLock::new(Vec::new());
        let mut terminate = TerminationUtil::new();

        let iter = lynx_vars.get_par_iter_mut();
        iter.for_each(|x| {
            let mut lynx_vars_generic = LynxVarsGeneric::SingleThreadedMutRef(x);
            let mut terminate_local = terminate.clone();

            for _ in 0..max_sample_tries {
                if terminate_local.get_terminate() { return; }
                let s = self.lynx_float_vec_sampler_sample(&mut lynx_vars_generic);
                if s.is_ok() {
                    let s_unwrap = s.unwrap();
                    out_vec.write().unwrap().push(s_unwrap);
                    let out_vec_read_unwrap = out_vec.read().unwrap();
                    if out_vec_read_unwrap.len() >= target_num_samples {
                        terminate_local.set_to_terminate();
                    }
                }
            }
        });


        let mut out_vec_unwrap = out_vec.read().unwrap().to_vec();
        return Ok(out_vec_unwrap);
    }
    fn to_lynx_multi_float_vec_sampler_box(&self) -> LynxMultiFloatVecSamplerBox {
        return LynxMultiFloatVecSamplerBox(self.lynx_multi_float_vec_sampler_clone_box());
    }
}

pub trait LynxMultiFloatVecSamplerClone {
    fn lynx_multi_float_vec_sampler_clone_box(&self) -> Box<dyn LynxMultiFloatVecSampler>;
}
impl<T> LynxMultiFloatVecSamplerClone for T where T: 'static + LynxMultiFloatVecSampler + Clone {
    fn lynx_multi_float_vec_sampler_clone_box(&self) -> Box<dyn LynxMultiFloatVecSampler> {
        Box::new(self.clone())
    }
}

pub trait AsLynxMultiFloatVecSampler {
    fn as_lynx_multi_float_vec_sampler(&self) -> &LynxMultiFloatVecSampler;
}
impl<T: LynxMultiFloatVecSampler> AsLynxMultiFloatVecSampler for T {
    fn as_lynx_multi_float_vec_sampler(&self) -> &LynxMultiFloatVecSampler {
        self
    }
}

pub struct LynxMultiFloatVecSamplerBox(Box<dyn LynxMultiFloatVecSampler>);
impl LynxMultiFloatVecSamplerBox {
    pub fn new(lynx_multi_float_vec_sampler: &dyn LynxMultiFloatVecSampler) -> Self {
        return Self( lynx_multi_float_vec_sampler.lynx_multi_float_vec_sampler_clone_box() );
    }

    pub fn lynx_multi_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric, target_num_samples: usize, max_sample_tries: usize, force_single_threaded: bool) -> Result<Vec<DVector<f64>>, String> {
        return self.0.lynx_multi_float_vec_sampler_sample(lynx_vars, target_num_samples, max_sample_tries, force_single_threaded);
    }
}
impl Clone for LynxMultiFloatVecSamplerBox {
    fn clone(&self) -> Self {
        let c = self.0.lynx_multi_float_vec_sampler_clone_box();
        return Self(c);
    }
}
unsafe impl Send for LynxMultiFloatVecSamplerBox { }
unsafe impl Sync for LynxMultiFloatVecSamplerBox { }