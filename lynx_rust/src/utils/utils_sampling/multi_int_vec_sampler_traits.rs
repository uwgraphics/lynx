use crate::utils::utils_vars::{prelude::*};
use crate::utils::utils_sampling::{int_vec_sampler_traits::*};
use crate::utils::utils_runtime_management::termination_util::TerminationUtil;
use nalgebra::DVector;
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use std::marker::{Sync, Send};
use std::sync::RwLock;
use rayon::prelude::*;


pub trait MultiIntVecSampler: Send + Sync + MultiIntVecSamplerClone + IntVecSampler {
    fn multi_int_vec_sampler_sample(&self, target_num_samples: usize, max_sample_tries: usize, parallel: bool) -> Result<Vec<DVector<i32>>, String> {
        return if parallel {
            self.multi_int_vec_sampler_sample_default_parallel(target_num_samples, max_sample_tries)
        } else {
            self.multi_int_vec_sampler_sample_default_single_threaded(target_num_samples, max_sample_tries)
        }
    }
    fn multi_int_vec_sampler_sample_default_single_threaded(&self, target_num_samples: usize, max_sample_tries: usize) -> Result<Vec<DVector<i32>>, String> {
        let mut out_vec = Vec::new();

        for _ in 0..max_sample_tries {
            let s = self.int_vec_sampler_sample();
            if s.is_ok() {
                out_vec.push(s.ok().unwrap());
            }
            if out_vec.len() == target_num_samples { return Ok(out_vec); }
        }
        return Ok(out_vec);
    }
    fn multi_int_vec_sampler_sample_default_parallel(&self, target_num_samples: usize, max_sample_tries: usize) -> Result<Vec<DVector<i32>>, String> {
        let mut out_vec = RwLock::new(Vec::new());
        let mut terminate = TerminationUtil::new();

        let a : Vec<usize> = (0..num_cpus::get()).collect();
        a.par_iter().for_each(|x| {
            let mut terminate_local = terminate.clone();

            for _ in 0..max_sample_tries {
                if terminate_local.get_terminate() { return; }
                let s = self.int_vec_sampler_sample();
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
    fn to_multi_int_vec_sampler_box(&self) -> MultiIntVecSamplerBox {
        return MultiIntVecSamplerBox(self.multi_int_vec_sampler_clone_box());
    }
}

pub trait MultiIntVecSamplerClone {
    fn multi_int_vec_sampler_clone_box(&self) -> Box<dyn MultiIntVecSampler>;
}
impl<T> MultiIntVecSamplerClone for T where T: 'static + MultiIntVecSampler + Clone {
    fn multi_int_vec_sampler_clone_box(&self) -> Box<dyn MultiIntVecSampler> {
        Box::new(self.clone())
    }
}

pub trait AsMultiIntVecSampler {
    fn as_multi_int_vec_sampler(&self) -> &MultiIntVecSampler;
}
impl<T: MultiIntVecSampler> AsMultiIntVecSampler for T {
    fn as_multi_int_vec_sampler(&self) -> &MultiIntVecSampler {
        self
    }
}

pub struct MultiIntVecSamplerBox(Box<dyn MultiIntVecSampler>);
impl MultiIntVecSamplerBox {
    pub fn multi_int_vec_sampler_sample(&self, target_num_samples: usize, max_sample_tries: usize, parallel: bool) -> Result<Vec<DVector<i32>>, String> {
        return self.0.multi_int_vec_sampler_sample(target_num_samples, max_sample_tries, parallel);
    }
}
impl Clone for MultiIntVecSamplerBox {
    fn clone(&self) -> Self {
        let c = self.0.multi_int_vec_sampler_clone_box();
        return Self(c);
    }
}
unsafe impl Send for MultiIntVecSamplerBox { }
unsafe impl Sync for MultiIntVecSamplerBox { }

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait LynxMultiIntVecSampler: Send + Sync + LynxMultiIntVecSamplerClone + LynxIntVecSampler {
    fn lynx_multi_int_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric, target_num_samples: usize, max_sample_tries: usize, force_single_threaded: bool) -> Result<Vec<DVector<i32>>, String> {
        return match lynx_vars {
            LynxVarsGeneric::SingleThreaded(_) => {
                self.lynx_multi_int_vec_sampler_sample_default_single_threaded(lynx_vars, target_num_samples, max_sample_tries)
            }
            LynxVarsGeneric::SingleThreadedMutRef(_) => {
                self.lynx_multi_int_vec_sampler_sample_default_single_threaded(lynx_vars, target_num_samples, max_sample_tries)
            }
            LynxVarsGeneric::Parallel(l) => {
                if force_single_threaded {
                    self.lynx_multi_int_vec_sampler_sample_default_single_threaded(lynx_vars, target_num_samples, max_sample_tries)
                } else {
                    self.lynx_multi_int_vec_sampler_sample_default_parallel(l, target_num_samples, max_sample_tries)
                }
            }
        }
    }
    fn lynx_multi_int_vec_sampler_sample_default_single_threaded(&self, lynx_vars: &mut LynxVarsGeneric, target_num_samples: usize, max_sample_tries: usize) -> Result<Vec<DVector<i32>>, String> {
        let mut out_vec = Vec::new();

        for _ in 0..max_sample_tries {
            let s = self.lynx_int_vec_sampler_sample(lynx_vars);
            if s.is_ok() {
                out_vec.push(s.ok().unwrap());
            }
            if out_vec.len() == target_num_samples { return Ok(out_vec); }
        }
        return Ok(out_vec);
    }
    fn lynx_multi_int_vec_sampler_sample_default_parallel(&self, lynx_vars: &mut LynxVarsParallel, target_num_samples: usize, max_sample_tries: usize) -> Result<Vec<DVector<i32>>, String> {
        let mut out_vec = RwLock::new(Vec::new());
        let mut terminate = TerminationUtil::new();

        let iter = lynx_vars.get_par_iter_mut();
        iter.for_each(|x| {
            let mut lynx_vars_generic = LynxVarsGeneric::SingleThreadedMutRef(x);
            let mut terminate_local = terminate.clone();

            for _ in 0..max_sample_tries {
                if terminate_local.get_terminate() { return; }
                let s = self.lynx_int_vec_sampler_sample(&mut lynx_vars_generic);
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
    fn to_lynx_multi_int_vec_sampler_box(&self) -> LynxMultiIntVecSamplerBox {
        return LynxMultiIntVecSamplerBox(self.lynx_multi_int_vec_sampler_clone_box());
    }
}

pub trait LynxMultiIntVecSamplerClone {
    fn lynx_multi_int_vec_sampler_clone_box(&self) -> Box<dyn LynxMultiIntVecSampler>;
}
impl<T> LynxMultiIntVecSamplerClone for T where T: 'static + LynxMultiIntVecSampler + Clone {
    fn lynx_multi_int_vec_sampler_clone_box(&self) -> Box<dyn LynxMultiIntVecSampler> {
        Box::new(self.clone())
    }
}

pub trait AsLynxMultiIntVecSampler {
    fn as_lynx_multi_int_vec_sampler(&self) -> &LynxMultiIntVecSampler;
}
impl<T: LynxMultiIntVecSampler> AsLynxMultiIntVecSampler for T {
    fn as_lynx_multi_int_vec_sampler(&self) -> &LynxMultiIntVecSampler {
        self
    }
}

pub struct LynxMultiIntVecSamplerBox(Box<dyn LynxMultiIntVecSampler>);
impl LynxMultiIntVecSamplerBox {
    pub fn lynx_multi_int_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric, target_num_samples: usize, max_sample_tries: usize, force_single_threaded: bool) -> Result<Vec<DVector<i32>>, String> {
        return self.0.lynx_multi_int_vec_sampler_sample(lynx_vars, target_num_samples, max_sample_tries, force_single_threaded);
    }
}
impl Clone for LynxMultiIntVecSamplerBox {
    fn clone(&self) -> Self {
        let c = self.0.lynx_multi_int_vec_sampler_clone_box();
        return Self(c);
    }
}
unsafe impl Send for LynxMultiIntVecSamplerBox { }
unsafe impl Sync for LynxMultiIntVecSamplerBox { }