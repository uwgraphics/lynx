use crate::utils::utils_vars::{lynx_vars::LynxVars, lynx_vars_generic::LynxVarsGeneric};
use rayon::prelude::*;
use num_cpus;
use std::sync::Mutex;

#[derive(Clone, Debug)]
pub struct LynxVarsParallel {
    _lynx_vars_vec: Vec<LynxVars>
}

impl LynxVarsParallel {
    pub fn new_empty(num_threads: Option<usize>) -> Self {
        let mut _num_threads = num_cpus::get();
        if num_threads.is_some() {
            _num_threads = num_threads.unwrap();
        }
        if _num_threads == 0 { _num_threads = 1; }

        let mut _lynx_vars_vec = Vec::new();
        for _ in 0.._num_threads {
            _lynx_vars_vec.push( LynxVars::new_empty() );
        }
        return Self { _lynx_vars_vec };
    }

    pub fn new_with_parallel_option(lynx_vars: &LynxVars, num_threads: Option<usize>, in_parallel: bool) -> Self {
        return if in_parallel { Self::new_in_parallel(lynx_vars, num_threads ) } else { Self::new(lynx_vars, num_threads) }
    }

    pub fn new(lynx_vars: &LynxVars, num_threads: Option<usize>) -> Self {
        let mut _num_threads = num_cpus::get();
        if num_threads.is_some() {
            _num_threads = num_threads.unwrap();
        }
        if _num_threads == 0 { _num_threads = 1; }

        let mut _lynx_vars_vec = Vec::new();
        for _ in 0.._num_threads {
            _lynx_vars_vec.push( lynx_vars.clone() );
        }
        return Self { _lynx_vars_vec };
    }

    pub fn new_in_parallel(lynx_vars: &LynxVars, num_threads: Option<usize>) -> Self {
        let mut _num_threads = num_cpus::get();
        if num_threads.is_some() {
            _num_threads = num_threads.unwrap();
        }
        if _num_threads == 0 { _num_threads = 1; }

        let a: Vec<usize> = (0.._num_threads).collect();

        let mut _lynx_vars_vec = Mutex::new(Vec::new());
        a.par_iter().for_each(|x| {
            let clone = lynx_vars.clone();
            _lynx_vars_vec.lock().unwrap().push(clone);
        });

        let out_lynx_vars_vec = _lynx_vars_vec.lock().unwrap().to_vec();

        return Self { _lynx_vars_vec: out_lynx_vars_vec };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_par_iter_mut(&mut self) -> rayon::slice::IterMut<LynxVars> {
        return self._lynx_vars_vec.par_iter_mut();
    }

    pub fn get_iter_mut(&mut self) -> std::slice::IterMut<LynxVars> {
        return self._lynx_vars_vec.iter_mut();
    }

    pub fn get_first_mut_ref(&mut self) -> &mut LynxVars {
        return &mut self._lynx_vars_vec[0];
    }

    pub fn get_first_ref(&self) -> &LynxVars {
        return &self._lynx_vars_vec[0];
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_num_threads(&self) -> usize {
        return self._lynx_vars_vec.len();
    }

    pub fn print_num_threads(&self) {
        println!("{:?}", self.get_num_threads());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn first_contains_variable(&self, var_type: &'static str, var_name: &'static str) -> bool {
        return self._lynx_vars_vec[0].contains_variable(var_type, var_name);
    }

    pub fn all_contain_variable(&self, var_type: &'static str, var_name: &'static str) -> bool {
        let num_threads = self.get_num_threads();
        for i in 0..num_threads {
            if !(self._lynx_vars_vec[i].contains_variable(var_type, var_name)) { return false; }
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print(&self) {
        let l = self.get_num_threads();
        for i in 0..l {
            println!("{:?} --- > {:?}", i, self._lynx_vars_vec[i]);
        }
    }

}