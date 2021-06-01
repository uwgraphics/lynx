use crate::utils::utils_optimization::{objective_function::*, isolated_objective_term::*};
use crate::utils::utils_vars::prelude::*;
use crate::robot_modules::robot::Robot;
use crate::utils::utils_recorders::prelude::*;
use optimization_engine::{constraints::*, panoc::*, *};
use nlopt::*;
use nalgebra::DVector;
use std::time::{Duration, Instant};
use crate::utils::utils_math::nalgebra_utils::vec_to_dvec;
use std::sync::{Mutex, Arc};
use termion::{color, style};
use std::ops::Deref;

pub trait NonlinearOptimizationEngine: LynxVarsUser + AsLynxVarsUser + Send + Sync + NonlinearOptimizationEngineClone {
    fn optimize(&self, initial_condition: &DVector<f64>, objective_function: &ObjectiveFunction, lynx_vars: &mut LynxVarsGeneric, max_iter: Option<usize>, max_evalution_time: Option<f64>, recorder: &RecorderArcMutexOption, debug: bool) -> Result<NonlinearOptimizationResult, String>;
    fn to_nonlinear_optimization_engine_box(&self) -> NonlinearOptimizationEngineBox {
        return NonlinearOptimizationEngineBox(self.clone_box());
    }
}

pub trait NonlinearOptimizationEngineClone {
    fn clone_box(&self) -> Box<dyn NonlinearOptimizationEngine>;
}
impl<T> NonlinearOptimizationEngineClone for T where T: 'static + NonlinearOptimizationEngine + Clone {
    fn clone_box(&self) -> Box<dyn NonlinearOptimizationEngine> {
        Box::new(self.clone())
    }
}

pub struct NonlinearOptimizationEngineBox(Box<dyn NonlinearOptimizationEngine>);
impl NonlinearOptimizationEngineBox {
    pub fn optimize(&self, initial_condition: &DVector<f64>, objective_function: &ObjectiveFunction, lynx_vars: &mut LynxVarsGeneric, max_iter: Option<usize>, max_evalution_time: Option<f64>, recorder: &RecorderArcMutexOption, debug: bool) -> Result<NonlinearOptimizationResult, String> {
        return self.0.optimize(initial_condition, objective_function, lynx_vars, max_iter, max_evalution_time, recorder, debug);
    }
}
impl Clone for NonlinearOptimizationEngineBox {
    fn clone(&self) -> Self {
        let c = self.0.clone_box();
        return Self(c);
    }
}
unsafe impl Send for NonlinearOptimizationEngineBox { }
unsafe impl Sync for NonlinearOptimizationEngineBox { }

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct NLoptNonlinearOptimizationEngine {
    _algorithm: Algorithm
}
impl NLoptNonlinearOptimizationEngine {
    pub fn new(algorithm: Algorithm) -> Self {
        Self {_algorithm: algorithm}
    }

    pub fn new_slsqp() -> Self {
        NLoptNonlinearOptimizationEngine::new(Algorithm::Slsqp)
    }

    pub fn new_mma() -> Self {
        NLoptNonlinearOptimizationEngine::new(Algorithm::Mma)
    }

    pub fn new_cobyla() -> Self { NLoptNonlinearOptimizationEngine::new(Algorithm::Cobyla) }

    pub fn new_bobyqa() -> Self { NLoptNonlinearOptimizationEngine::new(Algorithm::Bobyqa) }

    pub fn new_ccsaq() -> Self { NLoptNonlinearOptimizationEngine::new(Algorithm::Ccsaq) }
}
impl NonlinearOptimizationEngine for NLoptNonlinearOptimizationEngine {
    fn optimize(&self, initial_condition: &DVector<f64>, objective_function: &ObjectiveFunction, lynx_vars: &mut LynxVarsGeneric, max_iter: Option<usize>, max_evalution_time: Option<f64>, recorder: &RecorderArcMutexOption, debug: bool) -> Result<NonlinearOptimizationResult, String> {
        let mut output = NonlinearOptimizationOutput::new_empty();

        let mut lynx_vars_mutex = Mutex::new( lynx_vars );
        let mut objective_count = Mutex::new(0 as usize);
        let mut gradient_count = Mutex::new(0 as usize);

        let obj_f = |x: &[f64], _gradient: Option<&mut [f64]>, _params: &mut ()| -> f64 {
            let mut objective_count_unwrap = objective_count.lock().unwrap();
            *objective_count_unwrap += 1;

            let mut lynx_vars_unwrap = lynx_vars_mutex.lock().unwrap();

            let objective_result = objective_function.call( &vec_to_dvec(&x.to_vec()), *lynx_vars_unwrap, recorder, debug);
            if objective_result.is_err() {
                println!("{}{} ERROR: problem on objective_function.call(...) in NLoptNonlinearOptimizationEngine.  Error was {}. The NonlinearOptimizationEngine is about to fail. {}", color::Fg(color::Red), style::Bold, objective_result.as_ref().err().unwrap(), style::Reset);
            }
            let objective_result_unwrap = objective_result.as_ref().ok().unwrap();
            if debug {
                println!("{}{}Objective result {} on x vector {:?} ---> {}", color::Fg(color::LightCyan), style::Bold, objective_count_unwrap, x, style::Reset);
                objective_result_unwrap.print_summary();
            }

            if _gradient.is_some() {
                let mut gradient_count_unwrap = gradient_count.lock().unwrap();
                *gradient_count_unwrap += 1;

                let gradient_result = objective_function.gradient(&vec_to_dvec(&x.to_vec()), *lynx_vars_unwrap, recorder, debug );
                if gradient_result.is_err() {
                    println!("{}{} ERROR: problem on objective_function.gradient(...) in NLoptNonlinearOptimizationEngine.  Error was {}. The NonlinearOptimizationEngine is about to fail. {}", color::Fg(color::Red), style::Bold, gradient_result.as_ref().err().unwrap(), style::Reset);
                }
                let gradient_result_unwrap = gradient_result.as_ref().ok().unwrap();
                if debug {
                    println!("{}{}Gradient result {} on x vector {:?} ---> {}", color::Fg(color::LightMagenta), style::Bold, gradient_count_unwrap, x, style::Reset);
                    gradient_result_unwrap.print_summary();
                }

                let my_grad = gradient_result_unwrap.get_gradient();

                let g = _gradient.unwrap();

                let l = my_grad.len();
                for i in 0..l {
                    g[i] = my_grad[i];
                }
            }
            objective_result_unwrap.get_obj_val()
        };

        let mut lynx_vars_unwrap = lynx_vars_mutex.lock().unwrap();

        type f64VecOption = Option<Vec<f64>>;
        let upper_bounds_option = get_lynx_var_ref_generic!(lynx_vars_unwrap.deref(), f64VecOption, "upper_bounds_option")?;
        let lower_bounds_option = get_lynx_var_ref_generic!(lynx_vars_unwrap.deref(), f64VecOption, "lower_bounds_option")?;

        let mut opt = Nlopt::new(self._algorithm, initial_condition.len(), obj_f, Target::Minimize, ());
        opt.set_ftol_rel(0.0001);
        opt.set_ftol_abs(0.0001);
        opt.set_xtol_rel(0.0001);
        opt.set_maxeval(30 as u32);
        if max_iter.is_some() {
            opt.set_maxeval(max_iter.unwrap() as u32);
        }
        if max_evalution_time.is_some() {
            opt.set_maxtime(max_evalution_time.unwrap());
        }
        if upper_bounds_option.is_some() {
            opt.set_upper_bounds(upper_bounds_option.as_ref().unwrap().as_slice());
        }
        if lower_bounds_option.is_some() {
            opt.set_lower_bounds(lower_bounds_option.as_ref().unwrap().as_slice());
        }

        let mut x = initial_condition.clone();
        type DVectorF64 = DVector<f64>;
        set_or_add_lynx_var_generic!(lynx_vars_unwrap.deref_mut(), DVectorF64, "initial_condition", x.clone());
        drop(lynx_vars_unwrap);
        let res = opt.optimize(x.as_mut_slice());
        output._evaluation_time = output._start_time.elapsed();
        output._num_objective_calls = objective_count.lock().unwrap().clone();
        output._num_gradient_calls = gradient_count.lock().unwrap().clone();

        match res {
            Ok(s) => {
                output._f_val = Some(s.1);
                output._x_star = Some(x.clone());

                return Ok(NonlinearOptimizationResult::Success(output));

            } Err(s) => {
                output._f_val = Some(s.1);
                return Ok(NonlinearOptimizationResult::Failure(output));
            }
        }
    }
}
impl LynxVarsUser for NLoptNonlinearOptimizationEngine {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("f64VecOption", "upper_bounds_option"),
                     ("f64VecOption", "lower_bounds_option"),
                     ("DVectorF64", "initial_condition") ]
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct OpenNonlinearOptimizationEngine;
impl NonlinearOptimizationEngine for OpenNonlinearOptimizationEngine {
    fn optimize(&self, initial_condition: &DVector<f64>, objective_function: &ObjectiveFunction, lynx_vars: &mut LynxVarsGeneric, max_iter: Option<usize>, max_evalution_time: Option<f64>, recorder: &RecorderArcMutexOption, debug: bool) -> Result<NonlinearOptimizationResult, String> {
        let mut output = NonlinearOptimizationOutput::new_empty();
        let dim = initial_condition.len();

        let mut cache = PANOCCache::new(dim, 1e-14, 16);

        let mut lynx_vars_mutex = Mutex::new( lynx_vars );
        let mut objective_count = Mutex::new(0 as usize);
        let mut gradient_count = Mutex::new(0 as usize);

        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let mut gradient_count_unwrap = gradient_count.lock().unwrap();
            *gradient_count_unwrap += 1;

            let mut lynx_vars_unwrap = lynx_vars_mutex.lock().unwrap();

            let gradient_result = objective_function.gradient(&vec_to_dvec(&u.to_vec()), *lynx_vars_unwrap, recorder, debug);
            if gradient_result.is_err() {
                println!("{}{} ERROR: problem on objective_function.gradient(...) in OpenNonlinearOptimizationEngine.  Error was {}. The NonlinearOptimizationEngine is about to fail. {}", color::Fg(color::Red), style::Bold, gradient_result.as_ref().err().unwrap(), style::Reset);
                return Err(SolverError::Cost);
            }
            let gradient_result_unwrap = gradient_result.as_ref().ok().unwrap();
            if debug {
                println!("{}{}Gradient result {} on x vector {:?} ---> {}", color::Fg(color::LightMagenta), style::Bold, gradient_count_unwrap, u, style::Reset);
                gradient_result_unwrap.print_summary();
            }

            let my_grad = gradient_result_unwrap.get_gradient();

            let l = my_grad.len();
            for i in 0..l {
                grad[i] = my_grad[i];
            }

            Ok(())
        };

        let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            let mut objective_count_unwrap = objective_count.lock().unwrap();
            *objective_count_unwrap += 1;

            let mut lynx_vars_unwrap = lynx_vars_mutex.lock().unwrap();

            let objective_result = objective_function.call( &vec_to_dvec(&u.to_vec()), *lynx_vars_unwrap, recorder, debug );
            if objective_result.is_err() {
                println!("{}{} ERROR: problem on objective_function.call(...) in OpenNonlinearOptimizationEngine.  Error was {}. The NonlinearOptimizationEngine is about to fail. {}", color::Fg(color::Red), style::Bold, objective_result.as_ref().err().unwrap(), style::Reset);
            }
            let objective_result_unwrap = objective_result.as_ref().ok().unwrap();
            if debug {
                println!("{}{}Objective result {} on x vector {:?} ---> {}", color::Fg(color::LightCyan), style::Bold, objective_count_unwrap, u, style::Reset);
                objective_result_unwrap.print_summary();
            }

            *c = objective_result_unwrap.get_obj_val();

            Ok(())
        };

        let mut lynx_vars_unwrap = lynx_vars_mutex.lock().unwrap();

        type f64VecOption = Option<Vec<f64>>;
        let upper_bounds_option = get_lynx_var_ref_generic!(lynx_vars_unwrap.deref(), f64VecOption, "upper_bounds_option")?;
        let lower_bounds_option = get_lynx_var_ref_generic!(lynx_vars_unwrap.deref(), f64VecOption, "lower_bounds_option")?;
        let lower_bounds = lower_bounds_option.clone();
        let upper_bounds = upper_bounds_option.clone();

        let tmp_lower = vec![-std::f64::INFINITY; dim];
        let tmp_upper = vec![std::f64::INFINITY; dim];

        let mut bounds = Rectangle::new( Some( tmp_lower.as_slice() ), Some( tmp_upper.as_slice() ) );
        if lower_bounds.is_some() && upper_bounds.is_some() {
            bounds = Rectangle::new( Option::from(lower_bounds.as_ref().unwrap().as_slice()), Option::from(upper_bounds.as_ref().unwrap().as_slice()) );
        } else if lower_bounds.is_some() {
            bounds = Rectangle::new( Option::from(lower_bounds.as_ref().unwrap().as_slice()), None );
        } else if upper_bounds.is_some() {
            bounds = Rectangle::new( None, Option::from(upper_bounds.as_ref().unwrap().as_slice()) );
        }

        let problem = Problem::new(&bounds, df, f);

        let mut max_iter_ = 30; if max_iter.is_some() { max_iter_ = max_iter.unwrap(); }
        let mut max_evaluation_time_ = 99999999.9; if max_evalution_time.is_some() { max_evaluation_time_ = max_evalution_time.unwrap(); }
        let mut panoc = PANOCOptimizer::new(problem, &mut cache).with_max_iter(max_iter_).with_tolerance(0.0005).with_max_duration( Duration::from_secs_f64( max_evaluation_time_ ) );

        let mut x = initial_condition.clone();
        type DVectorF64 = DVector<f64>;
        set_or_add_lynx_var_generic!(lynx_vars_unwrap.deref_mut(), DVectorF64, "initial_condition", x.clone());
        drop(lynx_vars_unwrap);
        let status = panoc.solve(x.as_mut_slice());
        output._evaluation_time = output._start_time.elapsed();
        output._num_objective_calls = objective_count.lock().unwrap().clone();
        output._num_gradient_calls = gradient_count.lock().unwrap().clone();

        match status {
            Ok(s) => {
                output._x_star = Some(x);
                output._f_val = Some(s.cost_value());
                return Ok(NonlinearOptimizationResult::Success(output));
            },
            Err(s) => {
                return Ok(NonlinearOptimizationResult::Failure(output));
            }
        }
    }
}
impl LynxVarsUser for OpenNonlinearOptimizationEngine {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return vec![ ("f64VecOption", "upper_bounds_option"),
                     ("f64VecOption", "lower_bounds_option"),
                     ("DVectorF64", "initial_condition") ]
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Debug, Clone)]
pub enum NonlinearOptimizationResult {
    Success(NonlinearOptimizationOutput),
    Failure(NonlinearOptimizationOutput)
}
impl NonlinearOptimizationResult {
    pub fn print_summary(&self) {
        println!("\n{}{}Optimization Result Summary ---> {}", color::Fg(color::Blue), style::Bold, style::Reset);
        match self {
            NonlinearOptimizationResult::Success(s) => {
                println!("{}{}Optimization succeeded! {}", color::Fg(color::Green), style::Bold, style::Reset);
                println!("{}    evaluation time: {:?} {}", color::Fg(color::Green), s._evaluation_time, style::Reset);
                println!("{}    num objective calls: {:?} {}", color::Fg(color::Green), s._num_objective_calls, style::Reset);
                println!("{}    num gradient calls: {:?} {}", color::Fg(color::Green), s._num_gradient_calls, style::Reset);
                println!("{}    x_star: {:?} {}", color::Fg(color::Green), s._x_star, style::Reset);
                println!("{}    f_val: {:?} {}", color::Fg(color::Green), s._f_val, style::Reset);
            },
            NonlinearOptimizationResult::Failure(s) => {
                println!("{}{}Optimization failed {}", color::Fg(color::Red), style::Bold, style::Reset);
                println!("{}    evaluation time: {:?} {}", color::Fg(color::Red), s._evaluation_time, style::Reset);
                println!("{}    num objective calls: {:?} {}", color::Fg(color::Green), s._num_objective_calls, style::Reset);
                println!("{}    num gradient calls: {:?} {}", color::Fg(color::Green), s._num_gradient_calls, style::Reset);
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct NonlinearOptimizationOutput {
    _start_time: Instant,
    _num_objective_calls: usize,
    _num_gradient_calls: usize,
    _evaluation_time: Duration,
    _x_star: Option<DVector<f64>>,
    _f_val: Option<f64>
}
impl NonlinearOptimizationOutput {
    pub fn new_empty() -> Self {
        let _start_time = Instant::now();
        Self { _start_time, _num_objective_calls: 0, _num_gradient_calls: 0, _evaluation_time: _start_time.elapsed(), _x_star: None, _f_val: None}
    }

    pub fn get_evaluation_time(&self) -> &Duration {
        return &self._evaluation_time;
    }

    pub fn get_x_star(&self) -> &Option<DVector<f64>> {
        return &self._x_star;
    }

    pub fn get_f_val(&self) -> &Option<f64> {
        return &self._f_val;
    }

    pub fn get_num_objective_calls(&self) -> usize {
        return self._num_gradient_calls;
    }

    pub fn get_num_gradient_calls(&self) -> usize { return self._num_gradient_calls; }
}