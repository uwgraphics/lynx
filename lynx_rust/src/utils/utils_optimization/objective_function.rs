use crate::utils::utils_vars::prelude::*;
use crate::utils::utils_optimization::{loss_function::*, isolated_objective_term::*};
use crate::robot_modules::robot::Robot;
use crate::utils::utils_recorders::prelude::*;
use nalgebra::{DVector};
use termion::{color, style};
use std::collections::HashMap;
use std::clone::Clone;
use std::sync::{Arc, Mutex};

pub struct ObjectiveFunction {
    _isolated_objective_terms: Vec<IsolatedObjectiveTermBox>,
    _loss_functions: Vec<Box<dyn LossFunction>>,
    _weights: Vec<f64>
}

impl ObjectiveFunction {
    pub fn new( isolated_objective_terms: Vec<IsolatedObjectiveTermBox>, weights: Option<Vec<Option<f64>>>) -> Result<Self, String> {
        if weights.is_some() && weights.as_ref().unwrap().len() != isolated_objective_terms.len() {
            return Err(format!("isolated_objective_terms and wegiths must have same number of inputs (right now it's {:?} and {:?})", isolated_objective_terms.len(), weights.as_ref().unwrap().len()));
        }

        let mut _loss_functions = Vec::new();
        let l = isolated_objective_terms.len();
        for i in 0..l {
            _loss_functions.push( isolated_objective_terms[i].get_default_loss_function() );
        }

        let mut _weights = Vec::new();
        match weights {
            Some(weights_vec) => {
                for i in 0..l {
                    if weights_vec[i].is_some() { _weights.push(weights_vec[i].unwrap()) }
                    else { _weights.push(1.0); }
                }
            },
            None => _weights = vec![1.0; l]
        }

        return Ok( Self { _isolated_objective_terms: isolated_objective_terms, _loss_functions, _weights } );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn call(&self, x: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, debug: bool) -> Result<ObjectiveFunctionCallOutput, String> {
        let mut output = ObjectiveFunctionCallOutput::new_emtpy(debug);

        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            let isolated_objective_term_val_before_loss_function = self._isolated_objective_terms[i].call(x, lynx_vars, recorder)?;
            let isolated_objective_term_value_before_weight = self._loss_functions[i].loss(isolated_objective_term_val_before_loss_function);
            let isolated_objective_term_value = self._weights[i] * isolated_objective_term_value_before_weight;
            if debug {
                output.add_debug_entry(self._isolated_objective_terms[i].name(), isolated_objective_term_value, isolated_objective_term_value_before_weight, isolated_objective_term_val_before_loss_function);
            }
            output._obj_val += isolated_objective_term_value;
        }

        return Ok(output);
    }

    pub fn gradient(&self, x: &DVector<f64>, lynx_vars: &mut LynxVarsGeneric, recorder: &RecorderArcMutexOption, debug: bool) -> Result<ObjectiveFunctionGradientOutput, String> {
        let mut output = ObjectiveFunctionGradientOutput::new_emtpy(x.len(), debug);

        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            let isolated_objective_term_gradient_before_loss_function = self._isolated_objective_terms[i].gradient(x, lynx_vars, recorder)?;

            let isolated_objective_term_val_before_loss_function = self._isolated_objective_terms[i].call(x, lynx_vars, recorder)?;
            let s = self._loss_functions[i].derivative(  isolated_objective_term_val_before_loss_function  );

            let isolated_objective_term_gradient_before_weight= s * &isolated_objective_term_gradient_before_loss_function;
            let isolated_objective_term_gradient= self._weights[i] * &isolated_objective_term_gradient_before_weight;

            if debug {
                output.add_debug_entry( self._isolated_objective_terms[i].name(), &isolated_objective_term_gradient, &isolated_objective_term_gradient_before_weight, &isolated_objective_term_gradient_before_loss_function );
            }

            output._gradient += &isolated_objective_term_gradient;
        }

        return Ok(output);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_diagnostics_for_all_isolated_objective_terms(&self, lynx_vars: &mut LynxVarsGeneric, x: &Option<Vec<f64>>) {
        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            self._isolated_objective_terms[i].print_diagnostics_information(lynx_vars, &x);
        }
    }

    pub fn add_isolated_objective_term(&mut self, isolated_objective_term: IsolatedObjectiveTermBox, weight: Option<f64>) {
        self._loss_functions.push( isolated_objective_term.get_default_loss_function() );
        self._isolated_objective_terms.push(isolated_objective_term);
        if weight.is_none() { self._weights.push(1.0); }
        else { self._weights.push( weight.unwrap() ); }
    }

    pub fn remove_all_isolated_objective_terms_with_given_name(&mut self, isolated_objective_term_name: &str) {
        let mut remove_idxs = Vec::new();
        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            if &self._isolated_objective_terms[i].name() == isolated_objective_term_name {
                remove_idxs.insert(0, i);
            }
        }

        let l = remove_idxs.len();
        for i in 0..l {
            self._isolated_objective_terms.remove( remove_idxs[i] );
            self._loss_functions.remove( remove_idxs[i] );
            self._weights.remove( remove_idxs[i] );
        }
    }

    pub fn remove_only_first_isolated_objective_term_with_given_name(&mut self, isolated_objective_term_name: &str) {
        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            if &self._isolated_objective_terms[i].name() == isolated_objective_term_name {
                self._isolated_objective_terms.remove( i );
                self._loss_functions.remove( i );
                self._weights.remove( i );
                return;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn update_weight(&mut self, objective_name: &String, new_weight: f64) -> Result<(), String> {
        // let idx = self._objective_term_to_idx_hashmap.get(objective_name);
        let idx = self._get_first_idx_from_name(objective_name);
        if idx.is_none() { return Err(format!("objective name {} not found in ObjectiveFunction", objective_name)); }

        return self.update_weight_via_idx(idx.unwrap(), new_weight);
    }

    pub fn update_weight_via_idx(&mut self, idx: usize, new_weight: f64) -> Result<(), String> {
        if idx >= self._weights.len() {
            return Err(format!("idx too high for weight update in ObjectiveFunction ({:?} with _weights length of {:?})", idx, self._weights.len()))
        }

        self._weights[idx] = new_weight;

        return Ok(());
    }

    pub fn update_loss_function(&mut self, objective_name: &String, new_loss_function: Box<dyn LossFunction>) -> Result<(), String> {
        // let idx = self._objective_term_to_idx_hashmap.get(objective_name);
        let idx = self._get_first_idx_from_name(objective_name);
        if idx.is_none() { return Err(format!("objective name {} not found in ObjectiveFunction", objective_name)); }

        return self.update_loss_function_via_idx(idx.unwrap(), new_loss_function);
    }

    pub fn update_loss_function_via_idx(&mut self, idx: usize, new_loss_function: Box<dyn LossFunction>) -> Result<(), String> {
        if idx >= self._loss_functions.len() {
            return Err(format!("idx too high for loss function update in ObjectiveFunction ({:?} with _loss_functions length of {:?})", idx, self._loss_functions.len()))
        }

        self._loss_functions[idx] = new_loss_function;

        return Ok(());
    }

    pub fn get_num_isolated_objective_terms(&self) -> usize {
        return self._isolated_objective_terms.len();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            println!("{}{} Isolated objective term {:?} ---> {}", color::Fg(color::Blue), style::Bold, i, style::Reset);
            println!("{}       name: {:?} {}", color::Fg(color::LightWhite), self._isolated_objective_terms[i].name(), style::Reset);
            println!("{}       loss function: {:?} {}", color::Fg(color::LightWhite), self._loss_functions[i], style::Reset);
            println!("{}       weight: {:?} {}", color::Fg(color::LightWhite), self._weights[i], style::Reset);
        }
        println!("\n");
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _get_first_idx_from_name(&self, objective_name: &String) -> Option<usize> {
        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            if &self._isolated_objective_terms[i].name() == objective_name {
                return Some(i);
            }
        }
        return None;
    }

    fn _get_all_idxs_from_name(&self, objective_name: &String) -> Vec<usize> {
        let mut out_vec = Vec::new();

        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            if &self._isolated_objective_terms[i].name() == objective_name {
                out_vec.push(i);
            }
        }
        return out_vec;
    }
}

impl LynxVarsUser for ObjectiveFunction {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        let mut out_vec = Vec::new();
        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            let lynx_vars_types = self._isolated_objective_terms[i].get_lynx_vars_types();
            let l2 = lynx_vars_types.len();
            for j in 0..l2 {
                out_vec.push( lynx_vars_types[j].clone() );
            }
        }
        return out_vec;
    }
}
impl Clone for ObjectiveFunction {
    fn clone(&self) -> Self {
        let mut _isolated_objective_terms = Vec::new();
        let l = self._isolated_objective_terms.len();
        for i in 0..l {
            _isolated_objective_terms.push( self._isolated_objective_terms[i].clone() );
        }

        let mut _loss_functions = Vec::new();
        let l = self._loss_functions.len();
        for i in 0..l {
            _loss_functions.push( self._loss_functions[i].clone_box() );
        }

        let _weights = self._weights.clone();

        Self { _isolated_objective_terms, _loss_functions, _weights }
    }
}
unsafe impl Send for ObjectiveFunction { }
unsafe impl Sync for ObjectiveFunction { }

#[derive(Debug, Clone)]
pub struct ObjectiveFunctionCallOutput {
    _obj_val: f64,
    _isolated_objective_term_names: Option<Vec<String>>,
    _isolated_objective_term_values: Option<Vec<f64>>,
    _isolated_objective_term_values_before_weight: Option<Vec<f64>>,
    _isolated_objective_term_values_before_loss_function: Option<Vec<f64>>
}
impl ObjectiveFunctionCallOutput {
    pub fn new_emtpy(debug: bool) -> Self {

        match debug {
            true => {
                return Self { _obj_val: 0.0, _isolated_objective_term_names: Some(Vec::new()),
                    _isolated_objective_term_values: Some(Vec::new()),
                    _isolated_objective_term_values_before_weight: Some(Vec::new()),
                    _isolated_objective_term_values_before_loss_function: Some(Vec::new())};
            },
            false => {
                return Self { _obj_val: 0.0, _isolated_objective_term_names: None, _isolated_objective_term_values: None, _isolated_objective_term_values_before_weight: None,_isolated_objective_term_values_before_loss_function: None };
            }
        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_debug_entry(&mut self, isolated_objective_term_name: String, isolated_objective_term_value: f64, isolated_objective_term_value_before_weight: f64, isolated_objective_term_value_before_loss_function: f64) -> Result<(), String> {
        if self._isolated_objective_term_names.is_none() {
            return Err("tried to add debug entry in ObjectiveFunctionCallOutput, but debug was false".to_string());
        }

        self._isolated_objective_term_names.as_mut().unwrap().push(isolated_objective_term_name);
        self._isolated_objective_term_values.as_mut().unwrap().push(isolated_objective_term_value);
        self._isolated_objective_term_values_before_weight.as_mut().unwrap().push(isolated_objective_term_value_before_weight);
        self._isolated_objective_term_values_before_loss_function.as_mut().unwrap().push(isolated_objective_term_value_before_loss_function);

        return Ok(());
    }

    pub fn print_summary(&self) {
        let debug = self._isolated_objective_term_names.is_some();

        match debug {
            false => {
                println!("{}{}objective value: {} {}", color::Fg(color::LightCyan), style::Bold, self._obj_val, style::Reset);
                println!("{}   looks like there wasn't any debug information saved for this output.  If you want more information, make sure to set debug to true next call. {}", color::Fg(color::Yellow), style::Reset);
            },
            true => {
                println!("{}{}objective value: {} {}", color::Fg(color::LightCyan), style::Bold, self._obj_val, style::Reset);
                let l = self._isolated_objective_term_names.as_ref().unwrap().len();
                for i in 0..l {
                    println!("{}     {:?} isolated objective term {:?} ---> {}", color::Fg(color::Blue), i, self._isolated_objective_term_names.as_ref().unwrap()[i], style::Reset);
                    println!("{}          value: {:?} {}", color::Fg(color::White), self._isolated_objective_term_values.as_ref().unwrap()[i], style::Reset);
                    println!("{}          value before weight: {:?} {}", color::Fg(color::White), self._isolated_objective_term_values_before_weight.as_ref().unwrap()[i], style::Reset);
                    println!("{}          value before loss function: {:?} {}", color::Fg(color::White), self._isolated_objective_term_values_before_loss_function.as_ref().unwrap()[i], style::Reset);
                }
            }
        }
    }

    pub fn get_obj_val(&self) -> f64 {
        return self._obj_val;
    }
}

#[derive(Debug, Clone)]
pub struct ObjectiveFunctionGradientOutput {
    _gradient: DVector<f64>,
    _isolated_objective_term_names: Option<Vec<String>>,
    _isolated_objective_term_gradients: Option<Vec<DVector<f64>>>,
    _isolated_objective_term_gradients_before_weight: Option<Vec<DVector<f64>>>,
    _isolated_objective_term_gradients_before_loss_function: Option<Vec<DVector<f64>>>,
}
impl ObjectiveFunctionGradientOutput {
    pub fn new_emtpy(dim: usize, debug: bool) -> Self {

        match debug {
            true => {
                return Self { _gradient: DVector::from_element(dim, 0.0), _isolated_objective_term_names: Some(Vec::new()),
                    _isolated_objective_term_gradients: Some(Vec::new()),
                    _isolated_objective_term_gradients_before_weight: Some(Vec::new()),
                    _isolated_objective_term_gradients_before_loss_function: Some(Vec::new())};
            },
            false => {
                return Self { _gradient: DVector::from_element(dim, 0.0), _isolated_objective_term_names: None, _isolated_objective_term_gradients: None, _isolated_objective_term_gradients_before_weight: None,_isolated_objective_term_gradients_before_loss_function: None };
            }
        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_debug_entry(&mut self, isolated_objective_term_name: String, isolated_objective_term_gradient: &DVector<f64>, isolated_objective_term_gradient_before_weight: &DVector<f64>, isolated_objective_gradient_value_before_loss_function: &DVector<f64>) -> Result<(), String> {
        if self._isolated_objective_term_names.is_none() {
            return Err("tried to add debug entry in ObjectiveFunctionGradientOutput, but debug was false".to_string());
        }

        self._isolated_objective_term_names.as_mut().unwrap().push(isolated_objective_term_name);
        self._isolated_objective_term_gradients.as_mut().unwrap().push(isolated_objective_term_gradient.clone());
        self._isolated_objective_term_gradients_before_weight.as_mut().unwrap().push(isolated_objective_term_gradient_before_weight.clone());
        self._isolated_objective_term_gradients_before_loss_function.as_mut().unwrap().push(isolated_objective_gradient_value_before_loss_function.clone());

        return Ok(());
    }

    pub fn print_summary(&self) {
        let debug = self._isolated_objective_term_names.is_some();

        match debug {
            false => {
                println!("{}{}gradient: {:?} {}", color::Fg(color::LightMagenta), style::Bold, self._gradient, style::Reset);
                println!("{}   looks like there wasn't any debug information saved for this output.  If you want more information, make sure to set debug to true next call. {}", color::Fg(color::Yellow), style::Reset);
            },
            true => {
                println!("{}{}gradient: {:?} {}", color::Fg(color::LightMagenta), style::Bold, self._gradient, style::Reset);
                let l = self._isolated_objective_term_names.as_ref().unwrap().len();
                for i in 0..l {
                    println!("{}     {:?} isolated objective term {:?} ---> {}", color::Fg(color::Blue), i, self._isolated_objective_term_names.as_ref().unwrap()[i], style::Reset);
                    println!("{}          gradient: {:?} {}", color::Fg(color::White), self._isolated_objective_term_gradients.as_ref().unwrap()[i], style::Reset);
                    println!("{}          gradient before weight: {:?} {}", color::Fg(color::White), self._isolated_objective_term_gradients_before_weight.as_ref().unwrap()[i], style::Reset);
                    println!("{}          gradient before loss function: {:?} {}", color::Fg(color::White), self._isolated_objective_term_gradients_before_loss_function.as_ref().unwrap()[i], style::Reset);
                }
            }
        }
    }

    pub fn get_gradient(&self) -> &DVector<f64> {
        return &self._gradient;
    }
}

