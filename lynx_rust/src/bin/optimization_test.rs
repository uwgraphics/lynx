extern crate lynx_lib;
use lynx_lib::{add_lynx_var, get_lynx_vars_tuple_mut_ref_from_ident, get_lynx_vars_tuple_ref_from_ident};
use lynx_lib::utils::utils_optimization::{objective_function::*, isolated_objective_term::*, nonlinear_optimization_engine::*, loss_function::*};
use lynx_lib::utils::utils_vars::lynx_vars::LynxVars;
use lynx_lib::utils::utils_vars::lynx_vars_user::LynxVarsUser;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use std::sync::{Mutex};

fn main() -> Result<(), String> {
    let mut lynx_vars = LynxVars::new_empty();

    add_lynx_var!(lynx_vars, f64, "a", 5.0);
    add_lynx_var!(lynx_vars, f64, "b", 1.0);
    add_lynx_var!(lynx_vars, f64_vec_option, "upper_bounds_option", None);
    add_lynx_var!(lynx_vars, f64_vec_option, "lower_bounds_option", None);

    let objective_function = ObjectiveFunction::new( vec![Box::new(Test)], None)?;

    // objective_function.check_valid(&lynx_vars, true);
    objective_function.print_diagnostics_for_all_isolated_objective_terms(&mut lynx_vars, &Some(vec![1.,1.]), &mut None);

    // let mut o = NLoptNonlinearOptimizationEngine::new_mma();
    let mut o = OpenNonlinearOptimizationEngine;
    // o.check_valid(&lynx_vars, true);

    let res = o.optimize(&vec_to_dvec(&vec![5.01,1.]), &objective_function, &mut lynx_vars, &mut None, Some(100), Some(1.0))?;

    res.print_summary();

    Ok(())
}