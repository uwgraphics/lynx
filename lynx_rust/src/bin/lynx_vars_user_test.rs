extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;
use lynx_lib::utils::utils_vars::lynx_vars_user::{LynxVarsUser, AsLynxVarsUser};
use lynx_lib::utils::utils_vars::lynx_vars::LynxVars;
use lynx_lib::utils::utils_optimization::objective_function::ObjectiveFunction;
use lynx_lib::utils::utils_optimization::isolated_objective_term::*;
use lynx_lib::{add_lynx_var, get_lynx_vars_tuple_ref_from_ident, get_lynx_vars_tuple_mut_ref_from_ident};

fn main() -> Result<(), String> {
    /*
    let mut r = AlwaysCollision;

    let mut vars = LynxVars::new_empty();

    r.check_valid(&vars, true);
    */
    let mut vars = LynxVars::new_empty();

    add_lynx_var!(vars, f64, "a" , 1.0);
    add_lynx_var!(vars, f64, "b" , 1.0);

    let mut t = Test;
    t.print_diagnostics_information(&mut vars, &mut None, &Some(vec![0.,0.]));

    let mut o = ObjectiveFunction::new(vec![Box::new(t)], None)?;

    o.check_valid(&vars, true);

    Ok(())
}