extern crate lynx_lib;

use lynx_lib::utils::utils_optimization::objective_function::*;
use lynx_lib::utils::utils_optimization::isolated_objective_term::*;
use lynx_lib::utils::utils_vars::lynx_vars::LynxVars;
use lynx_lib::utils::utils_vars::lynx_vars_user::LynxVarsUser;

fn main() -> Result<(), String> {
    let mut o = ObjectiveFunction::new(vec![Box::new(Test)], None)?;
    o.add_isolated_objective_term(Box::new(Test), None);
    let mut l = LynxVars::new_empty();

    o.print_summary();

    // o.print_diagnostics_for_all_isolated_objective_terms(&mut l, &None, &mut None);
    // o.print_all_lynx_vars_types();

    o.remove_all_isolated_objective_terms_with_given_name("test");

    o.print_summary();

    Ok(())
}