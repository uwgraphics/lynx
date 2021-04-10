extern crate lynx_lib;
use lynx_lib::utils::utils_optimization::isolated_objective_term::*;
use lynx_lib::utils::utils_vars::lynx_vars::LynxVars;
use lynx_lib::utils::utils_vars::lynx_vars_user::LynxVarsUser;

use lynx_lib::{add_lynx_var, get_lynx_vars_tuple_mut_ref_from_ident, get_lynx_vars_tuple_ref_from_ident};

fn main() {
    let mut t = Test;

    let mut lynx_vars = LynxVars::new_empty();
    // lynx_vars.add_f64_variable("a", 1.0);
    // lynx_vars.add_f64_variable("b", 1.0);
    // lynx_vars.add_f64_variable("c", 1.0);

    add_lynx_var!(lynx_vars, f64, "a", 1.0);
    add_lynx_var!(lynx_vars, f64, "b", 1.0);
    add_lynx_var!(lynx_vars, f64, "c", 1.0);

    t.print_diagnostics_information(&mut lynx_vars, &None ,&Some(vec![2.123,4.]));

}