extern crate lynx_lib;
use lynx_lib::utils::utils_vars::lynx_vars::LynxVars;
// use lynx_lib::utils::utils_vars::lynx_vars_type::LynxVarsType;
use lynx_lib::{add_lynx_var, get_lynx_vars_tuple_mut_ref_from_ident, set_lynx_var, set_lynx_var_via_idx, get_lynx_vars_tuple_ref_from_ident, get_lynx_var_idx, get_lynx_var_ref, get_lynx_var_mut_ref, get_lynx_var_ref_via_idx, get_lynx_var_mut_ref_via_idx};
// use lynx_lib::utils::utils_vars::lynx_vars_macros;
use std::collections::HashMap;

fn main() {
    let mut lynx_vars = LynxVars::new_empty();

    /*
    let res = add_lynx_var!(lynx_vars, usize, "a", 5);
    println!("{:?}", res);

    let res = add_lynx_var!(lynx_vars, usize, "a", 5);
    println!("{:?}", res);

    let res = set_lynx_var!(lynx_vars, usize, "a", 6);
    println!("{:?}", res);

    let res = get_lynx_var_idx!(lynx_vars, usize, "e");
    println!("{:?}", res);

    let res = get_lynx_var_mut_ref_via_idx!(lynx_vars, usize, 0);
    println!("{:?}", res);

    let res = add_lynx_var!(lynx_vars, f64, "a", 30.);
    println!("{:?}", res);

    let idx = get_lynx_var_idx!(lynx_vars, f64, "a");
    // println!("{:?}", idx);

    let i = idx.unwrap().clone();
    let res = get_lynx_var_ref_via_idx!(lynx_vars, f64, i);
    println!("{:?}", res);

    println!("{:?}", lynx_vars.all_var_types_and_names);
    */
}