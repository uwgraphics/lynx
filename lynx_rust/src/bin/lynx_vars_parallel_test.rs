extern crate lynx_lib;
use lynx_lib::utils::utils_vars::lynx_vars_parallel::LynxVarsParallel;
use lynx_lib::utils::utils_vars::lynx_vars_old::*;


fn main() -> Result<(), String> {
    let mut l = LynxVarsParallel::new_empty(None)?;

    l.get_iter_mut().for_each(|x| x.add_f64_variable("hi", 1.0));
    l.get_iter_mut().for_each(|x| {x.set_f64_variable("hi", 5.0);});

    let r = l.get_one().get_f64_variable_ref("hi");

    println!("{:?}", r);

    time_lynx_vars_initialization();

    Ok(())
}