extern crate lynx_lib;
use lynx_lib::utils::utils_vars::lynx_vars::LynxVars;
use lynx_lib::{set_or_add_lynx_var, get_lynx_vars_tuple_ref_from_ident, get_lynx_vars_tuple_mut_ref_from_ident, set_lynx_var, add_lynx_var, get_lynx_var_idx};

fn main() -> Result<(), String> {

    /*
    let mut lynx_vars = LynxVars::new_empty();

    // lynx_vars.add_variable( "hi", 1, LynxVarsType::USIZE );

    let mut robot_toolbox = RobotModuleToolbox::new("ur5", None, None)?;
    let fk = robot_toolbox.get_fk_module_ref().compute_fk_gradient_perturbations_on_all_zeros_config()?;

    lynx_vars.add_f64_variable("v", 10.);
    lynx_vars.set_f64_variable("v", 20.);
    lynx_vars.add_usize_variable("v", 1);
    lynx_vars.add_usize_variable("v", 2);
    lynx_vars.add_robot_fk_gradient_perturbation_result_ref_variable("fk_test", &fk);

    let res = lynx_vars.get_usize_variable_ref("v")?;

    println!("{:?}", res);

    let d = DVector::from_element(2, 0.0);

    lynx_vars.add_dvector_ref_variable("test", &d);

    println!("{:?}", lynx_vars.get_dvector_ref_variable_ref("test"));


    let res = lynx_vars.contains_variable(&LynxVarsType::USIZE("test"));
    println!("{:?}", res);

    lynx_vars.print_summary();

    let fk_get = lynx_vars.get_robot_fk_gradient_perturbation_result_ref_variable_ref("fk_test")?;
    let p = fk_get.get_x_ref();

    println!("{:?}", p);
    */

    let mut lynx_vars = LynxVars::new_empty();
    set_or_add_lynx_var!(lynx_vars, f64, "hi", 1.0);

    println!("{:?}", lynx_vars);

    set_or_add_lynx_var!(lynx_vars, f64, "hi", 3.0);

    println!("{:?}", lynx_vars);

    Ok(())
}