extern crate lynx_lib;
use lynx_lib::utils::utils_vars::lynx_vars_type::LynxVarsType;
use lynx_lib::utils::utils_vars::lynx_vars_user::LynxVarsUser;
use lynx_lib::utils::utils_vars::lynx_vars_old::LynxVars;
use lynx_lib::utils::utils_vars::lynx_vars_validity_checker::LynxVarsValidityChecker;
use lynx_lib::utils::utils_optimization::isolated_objective_term::IsolatedObjectiveTerm;
use nalgebra::DVector;

pub struct T;

impl LynxVarsUser for T {
    fn get_lynx_vars_types(&self) -> Vec<LynxVarsType> {
        return vec![ LynxVarsType::USIZE("1"), LynxVarsType::F64("woohoo") ];
    }
}



fn main() {

    let t = T;
    t.print_all_lynx_vars_types();

    let mut vars = LynxVars::new_empty();

    vars.add_usize_variable("1", 1);
    vars.add_f64_variable("woohoo", 1.0);

    LynxVarsValidityChecker::check_valid(&vars, &t, true);



}