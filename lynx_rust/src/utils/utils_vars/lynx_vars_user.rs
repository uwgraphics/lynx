use crate::utils::utils_vars::lynx_vars_validity_checker::LynxVarsValidityChecker;
use crate::utils::utils_vars::lynx_vars::LynxVars;
use crate::utils::utils_vars::prelude::LynxVarsGeneric;

pub trait LynxVarsUser : AsLynxVarsUser {
    fn get_lynx_vars_types(&self) -> Vec<(&'static str, &'static str)> {
        return Vec::new();
    }
    fn print_all_lynx_vars_types(&self) {
        let lynx_vars_types = self.get_lynx_vars_types();
        let l = lynx_vars_types.len();
        for i in 0..l {
            println!("{:?} ---> ({}, {})", i, lynx_vars_types[i].0, lynx_vars_types[i].1);
        }
    }
    fn check_valid(&self, lynx_vars: &LynxVars, print_output: bool) -> bool {
        LynxVarsValidityChecker::check_valid(lynx_vars, self.as_lynx_vars_user(), print_output)
    }
    fn check_valid_generic(&self, lynx_vars_generic: &LynxVarsGeneric, print_output: bool) -> bool {
        LynxVarsValidityChecker::check_valid_generic(lynx_vars_generic, self.as_lynx_vars_user(), print_output)
    }
}

pub trait AsLynxVarsUser {
    fn as_lynx_vars_user(&self) -> &LynxVarsUser;
}
impl<T: LynxVarsUser> AsLynxVarsUser for T {
    fn as_lynx_vars_user(&self) -> &LynxVarsUser {
        self
    }
}