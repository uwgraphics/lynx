use crate::utils::utils_vars::lynx_vars_user::LynxVarsUser;
use crate::utils::utils_vars::{lynx_vars::LynxVars, lynx_vars_generic::LynxVarsGeneric};
use termion::{style, color};
use crate::utils::utils_optimization::isolated_objective_term::IsolatedObjectiveTerm;

pub struct LynxVarsValidityChecker;

impl LynxVarsValidityChecker {
    pub fn check_valid(lynx_vars: &LynxVars, lynx_vars_user: &dyn LynxVarsUser, print_output: bool) -> bool {
        let mut valid = true;

        let lynx_vars_types = lynx_vars_user.get_lynx_vars_types();
        let l = lynx_vars_types.len();
        for i in 0..l {
            let present = lynx_vars.contains_variable(lynx_vars_types[i].0, lynx_vars_types[i].1);
            if present {
                if print_output { println!("{}    {:?} was found.  {}", color::Fg(color::Green), lynx_vars_types[i], style::Reset); }
            } else {
                if print_output { println!("{}    {:?} was NOT found!  {}", color::Fg(color::Red), lynx_vars_types[i], style::Reset); }
                valid = false;
            }
        }

        if print_output {
            if valid { println!("{}{}  > The lynx vars user is valid, all variables are present.  {}", color::Fg(color::Green), style::Bold, style::Reset); }
            else { println!("{}{}  > The lynx vars user is invalid, not all variables are present.  {}", color::Fg(color::Red), style::Bold, style::Reset); }
        }

        return valid;
    }

    pub fn check_valid_generic(lynx_vars_generic: &LynxVarsGeneric, lynx_vars_user: &dyn LynxVarsUser, print_output: bool) -> bool {
        match lynx_vars_generic {
            LynxVarsGeneric::SingleThreaded(l) => return Self::check_valid(l, lynx_vars_user, print_output),
            LynxVarsGeneric::SingleThreadedMutRef(l) => return Self::check_valid(l, lynx_vars_user, print_output),
            LynxVarsGeneric::Parallel(l) => {
                let lynx_vars = l.get_first_ref();
                return Self::check_valid(lynx_vars, lynx_vars_user, print_output);
            }
        }
    }
}