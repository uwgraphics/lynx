extern crate lynx_lib;
use lynx_lib::utils::utils_runtime_management::termination_util::TerminationUtilOption;
use rayon::prelude::*;

fn main() {
    let mut t1 = TerminationUtilOption::new();

    let a : Vec<usize> = (0..12).collect();

    a.par_iter().for_each(|x| {
        let mut local_t = t1.clone();

        if *x == 0 {
            local_t.set_to_terminate();
            println!("thread {:?} set to terminate!", x);
            return;
        }

        loop {
            if local_t.get_terminate() {
                println!("thread {:?} terminated.", x);
                return;
            }
        }

    });

}