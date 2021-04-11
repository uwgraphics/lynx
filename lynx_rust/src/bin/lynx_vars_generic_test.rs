extern crate lynx_lib;
use lynx_lib::utils::utils_vars::prelude::*;
use std::time::Instant;

fn main() {
    let mut lynx_vars_generic = LynxVarsGeneric::new_empty_single_threaded();

    let mut now = Instant::now();
    lynx_vars_generic.convert_to_parallel(None, false);
    println!("{:?}", now.elapsed());

    println!("{:?}", lynx_vars_generic.get_num_threads());
}