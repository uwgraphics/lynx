extern crate lynx_lib;
use lynx_lib::motion_planning::sprint::common_functions::*;
use nalgebra::DVector;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;

fn main() {
    let q = vec_to_dvec(&vec![-1000.,0.5]);
    let q_a = vec_to_dvec(&vec![-1.,-1.]);
    let q_b = vec_to_dvec(&vec![1.,1.]);


    let r = proj_plus(&q, &q_a, &q_b);
    println!("{:?}", r);


}