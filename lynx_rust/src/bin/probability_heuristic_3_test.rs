extern crate lynx_lib;
use lynx_lib::motion_planning::sprint::probability_heuristic_3::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;

fn main() {
    let q_p = vec_to_dvec(&vec![0.,0.]);
    let q_x = vec_to_dvec(&vec![0.1,0.0]);
    let q_m_star = vec_to_dvec(&vec![1.,0.]);

    let ob1 =  vec_to_dvec(&vec![0.2, 0.01]);
    let n_obs = vec![ &ob1 ];

    let r = probability_heuristic_3_manual_inputs(&q_x, &q_m_star, None, &n_obs, 0.1);

    println!("{:?}", r);
}