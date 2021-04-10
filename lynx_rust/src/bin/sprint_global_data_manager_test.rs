extern crate lynx_lib;
use lynx_lib::motion_planning::sprint::sprint_global_points_manager::*;
use lynx_lib::motion_planning::sprint::sprint_global_connection_manager::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use nalgebra::{DVector};

fn main() {
    let q_init = vec_to_dvec(&vec![0.,0.]);
    let q_goal = vec_to_dvec(&vec![1.,1.]);

    let r = vec_to_dvec(&vec![1.0, 0.2]);

    let mut p = SprintGlobalPointsManager::new(&q_init, &q_goal );
    let mut c = SprintGlobalConnectionManager::new(&p);

    for i in 0..10 {
        p.add_milestone_point(&r);
    }

    c.print_summary();
    p.print_summary();

    p.add_localmin_region((0,1));
    c.get_probability_of_good_connection((0,2), &p);

    c.print_summary();
    p.print_summary();

    // p.add_localmin_region((0,5));
    // c.get_global_search_region_info_add_if_needed((0,2), &p);

    c.print_summary();
    p.print_summary();

    let a = c.get_probability_of_good_connection((0,4), &p);
    println!("{:?}", a);
}