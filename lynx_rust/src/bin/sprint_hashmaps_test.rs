extern crate lynx_lib;
use lynx_lib::motion_planning::sprint::sprint_local_data_manager::SprintLocalDataManager;
use lynx_lib::motion_planning::sprint::probability_heuristic_2::probability_heuristic_2;
use lynx_lib::motion_planning::sprint::common_functions::proj_scalar;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use std::time::Instant;

fn main() {
    let mut s = SprintLocalDataManager::new(&vec_to_dvec(&vec![0., 0.]), &vec_to_dvec(&vec![5., 5.]) );
    s.label_q_x_as_checkpoint(0, &vec_to_dvec(&vec![0., 0.]));

    let r = s.add_freespace_q_c_star(0, &vec_to_dvec(&vec![0.1,0.1])); // 1
    let r = s.add_freespace_q_c_star(1, &vec_to_dvec(&vec![0.2,0.2])); // 2
    let r = s.add_freespace_q_c_star(2, &vec_to_dvec(&vec![0.3,0.3])); // 3
    let r = s.add_freespace_q_c_star(2, &vec_to_dvec(&vec![0.4,0.4])); // 4
    let r = s.add_freespace_q_c_star(2, &vec_to_dvec(&vec![0.5,0.5])); // 5
    let r = s.add_freespace_q_c_star(2, &vec_to_dvec(&vec![0.6,0.6])); // 6

    s.add_collision_q_c_star(6, &vec_to_dvec(&vec![0.7,0.7]));
    s.label_q_x_as_checkpoint(5, &vec_to_dvec(&vec![0.5, 0.5]));

    let r = s.add_freespace_q_c_star(5, &vec_to_dvec(&vec![0.7,0.5])); // 7
    let r = s.add_freespace_q_c_star(7, &vec_to_dvec(&vec![0.8,0.5])); // 8
    let r = s.add_freespace_q_c_star(8, &vec_to_dvec(&vec![0.9,0.5])); // 9

    s.add_collision_q_c_star(9, &vec_to_dvec(&vec![1.0,0.5]));
    s.label_q_x_as_checkpoint(7, &vec_to_dvec(&vec![0.8, 0.5]));

    s.print_summary();

    let r = s.get_nearby_collision_points(7, None);
    println!("{:?}", r);

    /*
    s.label_q_x_as_checkpoint(0, &vec_to_dvec(&vec![0., 0.]));

    let r = s.add_freespace_q_c_star(0, &vec_to_dvec(&vec![0.1,0.1])); // 1
    let r = s.add_freespace_q_c_star(1, &vec_to_dvec(&vec![0.2,0.2])); // 2
    let r = s.add_freespace_q_c_star(2, &vec_to_dvec(&vec![0.3,0.3])); // 3

    let r = s.add_collision_q_c_star(3, &vec_to_dvec(&vec![0.4,0.4]));
    let r = s.label_q_x_as_checkpoint(2, &vec_to_dvec(&vec![0.2,0.2]));

    let r = s.add_freespace_q_c_star(2, &vec_to_dvec(&vec![0.2,0.4])); // 4
    let r = s.add_freespace_q_c_star(4, &vec_to_dvec(&vec![0.3,0.45])); // 5

    let r = s.add_collision_q_c_star(5, &vec_to_dvec(&vec![0.35,0.5]));

    let r = s.add_freespace_q_c_star(5, &vec_to_dvec(&vec![-0.8,-0.85])); // 5

    // println!("{:?}", r);

    s.print_summary();

    let r = s.get_nearby_collision_points(4, None);
    println!("{:?}", r);

    // let p = probability_heuristic_2(4, &s, None);
    // println!("{:?}", p);
    */
}