extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::utils_surge::surge_points_manager::SurgePointsManager;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;


fn main() {

    let mut s = SurgePointsManager::new_empty();
    s.add_start_state(&vec_to_dvec(&vec![0.,0.]));
    s.add_end_state(&vec_to_dvec(&vec![1.,1.]));
    s.add_milestone_state(&vec_to_dvec(&vec![0.5, 0.5]));
    s.add_milestone_state(&vec_to_dvec(&vec![0.6, 0.5]));
    s.add_milestone_state(&vec_to_dvec(&vec![0.7, 0.5]));

    s.convert_milestone_to_start_dag_node(2);
    s.convert_milestone_to_end_dag_node(4);
    s.convert_milestone_to_end_dag_node(3);

    s.add_milestone_state(&vec_to_dvec(&vec![0.8, 0.5]));

    let res = s.convert_auto(1, 1);
    println!("{:?}", res);

    s.print_summary();

    let r = s.get_point_ref(2);
    println!("{:?}", r);

    println!("{:?}", s.are_all_start_and_end_states_reached());
}