extern crate lynx_lib;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_path_planning::utils_paths::path_segment_window::PathSegmentWindow;
use lynx_lib::utils::utils_path_planning::utils_planning_graphs::planning_tree::PlanningTree;


fn main() {
    let mut t = PlanningTree::new_unidirectional( &vec_to_dvec(&vec![0.,0.]) );

    t.add_node_with_auto_two_waypoint_inflow_edge( &vec_to_dvec(&vec![0.1,0.1]), 0 );
    t.add_node_with_auto_two_waypoint_inflow_edge( &vec_to_dvec(&vec![0.2,0.2]), 1 );

    let mut p = PathSegmentWindow::new(None);

    p.add_predecessor_point(t.get_node_ref(1));

    p.print_summary();
}