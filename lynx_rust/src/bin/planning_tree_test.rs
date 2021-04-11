extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::utils_planning_graphs::planning_tree::PlanningTree;
use lynx_lib::utils::utils_path_planning::utils_paths::linear_spline_path::LinearSplinePath;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;

fn main() -> Result<(), String> {
    let mut pt = PlanningTree::new_empty();

    let res = pt.add_another_root_node_vec( &vec![0.,0.] ); // 0
    let res = pt.add_another_root_node_vec( &vec![0.1,0.1] ); // 1
    let res = pt.add_another_root_node_vec( &vec![0.2,0.2] ); // 2
    let res = pt.add_another_root_node_vec( &vec![0.3,0.3] ); // 3
    
    let res = pt.add_node_vec_with_auto_two_waypoint_inflow_edge(&vec![0.2,0.3], 2)?;
    let res = pt.add_node_vec_with_auto_two_waypoint_inflow_edge(&vec![0.2,0.4], 4)?;
    let res = pt.add_node_vec_with_auto_two_waypoint_inflow_edge(&vec![0.2,0.5], 5)?;
    let res = pt.add_node_vec_with_auto_two_waypoint_inflow_edge(&vec![0.2,0.6], 6)?;

    pt.add_auto_linear_edge(0, 3, 0.01);

    // let res = pt.get_path_that_connects_two_roots(0, 3,  3)?;

    pt.print_tree();

    let r = pt.get_previous_n_states_that_lead_into_node(3, 2)?;
    println!("{:?}", r);

    // pt.print_tree();
    /*
    pt.add_node_vec_with_auto_two_waypoint_inflow_edge(&vec![0.1, 0.1], 0); // 2
    pt.add_node_vec_with_auto_two_waypoint_inflow_edge(&vec![0.9, 0.9], 1); // 3

    // pt.add_node_vec_with_two_waypoint_inflow_edge(&vec![0.1, 0.4], 3);

    let mut edge = LinearSplinePath::new_from_vecs(vec![vec![0.9, 0.9], vec![0.1, 0.6], vec![0.1, 0.1]]);

    let res = pt.add_edge(&edge,3, 2);

    println!("{:?}", res);

    pt.print_tree();

    // println!("{:?}", res);

    // let res = pt.get_path_from_tree_root_to_node(3);
    // println!("{:?}", res);
    // let res = pt.get_path_from_particular_tree_root_to_node(0, 1);

    let res = pt.get_path_that_connects_two_roots(0, 1, 2);

    println!("{:?}", res);
    */

    Ok(())
}