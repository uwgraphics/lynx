extern crate lynx_lib;
use lynx_lib::path_planning::sprint::sprint_global_points_manager::SprintGlobalPointsManager;
use lynx_lib::path_planning::sprint::sprint_global_connection_manager::SprintGlobalConnectionManager;
use lynx_lib::path_planning::sprint::probability_heuristic_1::*;
use lynx_lib::path_planning::sprint::sprint_local_search::SprintLocalSearch;
use lynx_lib::utils::utils_path_planning::path_planning_collision_checker::*;
use lynx_lib::utils::utils_path_planning::path_planner_output::*;
use lynx_lib::utils::utils_path_planning::utils_path_planning_sampling::{freespace_sampling::*, cspace_sampler::*};
use lynx_lib::utils::utils_path_planning::utils_planning_graphs::planning_tree::PlanningTree;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use std::time::Instant;
use nalgebra::DVector;


fn main() {
    let mut sprint_global_points_manager = SprintGlobalPointsManager::new(&vec_to_dvec(&vec![0.5,0.01]), &vec_to_dvec(&vec![0.5,0.9]));
    let mut sprint_global_connections_manager = SprintGlobalConnectionManager::new(&sprint_global_points_manager);

    let idx = sprint_global_points_manager.add_milestone_point(&vec_to_dvec(&vec![0.808048763039948, 0.8368498614599098])); // 2
    let idx = sprint_global_points_manager.add_milestone_point(&vec_to_dvec(&vec![0.13296593316690375, 0.83938825166552])); // 3
    let idx = sprint_global_points_manager.add_milestone_point(&vec_to_dvec(&vec![0.8016618669243893, 0.7984329183516454])); // 4

    sprint_global_points_manager.add_localmin_region((0, 2));
    sprint_global_points_manager.add_localmin_region((0, 3));


    println!("{:?}", sprint_global_connections_manager.get_probability_of_useful_connection((0, 4), &sprint_global_points_manager, true));

    /*
    println!("{:?}", idx);

    sprint_global_points_manager.add_localmin_region((0,1));
    sprint_global_points_manager.add_localmin_region((0,3));

    let p = sprint_global_connections_manager.get_probability_of_useful_connection((0,5), &sprint_global_points_manager, true);
    println!("{:?}", p);

    let res = probability_heuristic_1(&sprint_global_points_manager, &mut sprint_global_connections_manager, false);
    println!("{:?}", res);
    */
}