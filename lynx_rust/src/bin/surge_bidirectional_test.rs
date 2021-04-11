extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::prelude::*;
use lynx_lib::utils::utils_sampling::prelude::*;
use lynx_lib::utils::utils_math::prelude::*;
use lynx_lib::utils::utils_vars::prelude::*;
use lynx_lib::utils::utils_recorders::prelude::*;

fn main() -> Result<(), String> {
    let range_sampler = RangeFloatVecSampler::new(-2.0, 2.0, 2);
    let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
    let recorder = RecorderArcMutexOption::new_none();

    let mut surge = SurgeBidirectional::new_empty();
    surge.add_start_state(&vec_to_dvec(&vec![-1.,-1.]));
    surge.add_end_state(&vec_to_dvec(&vec![1.,1.]));
    for _ in 0..50 {
        surge.add_milestone_state(&range_sampler.float_vec_sampler_sample()?);
    }
    surge.add_surge_objective_term_box(SurgeObjectiveCloserToTargetDag::new_empty().to_surge_objective_term_box(), None);

    let lower_is_better = surge.get_lower_is_better()?;

    surge.initialize_all_surge_objective_terms_with_surge_points_manager(&mut lynx_vars, &recorder);

    surge.add_successful_connection_auto_linear_path(0, 7, 0.05)?;
    surge.add_successful_connection_auto_linear_path(7, 10, 0.05)?;
    surge.add_successful_connection_auto_linear_path(1, 17, 0.05)?;
    surge.add_successful_connection_auto_linear_path(17, 10, 0.05)?;
    surge.add_successful_connection_auto_linear_path(0, 8, 0.05)?;
    surge.add_successful_connection_auto_linear_path(17, 49, 0.05)?;
    surge.add_successful_connection_auto_linear_path(8, 49, 0.05)?;

    // surge.add_successful_connection_auto_linear_path(0,1, 0.01)?;
    // surge.add_successful_connection_auto_linear_path(1,0, 0.01)?;

    surge.get_surge_points_manager_ref().print_summary();

    println!("{:?}", surge.has_at_least_one_solution_been_found());

    // surge.get_planning_dag_ref().print_tree();
    // surge.get_planning_dag_and_surge_points_manager_idx_util_ref().print();

    let solution_paths = surge.get_all_solution_paths()?;
    println!("{:?}", solution_paths[0]);

    let start = Instant::now();
    let best = surge.get_n_best_candidate_connections(12, lower_is_better, false, &mut lynx_vars, &recorder)?;
    let stop = start.elapsed();
    println!("{:?}", stop);

    let start = Instant::now();
    let best = surge.get_n_best_candidate_connections(12, lower_is_better, false, &mut lynx_vars, &recorder)?;
    let stop = start.elapsed();
    println!("{:?}", stop);

    Ok(())
}