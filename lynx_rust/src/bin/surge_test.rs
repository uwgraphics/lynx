extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::utils_surge::prelude::*;
use lynx_lib::utils::utils_paths::linear_spline_path::LinearSplinePath;
use lynx_lib::utils::utils_vars::prelude::*;
use lynx_lib::utils::utils_recorders::prelude::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;

fn main() -> Result<(), String> {
    let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
    let mut recorder = RecorderArcMutexOption::new_none();

    let mut surge = Surge::new_empty();
    surge.add_surge_objective_term(&SurgeObjectivePairwiseDis, None);
    surge.add_surge_objective_term(&SurgeObjectiveDisToEndStateSet::new_empty(), None);
    surge.add_start_state(&vec_to_dvec(&vec![0.,0.]));
    surge.add_end_state(&vec_to_dvec(&vec![1.,1.]));
    surge.add_milestone_state(&vec_to_dvec(&vec![0.1,0.0]));

    surge.update_all_surge_objective_terms_after_connection_attempt(0, 1, false, &mut lynx_vars, &recorder);

    let res = surge.get_n_best_candidate_connections(3, &mut lynx_vars, &recorder)?;
    println!("{:?}", res);

    let a = surge.set_dead_connection(0,1);

    surge.get_surge_connection_manager_ref().print();

    surge.update_all_surge_objective_terms_after_connection_attempt(0, 1, false, &mut lynx_vars, &recorder);

    let res = surge.get_n_best_candidate_connections(3, &mut lynx_vars, &recorder)?;
    println!("{:?}", res);

    surge.update_all_surge_objective_terms_after_connection_attempt(0, 1, false, &mut lynx_vars, &recorder);

    let res = surge.get_n_best_candidate_connections(3, &mut lynx_vars, &recorder)?;
    println!("{:?}", res);

    Ok(())
}