extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::utils_surge::prelude::*;
use lynx_lib::utils::utils_vars::prelude::*;
use lynx_lib::utils::utils_recorders::prelude::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;

fn main() -> Result<(), String> {
    let mut lynx_vars = LynxVarsGeneric::new_empty_single_threaded();
    let mut recorder = RecorderArcMutexOption::new_none();
    let mut surge_point_manager = SurgePointsManager::new_empty();
    let mut surge_connection_manager = SurgeConnectionManager::new_empty();

    surge_point_manager.add_start_state(&vec_to_dvec(&vec![0.,0.]));
    surge_point_manager.add_end_state(&vec_to_dvec(&vec![1.,1.]));

    let mut o = SurgeObjectiveManager::new_empty();
    o.add_surge_objective_term( &SurgeObjectiveLocalMinAvoid::new_empty(), None );

    let surge_connection_info = surge_connection_manager.get_connection_mut_ref(0,1, &mut o, &surge_point_manager, &mut lynx_vars, &recorder)?;
    let res = o.call_surge_objective(0, surge_connection_info, &surge_point_manager, &mut lynx_vars, &recorder);
    println!("{:?}", res);

    o.update_all_after_connection_attempt(0,1, false, &surge_point_manager, &mut lynx_vars, &recorder);

    let surge_connection_info = surge_connection_manager.get_connection_mut_ref(0,1, &mut o, &surge_point_manager, &mut lynx_vars, &recorder)?;
    let res = o.call_surge_objective(0, surge_connection_info, &surge_point_manager, &mut lynx_vars, &recorder);

    println!("{:?}", res);

    let surge_connection_info = surge_connection_manager.get_connection_mut_ref(0,1, &mut o, &surge_point_manager, &mut lynx_vars, &recorder)?;
    let res = o.call_surge_objective(0, surge_connection_info, &surge_point_manager, &mut lynx_vars, &recorder);

    println!("{:?}", res);


    Ok(())
}