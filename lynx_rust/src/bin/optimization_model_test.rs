extern crate lynx_lib;
use lynx_lib::utils::utils_optimization::{optimization_model::OptimizationModel, optimization_model_prefab_old::*};
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_optimization::nonlinear_optimization_engine::OpenNonlinearOptimizationEngine;


fn main() -> Result<(), String> {
    /*
    let mut o = OptimizationModel::new_test()?;

    o.print_diagnostics(&Some(vec![0.,0.]));

    let res = o.optimize_open_default( &vec_to_dvec(&vec![1.,1.]) )?;
    res.print_summary();
    */

    let mut m = TestPrefab.get_optimization_model()?;
    m.print_diagnostics_with_default_nonlinear_optimization_engine_check(&Some(vec![0.2,0.2]));

    Ok(())
}