extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::utils_surge::prelude::*;


fn main() {
    let mut p = PlanningDAGAndSurgePointsManagerIdxUtil::new_empty();

    p.add_idx_pairing(5,0);

    p.print();

    let res = p.get_planning_dag_idx_from_surge_points_manager_idx(0);
    println!("{:?}", res);
}