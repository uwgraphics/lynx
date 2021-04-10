extern crate lynx_lib;
use lynx_lib::robot_modules::robot_world::RobotWorld;
use lynx_lib::optimal_motion_planning::prelude::vec_to_dvec;
use lynx_lib::robot_modules::robot_core_collision_module::LinkGeometryType;

fn main() -> Result<(), String> {
    let mut robot_world = RobotWorld::new("sawyer", None, None, Some("sawyer_vertical_bars"))?;

    let fk_res = robot_world.get_robot_module_toolbox_ref().get_fk_module_ref().compute_fk(&vec_to_dvec(&vec![0.,0.,0.,0.,0.,0.,0.,0.]))?;

    let res = robot_world.environment_contact_check(&fk_res, LinkGeometryType::OBBs, false, None)?;

    res.print_summary();

    Ok(())
}