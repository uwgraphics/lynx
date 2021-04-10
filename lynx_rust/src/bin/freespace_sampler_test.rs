extern crate lynx_lib;
use lynx_lib::utils::utils_sampling::prelude::*;
use lynx_lib::utils::utils_collisions::collision_checker::*;
use lynx_lib::utils::utils_vars::prelude::*;
use lynx_lib::utils::utils_recorders::prelude::*;
use lynx_lib::robot_modules::robot_world::RobotWorld;
use lynx_lib::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_files_and_strings::fileIO_directory_utils::*;

fn main() -> Result<(), String> {
    let mut lynx_vars = LynxVarsGeneric::new_empty_parallel(None);

    let mut robot_world = RobotWorld::new("ur5", None, None, Some("ur5_single_box"))?;
    let mut base_sampler = robot_world.get_robot_module_toolbox_ref().get_bounds_module_ref().to_lynx_float_vec_sampler_box();
    let mut collision_checker = CollisionCheckerBox::new(&RobotWorldCollisionChecker);
    set_or_add_lynx_var_generic!(&mut lynx_vars, RobotWorld, "robot_world", robot_world);

    // println!("{:?}", res);

    let mut sampler = FreeSpaceSampler::new(base_sampler, collision_checker.clone());

    let s = sampler.lynx_multi_float_vec_sampler_sample(&mut lynx_vars, 100, 500, false)?;

    output_dvec_path_to_file(&s, "test", "ur5", "test");

    println!("{:?}", s.len());

    Ok(())
}