extern crate lynx_lib;
use lynx_lib::prelude::*;

fn main() -> Result<(), String> {
    let mut lynx_vars = LynxVarsGeneric::new_empty_parallel_packaged_with_robot_world(None, "hubo", Some("c2"), None, Some("hubo_arms_around_table"))?;

    let base_sampler = lynx_vars.get_robot_world_ref(None)?.get_robot_module_toolbox_ref().get_bounds_module_ref().to_lynx_float_vec_sampler_box();
    let collision_checker = RobotWorldCollisionChecker.to_collision_checker_box();

    let sampler = FreeSpaceSampler::new(base_sampler.clone(), collision_checker.clone());
    let samples = sampler.lynx_multi_float_vec_sampler_sample(&mut lynx_vars, 200, 1000, false)?;

    println!("{:?}", samples.len());

    output_dvec_path_to_file(&samples, "sprint", "hubo", "test");

    Ok(())
}