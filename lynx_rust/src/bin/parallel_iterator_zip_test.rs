extern crate lynx_lib;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;
use rayon::prelude::*;


fn main() -> Result<(), String> {
    let mut robot_module_toolbox = RobotModuleToolbox::new("ur5", None, None)?;
    let mut robot_module_toolbox2 = RobotModuleToolbox::new("ur5", None, None)?;
    let mut robot_module_toolbox3 = RobotModuleToolbox::new("ur5", None, None)?;

    let a = robot_module_toolbox.get_core_collision_parallel_module_mut_ref().unwrap().get_all();
    let b = robot_module_toolbox2.get_core_collision_parallel_module_mut_ref().unwrap().get_all();
    let c = robot_module_toolbox3.get_core_collision_parallel_module_mut_ref().unwrap().get_all();

    // let mut iter_zip = (a,b,c).into_par_iter();


    let mut iter_zip = a.par_iter_mut().zip( b.par_iter_mut() ).zip( c.par_iter_mut() );

    iter_zip.for_each(|x| {
        // (x.1).test();
    });


    let mut iter_zip = a.par_iter_mut().zip( b.par_iter_mut() ).zip( c.par_iter_mut() );

    iter_zip.for_each(|x| {
        // (x.1).test();
    });


    Ok(())
}
