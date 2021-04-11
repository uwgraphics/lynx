extern crate lynx_lib;
use lynx_lib::robot_modules::robot_configuration_module::RobotConfigurationModule;
use lynx_lib::robot_modules::robot_fk_module::RobotFKModule;
use lynx_lib::robot_modules::robot_collision_detection_module::RobotCollisionDetectionModule;
use lynx_lib::robot_modules::robot_core_collision_module_::RobotCoreCollisionModule;
use lynx_lib::robot_modules::robot_dof_module::RobotDOFModule;
use lynx_lib::utils::utils_files_and_strings::file_utils::*;
use lynx_lib::utils::utils_parsing::yaml_parsing_utils::*;
use std::time::Instant;
use lynx_lib::robot_modules::robot_bounds_module::RobotBoundsModule;
use lynx_lib::utils::utils_preprocessing::preprocessing_utils::*;
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::utils::utils_collisions::collision_environment::CollisionEnvironment;

fn main () {
    let mut r = RobotConfigurationModule::_new("sawyer".to_string(), "test".to_string());
    let dof = RobotDOFModule::new(&r);
    let fk = RobotFKModule::new(&r, &dof);
    let b = RobotBoundsModule::new(&r, &dof, None);
    let mut c = RobotCoreCollisionModule::new(&r, &dof, &fk, &b);

    dof.print_joint_dof_order();

    r.robot_model_module.print_link_order();

    let res = fk.compute_fk_vec(&vec![-0.09,0.,0.,0.,0.,0.,0.]);
    // println!("{:?}", res);

    fk.print_results_next_to_link_names(&res, &r);

    let collision_environment = CollisionEnvironment::new("sawyer_vertical".to_string());

    let start = Instant::now();
    for i in 0..1000 {
        let res = c.environment_collision_check(&res, &collision_environment);
    }
    println!("{:?}", start.elapsed());

    let res = c.environment_collision_check(&res, &collision_environment);

    println!("{:?}", res);
}