extern crate lynx_lib;
use lynx_lib::robot_modules::robot_configuration_module::RobotConfigurationModule;
use lynx_lib::robot_modules::robot_fk_module::RobotFKModule;
use lynx_lib::robot_modules::robot_collision_detection_module::RobotCollisionDetectionModule;
use lynx_lib::robot_modules::robot_dof_module::RobotDOFModule;
use lynx_lib::utils::utils_files_and_strings::file_utils::*;
use lynx_lib::utils::utils_parsing::yaml_parsing_utils::*;
use std::time::Instant;
use lynx_lib::utils::utils_files_and_strings::robot_folder_utils::*;
use lynx_lib::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use lynx_lib::utils::utils_collisions::oriented_bounding_box::*;
use lynx_lib::utils::utils_math::implicit_dual_quaternion::ImplicitDualQuaternion;
use lynx_lib::robot_modules::robot_bounds_module::RobotBoundsModule;
use nalgebra::{Vector3};
use lynx_lib::utils::utils_math::nalgebra_utils::vec_to_dvec;
use lynx_lib::collision_detection::{collision_engine_pair_assessor::*, collision_engine_vars::*, collision_engine_vars_decoders::*, collision_engine::*};
use lynx_lib::utils::utils_math::choose_utils::n_choose_k;
use lynx_lib::utils::utils_collisions::oriented_bounding_box::{OBB, *};

fn main () {
    let mut r = RobotConfigurationModule::_new("hubo".to_string(), "test".to_string());
    let mut d = RobotDOFModule::new(&r);
    let mut fk = RobotFKModule::new(&r, &d);
    let mut b = RobotBoundsModule::new(&r, &d, None);

    let mut c = RobotCollisionDetectionModule::new(&r, &d, &fk, &b);

    let mut tree = c.robot_collision_tree.spawn();

    /*
    let start = Instant::now();
    for i in 0..1000 {
        let res = c.self_collision_check_vec(&fk, &vec![1., 0., 0., 0., 1., 0.]);
    }
    let stop = start.elapsed();
    println!("{:?}", stop);
    */

    c.print_num_collision_nodes_on_leaf_layer();

    let mut obb1 = OBB::new_from_path("dome.obj".to_string());
    let mut obb2 = OBB::new_from_path("dome.obj".to_string());

    obb1.set_curr_translation(0.1, 0.2, -0.6);
    obb2.set_curr_translation(0.4, 0.2, 0.6);

    obb1.update_max_and_mins(true);
    obb2.update_max_and_mins(true);

    let start = Instant::now();
    for i in 0..1000 {
        obb1.distance_check_bounding_sphere(&obb2);
    }
    println!("{:?}", start.elapsed());

    /*
    let pair_assessor = StandardPackInCollisionPairAssessor;
    let mut vars = CollisionEngineVars::new();
    InCollisionCEVarsDecoder::initialize(&mut vars, tree.tree_depth, tree.tree_depth);

    let start = Instant::now();
    for i in 0..1000 {
        pair_assessor.is_active_and_live_pair([5, 0], [5,7], &mut tree, &mut None, &mut vars);
    }
    let stop = start.elapsed();
    println!("{:?}", stop);
    */

    // println!("{:?}", 23%2);

    // c.print_num_collision_nodes_on_leaf_layer();
    // println!("{:?}", res.1.unwrap().num_collision_checks);

    // let r = get_number_of_pair_assessors_on_self_collision_check_from_number_of_nodes_on_leaf_layer(100);

    // println!("{:?}", r);
}