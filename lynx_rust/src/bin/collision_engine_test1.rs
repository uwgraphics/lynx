extern crate lynx_lib;
use lynx_lib::collision_detection::collision_tree::CollisionTree;
use lynx_lib::collision_detection::collision_tree_layer::CollisionTreeLayer;
use lynx_lib::collision_detection::collision_engine_visitor::*;
use lynx_lib::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use lynx_lib::collision_detection::collision_engine_vars::*;
use lynx_lib::collision_detection::collision_engine_vars_decoders::{DistanceCEVarsDecoder, InCollisionCEVarsDecoder};
use lynx_lib::collision_detection::collision_engine::*;

fn main() {
    let mut collision_tree_layer = CollisionTreeLayer::new();
    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome1".to_string()));
    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome2".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome2".to_string(), 1., 3.5, 0.0);
    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome3".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome3".to_string(), 0.0, 2., 10.0);
    let mut collision_tree1 = CollisionTree::new(collision_tree_layer, None);

    // collision_tree1.set_curr_translation_on_leaf_layer(&"dome2".to_string(), 1., 3.5, 0.0);
    // collision_tree1.set_curr_translation_on_leaf_layer(&"dome3".to_string(), 0.0, 2., 10.0);
    //collision_tree1._construct_full_tree(None);
    // collision_tree1.update_tree();

    let mut collision_tree_layer = CollisionTreeLayer::new();
    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome1".to_string()));
    collision_tree_layer.nodes[0].obb.set_curr_translation(-4., 3.5, 0.0);
    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome2".to_string()));
    collision_tree_layer.nodes[1].obb.set_curr_translation(-4., 0.0, 2.0);
    let mut collision_tree2 = CollisionTree::new(collision_tree_layer, None);
    collision_tree2.update_tree();

    // collision_tree2.set_curr_translation_on_leaf_layer(&"dome1".to_string(), -4., 3.5, 0.0);
    // collision_tree2.set_curr_translation_on_leaf_layer(&"dome2".to_string(), -4., 0.0, 2.0);
    // collision_tree2.update_tree();

    println!("{}", collision_tree1);
    println!("{}", collision_tree2);

    let res = CollisionEngine::distance_check_with_margin_and_closest_n_per_layer(&mut collision_tree1, &mut Some(&mut collision_tree2), 2.0, 1);
    println!("{:?}", res);


}