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
    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome0".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome0".to_string(), 0., 0., 0.);

    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome1".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome1".to_string(), 0., 3.0, 0.);

    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome2".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome2".to_string(), 3., 0.0, 0.);

    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome3".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome3".to_string(), 3., 3.0, 0.);

    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome4".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome4".to_string(), 0., 0.0, 2.5);

    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome5".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome5".to_string(), 0.0, 3.0, 2.5);

    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome6".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome6".to_string(), 3.0, 0.0, 2.5);

    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome7".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome7".to_string(), 3.0, 3.0, 2.5);

    collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome8".to_string()));
    collision_tree_layer.set_curr_translation_from_name(&"dome8".to_string(), 1.5, 1.5, 1.5);

    let mut collision_tree1 = CollisionTree::new(collision_tree_layer, None);
    collision_tree1.update_tree();

    println!("{}", collision_tree1);



    let res = CollisionEngine::distance_check_with_margin_and_closest_n_per_layer(&mut collision_tree1, &mut None, 0.0001, 1);
    println!("{:?}", res);

}