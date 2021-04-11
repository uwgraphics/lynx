extern crate lynx_lib;
use lynx_lib::collision_detection::collision_tree_node::CollisionTreeNode;
use lynx_lib::collision_detection::collision_tree_layer::CollisionTreeLayer;
use lynx_lib::collision_detection::collision_tree::CollisionTree;

use std::time::Instant;

fn main () {
    let mut collision_tree_layer = CollisionTreeLayer::new();
    let idx0 = collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome0".to_string()));
    let idx1 = collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome1".to_string()));
    let idx2 = collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome2".to_string()));
    let idx3 = collision_tree_layer.add_from_mesh_filepath("dome.obj".to_string(), Some("dome3".to_string()));

    collision_tree_layer.set_curr_translation(idx1, 1., 0., 0.);

    collision_tree_layer.assign_sibling_groupings( "bestfit".to_string(), None );

    println!("{:?}", collision_tree_layer.sibling_groupings);

    let mut collision_tree_layer2 = CollisionTreeLayer::new();
    let idx0 = collision_tree_layer2.add_from_mesh_filepath("dome.obj".to_string(), Some("dome0".to_string()));
    let idx1 = collision_tree_layer2.add_from_mesh_filepath("dome.obj".to_string(), Some("dome1".to_string()));
    let idx2 = collision_tree_layer2.add_from_mesh_filepath("dome.obj".to_string(), Some("dome2".to_string()));
    let idx3 = collision_tree_layer2.add_from_mesh_filepath("dome.obj".to_string(), Some("dome3".to_string()));

    collision_tree_layer2.assign_sibling_groupings_manual( collision_tree_layer.sibling_groupings.clone(), None );

    println!("{:?}", collision_tree_layer2.sibling_groupings);

    println!("{}", collision_tree_layer2);

    println!("{:?}", collision_tree_layer.nodes[0].id);
    println!("{:?}", collision_tree_layer2.nodes[0].id);

    collision_tree_layer.add_blackout_id_to_node(0, collision_tree_layer2.nodes[0].id);
    println!("{:?}", collision_tree_layer.nodes[0].blackout_ids);

}