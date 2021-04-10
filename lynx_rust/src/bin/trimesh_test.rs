extern crate lynx_lib;
use lynx_lib::utils::utils_collisions::{triangle_mesh_engine::TriMeshEngine, collision_object::CollisionObject, oriented_bounding_box::*};
use nalgebra::{Vector3, Point3};
use std::time::Instant;

fn main() {
    let t = TriMeshEngine::new_from_path( "/home/rakita/lynx/robots/hubo/base_meshes/visual/Body_LHR.STL".to_string() );
    let mut c1 = CollisionObject::new_convex_hull(&t);
    let mut o1 = OBB::new_from_path( "/home/rakita/lynx/robots/hubo/base_meshes/visual/Body_LHR.STL".to_string() );

    // println!("{:?}", t.vertices.len());

    let t = TriMeshEngine::new_from_path( "/home/rakita/lynx/robots/hubo/base_meshes/visual/Body_RAFT.STL".to_string() );
    let mut c2 = CollisionObject::new_convex_hull(&t);
    let mut o2 = OBB::new_from_path( "/home/rakita/lynx/robots/hubo/base_meshes/visual/Body_RAFT.STL".to_string() );

    // println!("{:?}", t.vertices.len());

    c2.set_curr_translation(0.0, 0., 0.);
    o2.set_curr_translation(0.0,0.,0.);
    let mut r = false;

    let start = Instant::now();
    for i in 0..1000 {
        r = o1.intersect_check(&o2);
    }
    println!("{:?}", start.elapsed());
    println!("{:?}", r);

    let mut c3 = CollisionObject::new_capsule(1.0, 0.5, None);
    let mut c4 = CollisionObject::new_capsule(1.0, 0.2, None);

    c4.set_curr_translation(1.0, 0., 0.);

    let start = Instant::now();
    for i in 0..1000 {
        r = c3.intersect_check(&c4);
    }
    println!("{:?}", start.elapsed());
    println!("{:?}", r);

}