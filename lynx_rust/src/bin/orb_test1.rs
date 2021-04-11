extern crate lynx_lib;
use lynx_lib::utils::utils_collisions::{triangle_mesh_engine::TriMeshEngine, collision_object::CollisionObject};
use lynx_lib::utils::utils_collisions::orb::*;
use nalgebra::{Vector3, Point3};



fn main() {
    let t = TriMeshEngine::new_from_path( "/home/rakita/lynx/robots/hubo/base_meshes/visual/Body_WST.STL".to_string() );
    let mut c = CollisionObject::new_convex_hull(&t);

    let o = Orb::new(&Vector3::new( 0.,0.,0.156 ), 1.0, &c);
    println!("{:?}", o.curr_position);
    println!("{:?}", o.uncertainty_radius);
    println!("{:?}", o.certainty_radius);
    println!("{:?}", o.interior);

    println!("{:?}", OrbProximityResult::Unsure);
    // println!("{:?}", c.project_point( &Point3::new(0., 0.0, 0.13) , false).is_inside);
}