use nalgebra::{Vector3, Point3};
use std::time::Instant;
extern crate lynx_lib;
use lynx_lib::utils::utils_math::implicit_dual_quaternion::ImplicitDualQuaternion;
use lynx_lib::utils::utils_collisions::collision_object::CollisionObject;
use lynx_lib::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use lynx_lib::utils::utils_sampling::sampler::{ThreadRangeSampler, ThreadSampler};


pub struct Test {
    pub v: Vector3<f64>,
    pub r: f64
}

fn main() {
    let sampler = ThreadRangeSampler::new(0., 1.0, 3);

    let mut idq = ImplicitDualQuaternion::new_from_euler_angles(0.1, 0.0, 0.0, Vector3::new(1., 0., 0.));

    let t1 = Test {v: Vector3::new(1., 0., 0.), r: 1.0};
    let mut t2 = Test {v: Vector3::new(1., 0.3, 1.), r: 1.0};

    let v1 = Vector3::new(1., 0., 0.);
    let mut v2 = Vector3::new(1., 0.3, 1.);
    let mut r = 1.0;

    let mut ts = Vec::new();
    for i in 0..10000 {
        let s = sampler.sample();
        ts.push( Test { v: Vector3::new(s[0], s[1], s[2]), r: 0.0 } );
    }

    let start = Instant::now();
    // for i in 0..10000 {
        // t2.r = (&t1.v - &t2.v).norm();
        // v2 = idq.multiply_by_point_shortcircuit(&t2.v);
        // t2.r = (&t2.v - &ts[i].v).norm();

    // }
    ts.iter().for_each( |x| t2.r = (&t2.v - x.v).norm() );
    println!("{:?}", start.elapsed());
    println!("{:?}", t2.r);

}