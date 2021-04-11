
extern crate lynx_lib;
use lynx_lib::utils::utils_sampling::sampler::{ThreadRangeSampler, ThreadSampler};
use lynx_lib::utils::utils_math::vp_search_utils::*;
use nalgebra::{DVector, Vector3};
use std::time::Instant;

pub struct Test {
    pub v: Vector3<f64>
}

fn main() {

    let sampler = ThreadRangeSampler::new(0., 1.0, 3);
    let mut points = Vec::new();

    for i in 0..10 {
        let s = sampler.sample();
        points.push( Vector3::new( s[0], s[1], s[2] ) );
    }

    let start = Instant::now();
    let vps = VPSearch3D::new( &points );
    println!("{:?}", start.elapsed());

    let t1 = Test {v: Vector3::new(1., 0., 0.)};
    let mut r = (0, 1.0);

    let start = Instant::now();
    for i in 0..1000 {
        // let s = sampler.sample();
        // r = vps.get_closest( &Vector3::new(s[0], s[1], s[2]) );
        r = vps.get_closest(&t1.v);
    }
    println!("{:?}", start.elapsed());
    println!("{:?}", r);

}