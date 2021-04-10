use ncollide3d::math::Point;
use nalgebra::{Vector3, DMatrix};
use nalgebra::linalg;
use crate::utils::utils_math::geometry_utils::*;
use crate::utils::utils_math::nalgebra_utils::*;
use crate::utils::utils_sampling::prelude::*;

pub fn ncollide_points_to_vector3_points(points: &Vec<Point<f64>>) -> Vec<Vector3<f64>> {
    let mut out_vec = Vec::new();

    let l = points.len();
    for i in 0..l {
        out_vec.push(  Vector3::new( points[i][0], points[i][1], points[i][2] )  );
    }
    out_vec
}

pub fn find_pointcloud_centroid_ncollide(points: &Vec<Point<f64>>) -> Point<f64> {
    let mut count = 0.0;
    let mut sum = Point::new(0.,0.,0.);

    let l = points.len();
    for i in 0..l {
        sum[0] += points[i][0];
        sum[1] += points[i][1];
        sum[2] += points[i][2];
        count += 1.0;
    }

    return sum / count;
}

pub fn find_pointcloud_centroid(points: &Vec<Vector3<f64>>) -> Vector3<f64> {
    let mut count = 0.0;
    let mut sum = Vector3::new(0.,0.,0.);

    let l = points.len();
    for i in 0..l {
        sum += &points[i];
        count += 1.0;
    }

    return sum / count;
}

/*
pub fn take_subset_of_points(points: &Vec<Vector3<f64>>, max_num_points: usize) -> Vec<Vector3<f64>> {
    if points.len() <= max_num_points { return points.clone() }

    let mut out_vec = Vec::new();

    let mut rand_idxs = Vec::new();
    let l = points.len();
    let sampler = ThreadRangeIntSampler::new(0, l as i32, 1);

    while !(rand_idxs.len() >= max_num_points) {
        let s = sampler.sample()[0];
        if !(rand_idxs.contains(&s)) {
            rand_idxs.push(s);
        }
    }

    let l = rand_idxs.len();
    for i in 0..l {
        out_vec.push( points[rand_idxs[i] as usize].clone() );
    }

    out_vec
}
*/


