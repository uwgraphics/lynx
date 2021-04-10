// PRE-DEPRECATION

/*

use ncollide3d::math::Point;
use nalgebra::{Vector3, DMatrix};
use nalgebra::linalg;
use crate::utils::utils_pointclouds::pointcloud_data_utils::*;
use crate::utils::utils_math::geometry_utils::*;
use crate::utils::utils_math::nalgebra_utils::*;

// NOTE: This is broken right now.  Probably don't use.

pub struct PointcloudPrincipalAxesUtils {
    pub axis_max: Vector3<f64>, // primary principal axis of point cloud
    pub axis_mid: Vector3<f64>, // primary principal axis of point cloud
    pub axis_min: Vector3<f64>,
    pub extent_points_max: [ Vector3<f64>; 2 ],
    pub extent_points_mid: [ Vector3<f64>; 2 ],
    pub extent_points_min: [ Vector3<f64>; 2 ],
    pub extent_points_max_projected_onto_axis: [ Vector3<f64>; 2 ],
    pub extent_points_mid_projected_onto_axis: [ Vector3<f64>; 2 ],
    pub extent_points_min_projected_onto_axis: [ Vector3<f64>; 2 ]
}

impl PointcloudPrincipalAxesUtils {
    pub fn new(points: &Vec<Vector3<f64>>, center: &Vector3<f64>, max_num_points: Option<usize>) -> Self {
        let mut points_local = points.clone();
        if max_num_points.is_some() { points_local = take_subset_of_points(points, max_num_points.unwrap()); }

        let l = points_local.len();
        let mut mat = DMatrix::from_element(3, l, 0.0);
        for i in 0..l {
            mat[(0,i)] = points_local[i][0];
            mat[(1,i)] = points_local[i][1];
            mat[(2,i)] = points_local[i][2];
        }

        let svd = linalg::SVD::new(mat, true, true);

        let sv = svd.singular_values;

        let mut idxmax = 0 as usize;
        let mut idxmid = 0 as usize;
        let mut idxmin = 0 as usize;

        if sv[0] > sv[1] && sv[1] > sv[2] {
            idxmax = 0; idxmid = 1; idxmin = 2;
        }
        if sv[0] > sv[2] && sv[2] > sv[1] {
            idxmax = 0; idxmid = 2; idxmin = 1;
        }
        if sv[1] > sv[0] && sv[0] > sv[2] {
            idxmax = 1; idxmid = 0; idxmin = 2;
        }
        if sv[1] > sv[2] && sv[2] > sv[0] {
            idxmax = 1; idxmid = 2; idxmin = 0;
        }
        if sv[2] > sv[0] && sv[0] > sv[1] {
            idxmax = 2; idxmid = 0; idxmin = 1;
        }
        if sv[2] > sv[1] && sv[1] > sv[0] {
            idxmax = 2; idxmid = 1; idxmin = 0;
        }

        let umat = svd.u.unwrap().clone();
        let vmat = svd.v_t.unwrap();

        let mut axis1 = Vector3::new(   umat[(0,idxmax)], umat[(1,idxmax)], umat[(2,idxmax)] );
        let mut axis2 = Vector3::new(   umat[(0,idxmid)], umat[(1,idxmid)], umat[(2,idxmid)] );
        let mut axis3 = Vector3::new(   umat[(0,idxmin)], umat[(1,idxmin)], umat[(2,idxmin)] );

        let mut axis1_max = -std::f64::INFINITY;
        let mut axis1_max_idx = 0;
        let mut axis1_min = std::f64::INFINITY;
        let mut axis1_min_idx = 0;

        let mut axis2_max = -std::f64::INFINITY;
        let mut axis2_max_idx = 0;
        let mut axis2_min = std::f64::INFINITY;
        let mut axis2_min_idx = 0;

        let mut axis3_max = -std::f64::INFINITY;
        let mut axis3_max_idx = 0;
        let mut axis3_min = std::f64::INFINITY;
        let mut axis3_min_idx = 0;

        for i in 0..points_local.len() {
            let axis1_val = vmat[(idxmax,i)];
            let axis2_val = vmat[(idxmid,i)];
            let axis3_val = vmat[(idxmin,i)];

            if axis1_val > axis1_max {
                axis1_max = axis1_val;
                axis1_max_idx = i;
            }
            if axis1_val < axis1_min {
                axis1_min = axis1_val;
                axis1_min_idx = i;
            }

            if axis2_val > axis2_max {
                axis2_max = axis2_val;
                axis2_max_idx = i;
            }
            if axis2_val < axis2_min {
                axis2_min = axis2_val;
                axis2_min_idx = i;
            }

            if axis3_val > axis3_max {
                axis3_max = axis3_val;
                axis3_max_idx = i;
            }
            if axis3_val < axis3_min {
                axis3_min = axis3_val;
                axis3_min_idx = i;
            }
        }

        println!("{:?}", points_local[axis1_max_idx]);
        println!("{:?}", points_local[axis1_min_idx]);

        let mut res_max1 = pt_dis_to_line(  &points_local[axis1_max_idx].data.to_vec(), &center.data.to_vec(), &(center + 100000.0 * &axis1).data.to_vec());
        let mut res_min1 = pt_dis_to_line(  &points_local[axis1_min_idx].data.to_vec(), &center.data.to_vec(), &(center + 100000.0 * &axis1).data.to_vec());
        let mut res_max2 = pt_dis_to_line(  &points_local[axis2_max_idx].data.to_vec(), &center.data.to_vec(), &(center + 100000.0 * &axis2).data.to_vec());
        let mut res_min2 = pt_dis_to_line(  &points_local[axis2_min_idx].data.to_vec(), &center.data.to_vec(), &(center + 100000.0 * &axis2).data.to_vec());
        let mut res_max3 = pt_dis_to_line(  &points_local[axis3_max_idx].data.to_vec(), &center.data.to_vec(), &(center + 100000.0 * &axis3).data.to_vec());
        let mut res_min3 = pt_dis_to_line(  &points_local[axis3_min_idx].data.to_vec(), &center.data.to_vec(), &(center + 100000.0 * &axis3).data.to_vec());

        Self { axis_max: axis1, axis_mid: axis2, axis_min: axis3,
        extent_points_max: [points[axis1_max_idx].clone(), points[axis1_min_idx].clone()],
        extent_points_mid: [points[axis2_max_idx].clone(), points[axis2_min_idx].clone()],
        extent_points_min: [points[axis3_max_idx].clone(), points[axis3_min_idx].clone()],
        extent_points_max_projected_onto_axis: [ dvec_to_vector3(&res_max1.1), dvec_to_vector3(&res_min1.1) ],
        extent_points_mid_projected_onto_axis: [ dvec_to_vector3(&res_max2.1), dvec_to_vector3(&res_min2.1) ],
        extent_points_min_projected_onto_axis: [ dvec_to_vector3(&res_max3.1), dvec_to_vector3(&res_min3.1) ] }
    }

    pub fn new_ncollide(points: &Vec<Point<f64>>, center: &Vector3<f64>, max_num_points: Option<usize>) -> Self {
        Self::new( &ncollide_points_to_vector3_points(&points), center, max_num_points)
    }
}

 */