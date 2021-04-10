use crate::utils::utils_collisions::collision_object::*;
use crate::utils::utils_pointclouds::pointcloud_principal_axes_utils::*;
use crate::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use crate::utils::utils_se3::transformation_utils::*;
use nalgebra::{Vector3, Matrix3, UnitQuaternion, Quaternion, Point3};
use ncollide3d::query::{Proximity, PointQuery, Contact, Ray, RayCast, PointProjection};
use ncollide3d::query;
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use crate::utils::utils_files_and_strings::string_utils::u64_to_string;

pub struct OBB {
    pub collision_object: CollisionObject,
    pub name: String,
    pub id: u64,
    pub center: Vector3<f64>,
    pub base_corners: Vec<Vector3<f64>>,
    pub curr_corners: Vec<Vector3<f64>>,
    pub half_extents: [f64; 3],
    pub dim_maximums: [f64; 3], // x, y, and z
    pub dim_minimums: [f64; 3], // x, y, and z,
    pub corners_outdated: bool,
    pub max_and_min_outdated: bool,
    pub volume: f64,
    pub initial_pose: ImplicitDualQuaternion
}

impl OBB {
    pub fn new_empty() -> Self {
        let mut collision_object = CollisionObject::new_cuboid(0.01, 0.01, 0.01, None, None);
        Self { collision_object, name: "0".to_string(), id: 0, center: Vector3::new(0.,0.,0.), base_corners: Vec::new(), curr_corners: Vec::new(), half_extents: [0., 0., 0.], dim_maximums: [0., 0., 0.], dim_minimums: [0., 0., 0.], corners_outdated: false, max_and_min_outdated: false, volume: 0.0, initial_pose: ImplicitDualQuaternion::new_identity() }
    }

    pub fn new_from_trimesh_engine(t: &TriMeshEngine, name: Option<String>) -> Self {
        let center = t.compute_center();

        let mut dim_maximums =  [-std::f64::INFINITY;3 ];
        let mut dim_minimums = [ std::f64::INFINITY;3 ];

        let l = t.vertices.len();
        for i in 0..l {
            for j in 0..3 {
                if t.vertices[i][j] < dim_minimums[j] { dim_minimums[j] = t.vertices[i][j]; }
                if t.vertices[i][j] > dim_maximums[j] { dim_maximums[j] = t.vertices[i][j]; }
            }
        }

        let mut center = Vector3::zeros();
        let mut half_extents = [ 0.0; 3 ];
        for j in 0..3 {
            center[j] = (dim_maximums[j] + dim_minimums[j]) / 2.0;
            half_extents[j] = (dim_maximums[j] - center[j]).abs();
        }

        let x = Vector3::new(1., 0., 0.);
        let y = Vector3::new(0., 1., 0.);
        let z = Vector3::new(0., 0., 1.);
        let mut base_corners = Vec::new();
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let corner = &center + ((-1.0 as f64).powi(i) * half_extents[0] * &x) + ((-1.0 as f64).powi(j) * half_extents[1] * &y) + ((-1.0 as f64).powi(k) * half_extents[2] * &z);
                    base_corners.push(  corner  );
                }
            }
        }

        let curr_corners = base_corners.clone();

        let mut volume = 2.0*half_extents[0] * 2.0*half_extents[1] * 2.0*half_extents[2];

        let initial_pose = ImplicitDualQuaternion::new_from_euler_angles(0., 0., 0., center.clone());

        let collision_object = CollisionObject::new_cuboid(half_extents[0], half_extents[1], half_extents[2], Some(initial_pose.clone()), name.clone());

        let mut id = Self::_get_id();
        let mut name_ = u64_to_string(id);
        if name.is_some() { name_ = name.unwrap(); }

        return Self { collision_object, name: name_, id, center, base_corners, curr_corners, half_extents, dim_maximums, dim_minimums, corners_outdated: false, max_and_min_outdated: false, volume, initial_pose }
    }

    pub fn new_from_path(fp: String, name: Option<String>) -> Result<Self, String> {
        let t = TriMeshEngine::new_from_path(fp)?;
        Ok(Self::new_from_trimesh_engine(&t, name))
    }

    pub fn spawn(&self) -> Self {
        let id = self.id;
        let name = self.name.clone();
        let center = self.center.clone();
        let base_corners = self.base_corners.clone();
        let curr_corners = self.curr_corners.clone();
        let half_extents = self.half_extents.clone();
        let dim_maximums = self.dim_maximums.clone();
        let dim_minimums = self.dim_minimums.clone();
        let corners_outdated = self.corners_outdated.clone();
        let max_and_min_outdated = self.max_and_min_outdated.clone();
        let volume = self.volume.clone();
        let mut collision_object = CollisionObject::new_cuboid(half_extents[0], half_extents[1], half_extents[2], Some(self.initial_pose.clone()), Some(self.name.clone()));
        collision_object.curr_pose = self.collision_object.curr_pose.clone();
        collision_object.base_pose = self.collision_object.base_pose.clone();
        collision_object.curr_translation_global_delta = self.collision_object.curr_translation_global_delta.clone();
        collision_object.curr_quaternion_global_delta = self.collision_object.curr_quaternion_global_delta.clone();
        collision_object.bounding_aabb = self.collision_object.bounding_aabb.clone();
        collision_object.bounding_sphere = self.collision_object.bounding_sphere.clone();
        collision_object.bounding_aabb_outdated = self.collision_object.bounding_aabb_outdated.clone();
        collision_object.bounding_sphere_outdated = self.collision_object.bounding_sphere_outdated.clone();
        collision_object.base_rot_mat = self.collision_object.base_rot_mat.clone();
        collision_object.manual_bounding_sphere = self.collision_object.manual_bounding_sphere.clone();
        collision_object.curr_isometry = self.collision_object.curr_isometry.clone();
        collision_object.has_base_pose = self.collision_object.has_base_pose;
        Self {collision_object, name, id, center, base_corners, curr_corners,half_extents,dim_maximums, dim_minimums, corners_outdated, max_and_min_outdated, volume, initial_pose: self.initial_pose.clone() }
    }

    pub fn stretch_or_shrink_dimensions(&mut self, x_ratio: f64, y_ratio: f64, z_ratio: f64) {
        self.half_extents = [ self.half_extents[0] * x_ratio,  self.half_extents[1] * y_ratio, self.half_extents[2] * z_ratio];

        let x = Vector3::new(1., 0., 0.);
        let y = Vector3::new(0., 1., 0.);
        let z = Vector3::new(0., 0., 1.);
        let mut base_corners = Vec::new();
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let corner = &self.center + ((-1.0 as f64).powi(i) * self.half_extents[0] * &x) + ((-1.0 as f64).powi(j) * self.half_extents[1] * &y) + ((-1.0 as f64).powi(k) * self.half_extents[2] * &z);
                    base_corners.push(  corner  );
                }
            }
        }

        self.base_corners = base_corners.clone();
        self.curr_corners = base_corners.clone();

        let collision_object = CollisionObject::new_cuboid(self.half_extents[0], self.half_extents[1], self.half_extents[2], Some(self.initial_pose.clone()), Some(self.name.clone()));

        self.collision_object = collision_object;

        let mut volume = 2.0*self.half_extents[0] * 2.0*self.half_extents[1] * 2.0*self.half_extents[2];
        self.volume = volume;

        self.max_and_min_outdated = true;
        self.corners_outdated = true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn combine(&mut self, other: &mut OBB) -> Self {
        // return Self::new_empty();

        if self.corners_outdated || self.max_and_min_outdated { self.update_max_and_mins(true);  }
        if other.corners_outdated || other.max_and_min_outdated { other.update_max_and_mins(true); }

        return self.combine_immutable(other);
    }

    pub fn combine_immutable(&self, other: &OBB) -> Self {
        let mut dim_maximums = [0.,0.,0.];
        let mut dim_minimums = [0.,0.,0.];

        for i in 0..3 {
            dim_maximums[i] = self.dim_maximums[i].max(other.dim_maximums[i]);
        }
        for i in 0..3 {
            dim_minimums[i] = self.dim_minimums[i].min(other.dim_minimums[i]);
        }

        let mut center = Vector3::new( (dim_minimums[0] + dim_maximums[0])/2.0, (dim_minimums[1] + dim_maximums[1])/2.0, (dim_minimums[2] + dim_maximums[2])/2.0 );

        let mut half_extents = [0., 0., 0.];
        half_extents[0] = (dim_maximums[0] - dim_minimums[0]) / 2.0;
        half_extents[1] = (dim_maximums[1] - dim_minimums[1]) / 2.0;
        half_extents[2] = (dim_maximums[2] - dim_minimums[2]) / 2.0;

        let mut base_pose = ImplicitDualQuaternion::new_from_euler_angles(0., 0., 0., center.clone());

        let mut id = Self::_get_id();
        let mut name = u64_to_string(id);

        let collision_object = CollisionObject::new_cuboid(half_extents[0], half_extents[1], half_extents[2], Some(base_pose.clone()), Some(u64_to_string(id)));

        let mut base_corners = Vec::new();
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let mut v = &center + Vector3::new( (-1.0 as f64).powi(i) * half_extents[0], (-1.0 as f64).powi(j) * half_extents[1], (-1.0 as f64).powi(k) * half_extents[2] );
                    base_corners.push(  v  );
                }
            }
        }

        let mut curr_corners = base_corners.clone();

        let mut volume = 2.0*half_extents[0] * 2.0*half_extents[1] * 2.0*half_extents[2];

        return Self { collision_object, name, id, center, base_corners, curr_corners, half_extents, dim_maximums, dim_minimums, corners_outdated: false, max_and_min_outdated: false, volume, initial_pose: base_pose };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn update_corner_positions(&mut self) {
        for i in 0..8 {
            self.curr_corners[i] = self.collision_object.curr_quaternion_global_delta * &self.base_corners[i] + &self.collision_object.curr_translation_global_delta;
        }
        self.corners_outdated = false;
    }

    pub fn update_max_and_mins(&mut self, update_corner_positions_first: bool) {
        if update_corner_positions_first { self.update_corner_positions(); }

        self.dim_maximums = [ -std::f64::INFINITY, -std::f64::INFINITY, -std::f64::INFINITY ];
        self.dim_minimums = [ std::f64::INFINITY, std::f64::INFINITY, std::f64::INFINITY ];

        for i in 0..8 {
            if self.dim_maximums[0] < self.curr_corners[i][0] { self.dim_maximums[0] = self.curr_corners[i][0]; }
            if self.dim_maximums[1] < self.curr_corners[i][1] { self.dim_maximums[1] = self.curr_corners[i][1]; }
            if self.dim_maximums[2] < self.curr_corners[i][2] { self.dim_maximums[2] = self.curr_corners[i][2]; }

            if self.dim_minimums[0] > self.curr_corners[i][0] { self.dim_minimums[0] = self.curr_corners[i][0]; }
            if self.dim_minimums[1] > self.curr_corners[i][1] { self.dim_minimums[1] = self.curr_corners[i][1]; }
            if self.dim_minimums[2] > self.curr_corners[i][2] { self.dim_minimums[2] = self.curr_corners[i][2]; }
        }
        self.max_and_min_outdated = false;
    }

    pub fn update_bounding_sphere(&mut self) {
        self.collision_object.update_bounding_sphere();
    }

    pub fn update_bounding_aabb(&mut self) {
        self.collision_object.update_bounding_aabb();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_curr_translation(&mut self, x: f64, y: f64, z: f64) {
        self.collision_object.set_curr_translation(x, y, z);
        self.corners_outdated = true;
        self.max_and_min_outdated = true;
    }

    pub fn set_curr_orientation(&mut self, w: f64, x: f64, y: f64, z: f64) {
        self.collision_object.set_curr_orientation(w, x, y, z);
        self.corners_outdated = true;
        self.max_and_min_outdated = true;
    }

    pub fn set_curr_orientation_from_euler_angles(&mut self, rx: f64, ry: f64, rz: f64) {
        self.collision_object.set_curr_orientation_from_euler_angles(rx, ry, rz);
        self.corners_outdated = true;
        self.max_and_min_outdated = true;
    }

    pub fn set_curr_pose(&mut self, implicit_dual_quaternion: &ImplicitDualQuaternion) {
        self.collision_object.set_curr_pose(implicit_dual_quaternion);
        self.corners_outdated = true;
        self.max_and_min_outdated = true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn intersect_check(&self, other: &OBB) -> bool {
        return self.collision_object.intersect_check(&other.collision_object);
    }

    pub fn intersect_check_bounding_aabb(&mut self, other: &mut OBB) -> bool {
        return self.collision_object.intersect_check_bounding_aabb(&mut other.collision_object);
    }

    pub fn intersect_check_bounding_aabb_immutable(&self, other: &OBB) -> bool {
        return self.collision_object.intersect_check_bounding_aabb_immutable(&other.collision_object);
    }

    pub fn intersect_check_bounding_sphere(&mut self, other: &mut OBB) -> bool {
        return self.collision_object.intersect_check_bounding_sphere(&mut other.collision_object);
    }

    pub fn intersect_check_bounding_sphere_immutable(&self, other: &OBB) -> bool {
        return self.collision_object.intersect_check_bounding_sphere_immutable(&other.collision_object);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn distance_check(&self, other: &OBB) -> f64 {
        return self.collision_object.distance_check(&other.collision_object);
    }

    pub fn distance_check_bounding_sphere(&self, other: &OBB) -> f64 {
        return self.collision_object.distance_check_bounding_sphere(&other.collision_object);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn contact_check(&self, other: &OBB, margin: Option<f64>) -> Option<Contact<f64>> {
        return self.collision_object.contact_check(&other.collision_object, margin);
    }

    pub fn contact_check_bounding_sphere(&self, other: &OBB, margin: Option<f64>) -> Option<Contact<f64>> {
        return self.collision_object.contact_check_bounding_sphere(&other.collision_object, margin);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn intersects_ray(&self, ray_origin: &Point3<f64>, ray_direction: &Vector3<f64>) -> bool {
        return self.collision_object.intersects_ray(ray_origin, ray_direction);
    }

    pub fn intersects_ray_bounding_sphere(&self, ray_origin: &Point3<f64>, ray_direction: &Vector3<f64>) -> bool {
        return self.collision_object.intersects_ray_bounding_sphere(ray_origin, ray_direction);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn contains_point(&self, point: &Point3<f64>) -> bool {
        return self.collision_object.contains_point(point);
    }

    pub fn contains_point_bounding_sphere(&self, point: &Point3<f64>) -> bool {
        return self.collision_object.contains_point_bounding_sphere(point);
    }

    pub fn project_point(&self, point: &Point3<f64>, solid: bool) -> PointProjection<f64> {
        return self.collision_object.project_point(point, solid);
    }

    pub fn project_point_bounding_sphere(&self, point: &Point3<f64>, solid: bool) -> PointProjection<f64> {
        return self.collision_object.project_point_bounding_sphere(point, solid);
    }

    pub fn distance_to_point(&self, point: &Point3<f64>, solid: bool) -> f64 {
        return self.collision_object.distance_to_point(point, solid);
    }

    pub fn distance_to_point_bounding_sphere(&self, point: &Point3<f64>, solid: bool) -> f64 {
        return self.collision_object.distance_to_point_bounding_sphere(point, solid);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _get_id() -> u64 {
        let mut rng = rand::thread_rng();
        let id = rng.gen_range(0, u64::max_value());
        return id
    }
}






