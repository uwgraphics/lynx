use ncollide3d::query;
use ncollide3d::query::{Proximity, PointQuery, Contact, Ray, RayCast, PointProjection};
use ncollide3d::shape::FeatureId;
use ncollide3d::shape::{Ball, Cuboid, Cylinder, Capsule, Cone, ConvexHull, Shape, TriMesh, CompositeShape, Compound, ShapeHandle};
use ncollide3d::bounding_volume::{self, BoundingVolume, BoundingSphere, AABB};
use ncollide3d::transformation;
use std::boxed::Box;
use nalgebra::{UnitQuaternion, Vector3, Translation3, Quaternion, Isometry3, DVector, Rotation3, Matrix3, Unit, Point3};
use crate::utils::utils_se3::transformation_utils::*;
use crate::utils::utils_se3::implicit_dual_quaternion::*;
use crate::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use crate::utils::utils_collisions::oriented_bounding_box::OBB;
use crate::utils::utils_files_and_strings::string_utils::u64_to_string;
use termion::{color, style};
use std::sync::Arc;
use rand::distributions::{Distribution, Uniform};
use rand::{Rng, SeedableRng};
use rand::rngs::{ThreadRng, StdRng};
use std::fmt;

pub struct CollisionObject {
    pub name: String,
    pub id: u64,
    pub active: bool,
    pub shape: Box<Arc<dyn Shape<f64>>>,
    pub bounding_sphere: BoundingSphere<f64>,
    pub base_bounding_sphere: BoundingSphere<f64>,
    pub manual_bounding_sphere: Ball<f64>, // manual bounding sphere mostly used for distance checking, rather than just intersection checking
    pub bounding_aabb: AABB<f64>,
    pub base_bounding_aabb: AABB<f64>,
    pub curr_pose: ImplicitDualQuaternion, // the current pose of the underlying collision object (which, by default, has to be the center).  Note that this may not actually be where you want to translate and rotate about.  This is set using the origin_anchor_point_in_base_pose_frame variable below.
    pub base_pose: ImplicitDualQuaternion, // the base pose of the ACTUAL object, i.e., this is the translation and rotation away from the object's origin in local space in order to get the collision object center to where it needs to be.  Then, when we actually rotate and translate the collision object, we can always respect this origin "anchor" point.
    pub curr_quaternion_global_delta: UnitQuaternion<f64>, // the quaternion such that curr_quaternion_delta * base_pose.quat = curr_pose.quat
    pub curr_translation_global_delta: Vector3<f64>,
    pub base_rot_mat: Matrix3<f64>, // currently not being used, but could be useful for coming up with local rotations
    pub has_base_pose: bool,
    pub curr_isometry: Isometry3<f64>,
    pub bounding_sphere_outdated: bool,
    pub bounding_aabb_outdated: bool
}

impl CollisionObject {
    pub fn new(shape: Box<Arc<dyn Shape<f64>>>, base_pose: Option<ImplicitDualQuaternion>, name: Option<String>) -> Self {
        let mut has_base_pose = base_pose.is_some();
        let mut base_pose_local = ImplicitDualQuaternion::new_identity();
        let mut base_rot_mat = Matrix3::identity();
        if has_base_pose {
            base_pose_local = base_pose.unwrap();
            base_rot_mat = *base_pose_local.quat.to_rotation_matrix().matrix();
        }
        let mut curr_pose = base_pose_local.clone();

        let translation = Translation3::new( curr_pose.translation[0], curr_pose.translation[1], curr_pose.translation[2] );
        let curr_isometry = Isometry3::from_parts(translation, curr_pose.quat.clone());

        let base_bounding_sphere = bounding_volume::bounding_sphere(&(**shape), &Isometry3::identity());
        let base_bounding_aabb = bounding_volume::aabb(&(**shape), &Isometry3::identity());
        let bounding_sphere = bounding_volume::bounding_sphere(&(**shape), &curr_isometry);
        let bounding_aabb = bounding_volume::aabb(&(**shape), &curr_isometry);

        let mut manual_bounding_sphere = Ball::new(base_bounding_sphere.radius());

        let mut curr_quaternion_global_delta = UnitQuaternion::identity();
        let mut curr_translation_global_delta = Vector3::zeros();

        let id = Self::_get_id();
        let mut name_ = u64_to_string(id);
        if name.is_some() { name_ = name.unwrap(); }

        Self {name: name_, id, active: true, shape, bounding_sphere, base_bounding_sphere, manual_bounding_sphere, bounding_aabb, base_bounding_aabb, curr_pose, base_pose: base_pose_local, curr_quaternion_global_delta, curr_translation_global_delta, base_rot_mat, has_base_pose, curr_isometry, bounding_sphere_outdated: false, bounding_aabb_outdated: false}
    }

    pub fn new_capsule(half_height: f64, radius: f64, base_pose: Option<ImplicitDualQuaternion>, name: Option<String>) -> Self {
        // half height refers to JUST the cylindrical part.  The TOTAL half height of the capsule will be half_height + radius.
        let shape = Capsule::new(half_height, radius);
        CollisionObject::new(Box::new(Arc::new(shape)), base_pose, name)
    }

    pub fn new_capsule_from_cylinder_parameters(half_height: f64, radius: f64, base_pose: Option<ImplicitDualQuaternion>, name: Option<String>) -> Self {
        let cylindrical_half_height = half_height - radius;
        return Self::new_capsule(cylindrical_half_height, radius, base_pose, name);
    }

    pub fn new_cuboid(x_half: f64, y_half: f64, z_half: f64, base_pose: Option<ImplicitDualQuaternion>, name: Option<String>) -> Self {
        let cuboid = Cuboid::new(Vector3::new(x_half, y_half, z_half));
        CollisionObject::new(Box::new(Arc::new(cuboid)), base_pose, name)
    }

    pub fn new_cuboid_from_obb(obb: &OBB) -> Self {
        return obb.collision_object.spawn();
    }

    pub fn new_cuboid_from_trimesh_engine(t: &TriMeshEngine, name: Option<String>) -> Self {
        let obb = OBB::new_from_trimesh_engine(t, name);
        return Self::new_cuboid_from_obb(&obb);
    }

    pub fn new_cuboid_from_mesh_file_path(fp: String, name: Option<String>) -> Result<Self, String> {
        let obb = OBB::new_from_path(fp, name)?;
        return Ok(Self::new_cuboid_from_obb(&obb));
    }

    pub fn new_ball(radius: f64, base_pose: Option<ImplicitDualQuaternion>, name: Option<String>) -> Self {
        let ball = Ball::new(radius);
        CollisionObject::new(Box::new(Arc::new(ball)), base_pose, name)
    }

    pub fn new_convex_hull(trimesh_engine: &TriMeshEngine, name: Option<String>) -> Self {
        let convex = ConvexHull::try_from_points(&trimesh_engine.vertices);
        if convex.is_none() {
            println!("{}{}WARNING: Convex hull computation failed when trying to build collision_object.  Defaulting to a tightly fitting cuboid instead.{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            let backup_obb = OBB::new_from_trimesh_engine(trimesh_engine, name.clone());
            let collision_object = CollisionObject::new_cuboid(backup_obb.half_extents[0], backup_obb.half_extents[1], backup_obb.half_extents[2], Some(backup_obb.initial_pose.clone()), name.clone());
            return collision_object;
        }

        CollisionObject::new(Box::new(Arc::new(convex.unwrap())), None, name)
    }

    pub fn new_mesh(trimesh_engine: &TriMeshEngine, name: Option<String>) -> Self {
        let mesh = TriMesh::new( trimesh_engine.vertices.clone(), trimesh_engine.indices.clone(), None );
        return Self::new( Box::new(Arc::new(mesh)), None, name);
    }

    pub fn new_mesh_from_mesh_file_path(fp: String, name: Option<String>) -> Result<Self, String> {
        let trimesh_engine = TriMeshEngine::new_from_path(fp)?;
        return Ok( Self::new_mesh(&trimesh_engine, name) );
    }

    pub fn spawn(&self) -> Self {
        let name = self.name.clone();
        let id = self.id.clone();
        let active = self.active.clone();
        let shape = Box::new(self.shape.clone_arc());
        let bounding_sphere = self.bounding_sphere.clone();
        let base_bounding_sphere = self.base_bounding_sphere.clone();
        let manual_bounding_sphere = self.manual_bounding_sphere.clone();
        let bounding_aabb = self.bounding_aabb.clone();
        let base_bounding_aabb = self.base_bounding_aabb.clone();
        let curr_pose = self.curr_pose.clone();
        let base_pose = self.base_pose.clone();
        let curr_quaternion_global_delta = self.curr_quaternion_global_delta.clone();
        let curr_translation_global_delta = self.curr_translation_global_delta.clone();
        let base_rot_mat = self.base_rot_mat.clone();
        let has_base_pose = self.has_base_pose.clone();
        let curr_isometry = self.curr_isometry.clone();
        let bounding_sphere_outdated = self.bounding_sphere_outdated;
        let bounding_aabb_outdated = self.bounding_aabb_outdated;
        return Self {name, id, active, shape, bounding_sphere, base_bounding_sphere, manual_bounding_sphere, bounding_aabb,
            base_bounding_aabb, curr_pose, base_pose, curr_quaternion_global_delta, curr_translation_global_delta,
            base_rot_mat, has_base_pose, curr_isometry, bounding_sphere_outdated, bounding_aabb_outdated };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn proximity_check(&self, other: &CollisionObject) -> Proximity {
        return query::proximity(&self.curr_isometry, &(**self.shape), &other.curr_isometry, &(**other.shape), 0.1);
    }

    pub fn intersect_check(&self, other: &CollisionObject) -> bool {
        let p = self.proximity_check(other);
        if p == Proximity::Intersecting {
            return true;
        } else {
            return false;
        }
    }

    pub fn intersect_check_bounding_aabb(&mut self, other: &mut CollisionObject) -> bool {
        if self.bounding_aabb_outdated { self.update_bounding_aabb(); }
        if other.bounding_aabb_outdated { other.update_bounding_aabb(); }

        self.bounding_aabb.intersects(&other.bounding_aabb)
    }

    pub fn intersect_check_bounding_aabb_immutable(&self, other: &CollisionObject) -> bool {
        self.bounding_aabb.intersects(&other.bounding_aabb)
    }

    pub fn intersect_check_bounding_sphere(&mut self, other: &mut CollisionObject) -> bool {
        if self.bounding_sphere_outdated { self.update_bounding_sphere(); self.bounding_sphere_outdated = false; }
        if other.bounding_sphere_outdated { other.update_bounding_sphere(); other.bounding_sphere_outdated = false; }

        self.bounding_sphere.intersects(&other.bounding_sphere)
    }

    pub fn intersect_check_bounding_sphere_immutable(&self, other: &CollisionObject) -> bool {
        self.bounding_sphere.intersects(&other.bounding_sphere)
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn distance_check(&self, other: &CollisionObject) -> f64 {
        let dis = query::distance( &self.curr_isometry, &(**self.shape), &other.curr_isometry, &(**other.shape) );
        return dis;
    }

    pub fn distance_check_bounding_sphere(&self, other: &CollisionObject) -> f64 {
        return query::distance( &self.curr_isometry, &self.manual_bounding_sphere, &other.curr_isometry, &other.manual_bounding_sphere );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn contact_check(&self, other: &CollisionObject, margin: Option<f64>) -> Option<Contact<f64>> {
        let mut prediction = 0.2; if margin.is_some() { prediction = margin.unwrap(); }
        return query::contact( &self.curr_isometry, &(**self.shape), &other.curr_isometry, &(**other.shape), prediction  );
    }

    pub fn contact_check_bounding_sphere(&self, other: &CollisionObject, margin: Option<f64>) -> Option<Contact<f64>> {
        let mut prediction = 0.2; if margin.is_some() { prediction = margin.unwrap(); }
        return query::contact( &self.curr_isometry, &self.manual_bounding_sphere, &other.curr_isometry, &other.manual_bounding_sphere, prediction);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn intersects_ray(&self, ray_origin: &Point3<f64>, ray_direction: &Vector3<f64>) -> bool {
        let ray = Ray::new(ray_origin.clone(), ray_direction.clone());
        return self.shape.intersects_ray(&self.curr_isometry, &ray, 1.0);
    }

    pub fn intersects_ray_bounding_sphere(&self, ray_origin: &Point3<f64>, ray_direction: &Vector3<f64>) -> bool {
        let ray = Ray::new(ray_origin.clone(), ray_direction.clone());
        return self.manual_bounding_sphere.intersects_ray(&self.curr_isometry, &ray, 1.0);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn contains_point(&self, point: &Point3<f64>) -> bool {
        return self.shape.contains_point(&self.curr_isometry, point);
    }

    pub fn contains_point_bounding_sphere(&self, point: &Point3<f64>) -> bool {
        return self.manual_bounding_sphere.contains_point(&self.curr_isometry, point);
    }

    pub fn project_point(&self, point: &Point3<f64>, solid: bool) -> PointProjection<f64> {
        return self.shape.project_point(&self.curr_isometry, point, solid)
    }

    pub fn project_point_bounding_sphere(&self, point: &Point3<f64>, solid: bool) -> PointProjection<f64> {
        return self.manual_bounding_sphere.project_point(&self.curr_isometry, point, solid)
    }

    pub fn distance_to_point(&self, point: &Point3<f64>, solid: bool) -> f64 {
        return self.shape.distance_to_point(&self.curr_isometry, point, solid)
    }

    pub fn distance_to_point_bounding_sphere(&self, point: &Point3<f64>, solid: bool) -> f64 {
        return self.manual_bounding_sphere.distance_to_point(&self.curr_isometry, point, solid)
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_curr_translation(&mut self, x: f64, y: f64, z: f64) {
        self.curr_translation_global_delta = Vector3::new(x, y, z);
        if self.has_base_pose {
            let rot_pt = self.curr_quaternion_global_delta * &self.base_pose.translation;

            self.curr_pose.translation[0] = rot_pt[0] + x; self.curr_pose.translation[1] = rot_pt[1] + y; self.curr_pose.translation[2] = rot_pt[2] + z;
        } else {
            self.curr_pose.translation[0] = x; self.curr_pose.translation[1] = y; self.curr_pose.translation[2] = z;
        }
        self.update_curr_isometry();
        self.bounding_sphere_outdated = true;
        self.bounding_aabb_outdated = true;
    }

    pub fn set_curr_orientation(&mut self, w: f64, x: f64, y: f64, z: f64) {
        if self.has_base_pose {
            let q = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
            self.curr_pose.quat = q * &self.base_pose.quat;

            let rot_pt = q * &self.base_pose.translation;

            self.curr_quaternion_global_delta = q;

            /*
            let mut new_quat: UnitQuaternion<f64> = UnitQuaternion::identity();
            if !((w.abs() - 1.0).abs() < 0.000000000001) {
                let mut axis_angle = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z)).axis_angle();
                let mut new_axis = self.base_rot_mat * &axis_angle.unwrap().0.unwrap();
                new_quat = UnitQuaternion::from_axis_angle(&Unit::new_unchecked(new_axis), axis_angle.unwrap().1);
            }
            let rot_pt = new_quat * &self.base_pose.translation;
            */

            self.curr_pose.translation[0] = rot_pt[0] + (self.curr_pose.translation[0] - self.base_pose.translation[0]); self.curr_pose.translation[1] = rot_pt[1] +  (self.curr_pose.translation[1] - self.base_pose.translation[1]); self.curr_pose.translation[2] = rot_pt[2] +  (self.curr_pose.translation[2] - self.base_pose.translation[2]);
        } else {
            self.curr_pose.quat = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
        }
        self.update_curr_isometry();
        self.bounding_sphere_outdated = true;
        self.bounding_aabb_outdated = true;
    }

    pub fn set_curr_orientation_from_euler_angles(&mut self, rx: f64, ry: f64, rz: f64) {
        let quat = UnitQuaternion::from_euler_angles(rx, ry, rz);
        self.set_curr_orientation(quat[3], quat[0], quat[1], quat[2]);
    }

    pub fn set_curr_pose(&mut self, implicit_dual_quaternion: &ImplicitDualQuaternion) {
        self.set_curr_orientation(implicit_dual_quaternion.quat.w, implicit_dual_quaternion.quat.i, implicit_dual_quaternion.quat.j, implicit_dual_quaternion.quat.k);
        self.set_curr_translation(implicit_dual_quaternion.translation[0], implicit_dual_quaternion.translation[1], implicit_dual_quaternion.translation[2]);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn update_curr_isometry(&mut self) {
        let translation = Translation3::new( self.curr_pose.translation[0], self.curr_pose.translation[1], self.curr_pose.translation[2] );
        self.curr_isometry = Isometry3::from_parts(translation, self.curr_pose.quat.clone());
    }

    pub fn update_bounding_sphere(&mut self) {
        self.bounding_sphere = self.base_bounding_sphere.transform_by(&self.curr_isometry);
        self.bounding_sphere_outdated = false;
    }

    pub fn update_bounding_aabb(&mut self) {
        self.bounding_aabb = self.base_bounding_aabb.transform_by(&self.curr_isometry);
        self.bounding_aabb_outdated = false;
    }

    pub fn update_all_bounding_volumes(&mut self) {
        self.update_bounding_sphere();
        self.update_bounding_aabb();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _get_id() -> u64 {
        let mut rng = rand::thread_rng();
        let id = rng.gen_range(0, u64::max_value());
        return id
    }
}

impl Clone for CollisionObject {
    fn clone(&self) -> Self {
        return self.spawn();
    }
}

impl fmt::Debug for CollisionObject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Ok(())
    }
}





