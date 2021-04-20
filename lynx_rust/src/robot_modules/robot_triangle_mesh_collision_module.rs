use crate::utils::utils_collisions::prelude::*;
use crate::robot_modules::{robot_configuration_module::RobotConfigurationModule, robot_dof_module::RobotDOFModule, robot_fk_module::*, robot_bounds_module::RobotBoundsModule};
use crate::utils::utils_preprocessing::mesh_preprocessing_utils::*;
use crate::utils::utils_files_and_strings::{file_utils::*, robot_folder_utils::*};
use crate::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use crate::utils::utils_files_and_strings::string_utils::usize_to_string;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use termion::{style, color};
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;

#[derive(Clone)]
pub struct RobotTriangleMeshCollisionModule {
    _link_triangle_meshes: Vec<Vec<CollisionObject>>,
    _link_triangle_meshes_skip_collision_check_tensor: BoolCollisionCheckTensor,
    _robot_name_copy: String
}

impl RobotTriangleMeshCollisionModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule, robot_fk_module: &RobotFKModule, robot_bounds_module: &RobotBoundsModule) -> Result<Self, String> {
        let _link_triangle_meshes = Vec::new();
        let _link_triangle_meshes_skip_collision_check_tensor = BoolCollisionCheckTensor::new_empty();
        let _robot_name_copy = robot_configuration_module.robot_model_module.robot_name.clone();

        let mut out_self = Self {_link_triangle_meshes, _link_triangle_meshes_skip_collision_check_tensor, _robot_name_copy};

        Self::_create_link_triangle_meshes_if_need_be(robot_configuration_module);

        out_self._create_link_triangle_meshes(robot_configuration_module);
        out_self._load_or_create_link_skip_collision_check_tensor(robot_fk_module, robot_bounds_module, 5, false);

        return Ok(out_self);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn self_intersect_check(&mut self, fk_res: &RobotFKResult, stop_at_first_detected: bool) -> Result<IntersectCheckMultipleResult, String> {
        self._set_poses_on_link_triangle_meshes(fk_res);
        return intersect_check_between_multiple_collision_objects( &self._link_triangle_meshes, &self._link_triangle_meshes, stop_at_first_detected, Some(&self._link_triangle_meshes_skip_collision_check_tensor));
    }

    pub fn self_distance_check(&mut self, fk_res: &RobotFKResult, stop_at_first_detected_intersection: bool) -> Result<DistanceCheckMultipleResult, String> {
        self._set_poses_on_link_triangle_meshes(fk_res);
        return distance_check_between_multiple_collision_objects( &self._link_triangle_meshes, &self._link_triangle_meshes, stop_at_first_detected_intersection, Some(&self._link_triangle_meshes_skip_collision_check_tensor), None);
    }

    pub fn self_contact_check(&mut self, fk_res: &RobotFKResult, stop_at_first_detected_intersection: bool, margin: Option<f64>) -> Result<ContactCheckMultipleResult, String> {
        self._set_poses_on_link_triangle_meshes(fk_res);
        return contact_check_between_multiple_collision_objects( &self._link_triangle_meshes, &self._link_triangle_meshes, stop_at_first_detected_intersection, margin, Some(&self._link_triangle_meshes_skip_collision_check_tensor), None);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn environment_intersect_check(&mut self, fk_res: &RobotFKResult, collision_environment: &CollisionEnvironment, stop_at_first_detected: bool) -> Result<IntersectCheckMultipleResult, String> {
        self._set_poses_on_link_triangle_meshes(fk_res);
        return intersect_check_between_multiple_collision_objects( &collision_environment.environment_obbs, &self._link_triangle_meshes, stop_at_first_detected, Some(&self._link_triangle_meshes_skip_collision_check_tensor));
    }

    pub fn environment_distance_check(&mut self, fk_res: &RobotFKResult, collision_environment: &CollisionEnvironment, stop_at_first_detected_intersection: bool) -> Result<DistanceCheckMultipleResult, String> {
        self._set_poses_on_link_triangle_meshes(fk_res);
        return distance_check_between_multiple_collision_objects( &collision_environment.environment_obbs, &self._link_triangle_meshes, stop_at_first_detected_intersection, Some(&self._link_triangle_meshes_skip_collision_check_tensor), None);
    }

    pub fn environment_contact_check(&mut self, fk_res: &RobotFKResult, collision_environment: &CollisionEnvironment, stop_at_first_detected_intersection: bool, margin: Option<f64>) -> Result<ContactCheckMultipleResult, String> {
        self._set_poses_on_link_triangle_meshes(fk_res);
        return contact_check_between_multiple_collision_objects( &collision_environment.environment_obbs, &self._link_triangle_meshes, stop_at_first_detected_intersection, margin, Some(&self._link_triangle_meshes_skip_collision_check_tensor), None);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _create_link_triangle_meshes_if_need_be(robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let exists = check_if_path_exists_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_triangle_meshes".to_string());
        if !exists {
            save_all_links_as_triangle_meshes(robot_name.clone())?;
        }
        Ok(())
    }

    fn _get_link_triangle_meshes_trimesh_engines(robot_configuration_module: &RobotConfigurationModule) -> Result<Vec<Option<TriMeshEngine>>, String> {
        let robot_name= robot_configuration_module.robot_model_module.robot_name.clone();

        let mut out_vec = Vec::new();

        let file_names = get_all_files_in_directory_with_extension_relative_to_robot_directory( robot_name.clone(), "autogenerated_metadata/link_triangle_meshes".to_string(), "stl".to_string() );
        let num_files = file_names.len();

        let num_links = robot_configuration_module.robot_model_module.links.len();
        for i in 0..num_links {
            let link_name = robot_configuration_module.robot_model_module.links[i].name.clone();
            let link_name_len = link_name.len();
            let mut found_idx = usize::max_value();
            for j in 0..num_files {
                if file_names[j].contains( &link_name ) && file_names[j].len() == link_name_len + 4 {
                    found_idx = j; break;
                }
            }

            if found_idx == usize::max_value() {
                out_vec.push(None);
            } else {
                let fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_triangle_meshes/" + file_names[found_idx].as_str();
                out_vec.push( Some(TriMeshEngine::new_from_path(fp)?) );
            }
        }

        return Ok(out_vec);
    }

    fn _create_link_triangle_meshes(&mut self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let triangle_mesh_trimesh_engines = Self::_get_link_triangle_meshes_trimesh_engines(robot_configuration_module)?;

        self._link_triangle_meshes = Vec::new();

        let l = triangle_mesh_trimesh_engines.len();
        for i in 0..l {
            if triangle_mesh_trimesh_engines[i].is_none() { self._link_triangle_meshes.push( Vec::new() ); }
            else {
                let triangle_mesh = CollisionObject::new_mesh(&triangle_mesh_trimesh_engines[i].as_ref().unwrap(), Some( robot_configuration_module.robot_model_module.links[i].name.clone() ));

                self._link_triangle_meshes.push( vec![triangle_mesh] );
            }
        }

        return Ok(());
    }

    fn _set_poses_on_link_triangle_meshes(&mut self, fk_res: &RobotFKResult) {
        let link_frames = fk_res.get_link_frames_ref();
        let l = link_frames.len();
        for i in 0..l {
            if link_frames[i].is_some() && !self._link_triangle_meshes[i].is_empty() {
                let l2 = self._link_triangle_meshes[i].len();
                for j in 0..l2 {
                    self._link_triangle_meshes[i][j].set_curr_pose( &link_frames[i].as_ref().unwrap() );
                    self._link_triangle_meshes[i][j].update_bounding_aabb();
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _load_or_create_link_skip_collision_check_tensor(&mut self, robot_fk_module: &RobotFKModule, robot_bounds_module: &RobotBoundsModule, num_samples: usize, create_new_no_matter_what: bool) -> Result<(), String> {
        let load_result = BoolCollisionCheckTensor::load_from_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_triangle_mesh_skip_collision_check_tensor.json".to_string());
        if load_result.is_ok() && !create_new_no_matter_what {
            self._link_triangle_meshes_skip_collision_check_tensor = load_result.ok().unwrap();
            return Ok(());
        }


        let mut count_collision_check_tensor = FloatCollisionCheckTensor::new(&self._link_triangle_meshes, &self._link_triangle_meshes);
        let mut initial_skip_collision_check_tensor = BoolCollisionCheckTensor::new(&self._link_triangle_meshes, &self._link_triangle_meshes, SkipCheckForSelfCollisionMode::SameObjectOnly);

        for i in 0..num_samples {
            println!("{}{}Calculating skip collision tensor for TriangleMesh: sample {} of {} {}", color::Fg(color::Blue), style::Bold, i, num_samples, style::Reset);

            let sample = robot_bounds_module.uniform_sample_from_bounds();
            let fk_res = robot_fk_module.compute_fk(&sample)?;
            self._set_poses_on_link_triangle_meshes(&fk_res);

            let intersect_check_multiple_result = intersect_check_between_multiple_collision_objects(&self._link_triangle_meshes, &self._link_triangle_meshes, false, Some(&initial_skip_collision_check_tensor))?;
            match intersect_check_multiple_result {
                IntersectCheckMultipleResult::NoIntersectionsFound(i) => { count_collision_check_tensor.increment_count_at_given_idxs( &vec![ ] ); },
                IntersectCheckMultipleResult::IntersectionFound(i) => { count_collision_check_tensor.increment_count_at_given_idxs( i.get_intersection_idxs() );  }
            }
        }

        let new_tensor = BoolCollisionCheckTensor::new_for_always_in_collision_pairs(&count_collision_check_tensor, Some(0.999), SkipCheckForSelfCollisionMode::SameObjectOnly)?;
        self._link_triangle_meshes_skip_collision_check_tensor = new_tensor;
        self._link_triangle_meshes_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_triangle_mesh_skip_collision_check_tensor.json".to_string());
        self._link_triangle_meshes_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_triangle_mesh_skip_collision_check_tensor_permanent.json".to_string());

        Ok(())
    }
}