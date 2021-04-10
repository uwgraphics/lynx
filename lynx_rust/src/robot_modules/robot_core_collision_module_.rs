use crate::utils::utils_collisions::{oriented_bounding_box::*, collision_object::CollisionObject};
use crate::robot_modules::{robot_configuration_module::RobotConfigurationModule, robot_bounds_module::RobotBoundsModule, robot_fk_module::RobotFKModule, robot_dof_module::RobotDOFModule, link::Link};
use crate::utils::utils_files_and_strings::{robot_folder_utils::*, file_utils::*};
use crate::utils::utils_parsing::urdf_parsing_utils::*;
use crate::robot_modules::link::MeshFilenameType;
use std::time::Instant;
use crate::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use crate::utils::utils_preprocessing::mesh_preprocessing_utils::convert_all_links_into_convex_shapes_and_save_mesh_files;
use termion::{color, style};
use nalgebra::{DVector};
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use crate::utils::utils_math::nalgebra_utils::vec_to_dvec;
use crate::utils::utils_collisions::collision_environment::CollisionEnvironment;
use crate::utils::utils_collisions::collision_check_result_enum::{CollisionCheckResult, CollisionCheckResult::{NotInCollision, InCollision}};



/*
pub struct RobotCoreCollisionModule {
    _link_obbs: Vec<Option<OBB>>,
    _link_convex_hulls: Vec<Option<CollisionObject>>,
    _always_in_collision_pairs: Vec<[String; 2]>,
    _never_in_collision_pairs: Vec<[String; 2]>,
    _always_in_collision_pairs_idxs: Vec<[usize; 2]>,
    _never_in_collision_pairs_idxs: Vec<[usize; 2]>,
    _skip_collision_pair_matrix: Vec<Vec<bool>>,
    _links_copy: Vec<Link>,
    _num_links: usize
}

impl RobotCoreCollisionModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule, robot_dof_module: &RobotDOFModule, robot_fk_module: &RobotFKModule, robot_bounds_module: &RobotBoundsModule) -> Self {
        let _link_obbs = Vec::new();
        let _link_convex_hulls = Vec::new();
        let _always_in_collision_pairs = Vec::new();
        let _never_in_collision_pairs = Vec::new();
        let _always_in_collision_pairs_idxs = Vec::new();
        let _never_in_collision_pairs_idxs = Vec::new();
        let _skip_collision_pair_matrix = Vec::new();
        let _links_copy = robot_configuration_module.robot_model_module.links.clone();
        let _num_links = _links_copy.len();

        let mut out_self = Self {_link_obbs, _link_convex_hulls, _always_in_collision_pairs,
            _never_in_collision_pairs, _always_in_collision_pairs_idxs, _never_in_collision_pairs_idxs,
            _skip_collision_pair_matrix, _links_copy, _num_links};

        out_self._initialize_obbs_and_convex_hulls(robot_configuration_module);
        out_self._initialize_always_in_collision_pairs(robot_configuration_module, true);
        out_self._initialize_never_in_collision_pairs(robot_configuration_module);
        out_self._initialize_skip_collision_pair_matrix();
        // out_self._initialize_link_obbs_smaller();

        return out_self;
    }

    pub fn spawn(&self) -> Self {
        let mut _link_obbs = Vec::new();
        let l = self._link_obbs.len();
        for i in 0..l {
            if self._link_obbs[i].is_none() { _link_obbs.push(None); }
            else { _link_obbs.push( Some( self._link_obbs[i].as_ref().unwrap().spawn() ) ); }
        }

        let mut _link_convex_hulls = Vec::new();
        let l = self._link_convex_hulls.len();
        for i in 0..l {
            if self._link_convex_hulls[i].is_none() { _link_convex_hulls.push(None); }
            else { _link_convex_hulls.push( Some( self._link_convex_hulls[i].as_ref().unwrap().spawn() ) ); }
        }

        let _always_in_collision_pairs = self._always_in_collision_pairs.clone();
        let _never_in_collision_pairs = self._never_in_collision_pairs.clone();
        let _always_in_collision_pairs_idxs = self._always_in_collision_pairs_idxs.clone();
        let _never_in_collision_pairs_idxs = self._never_in_collision_pairs_idxs.clone();
        let _skip_collision_pair_matrix = self._skip_collision_pair_matrix.clone();
        let _links_copy = self._links_copy.clone();
        let _num_links = self._num_links.clone();

        return Self {_link_obbs, _link_convex_hulls, _always_in_collision_pairs, _never_in_collision_pairs,
            _always_in_collision_pairs_idxs, _never_in_collision_pairs_idxs, _skip_collision_pair_matrix,
            _links_copy, _num_links};
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn self_collision_check(&mut self, fk_res: &Vec<Option<ImplicitDualQuaternion>>) -> CollisionCheckResult {
        // let fk_res = robot_fk_module.compute_fk(x);

        self._set_poses_on_link_obbs(&fk_res);
        // self._set_poses_on_link_obbs_smaller(&fk_res);
        // self._set_poses_on_link_convex_hulls(&fk_res);

        for i in 0..self._num_links {
            for j in 0..self._num_links {
                if i >= j { continue; }
                if self._skip_collision_pair(i, j) { continue; }
                if fk_res[i].is_none() || fk_res[j].is_none() { continue; }

                // let check1 = self._bounding_sphere_collision_check_between_links(i, j);
                // if check1 {
                let check1 = self._bounding_aabb_collision_check_between_links(i, j);
                if check1 {
                    let check2 = self._obb_collision_check_between_links(i, j);
                    if check2 {
                        return InCollision( format!("self-collision detected between links {:?} and {:?}", self._links_copy[i].name, self._links_copy[j].name) );
                        /*
                            if use_convex_hulls_if_necessary {
                                let check4 = self._convex_hull_collision_check_between_links(i, j);
                                if check4 {
                                    return true;
                                }
                            } else {
                                return true;
                            }
                            */
                    }
                }
                // }
            }
        }

        return NotInCollision;
    }

    pub fn self_collision_check_with_fk_computation(&mut self, x: &DVector<f64>, robot_fk_module: &RobotFKModule) -> CollisionCheckResult {
        let fk_res = robot_fk_module.compute_fk(x);
        return self.self_collision_check(&fk_res);
    }

    pub fn environment_collision_check(&mut self, fk_res: &Vec<Option<ImplicitDualQuaternion>>, collision_environment: &CollisionEnvironment) -> CollisionCheckResult {
        let num_environment_objects = collision_environment.environment_obbs.len();

        self._set_poses_on_link_obbs(&fk_res);

        for i in 0..self._num_links {
            if fk_res[i].is_some() && self._link_obbs[i].is_some() {
                self._update_obb_bounding_aabb_if_necessary(i);
                // self._update_obb_bounding_sphere_if_necessary(i);

                for j in 0..num_environment_objects {
                    let l = collision_environment.environment_obbs[j].len();
                    for k in 0..l {
                        let check1 = self._link_obbs[i].as_ref().unwrap().intersect_check_bounding_aabb_immutable( &collision_environment.environment_obbs[j][k] );
                        if check1 {
                            let check2 = self._link_obbs[i].as_ref().unwrap().intersect_check( &collision_environment.environment_obbs[j][k] );
                            if check2 {
                                return InCollision( format!("environment-collision detected between link {:?} and environment", self._links_copy[i].name) );
                            }
                        }
                    }
                }

            }
        }

        return NotInCollision;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _skip_collision_pair(&self, link_idx_1: usize, link_idx_2: usize) -> bool {
        return self._skip_collision_pair_matrix[link_idx_1][link_idx_2];
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _initialize_obbs_and_convex_hulls(&mut self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let check = check_if_path_exists_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_shapes".to_string());
        if !check {
            convert_all_links_into_convex_shapes_and_save_mesh_files(robot_name.clone());
        }
        let check = check_if_path_exists_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_shapes".to_string());
        if !check {
            println!("{}{}WARNING: RobotCoreCollisionModule could not be initialized correctly because mesh files for robot were not found.  {}", color::Fg(color::Yellow), style::Bold, style::Reset);
            return Ok(());
        }

        let filenames = get_all_files_in_directory_with_extension_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_shapes".to_string(), "stl".to_string());
        let mut filenames_without_extension: Vec<String> = filenames.iter().map( |x| get_filename_without_extension(x.clone()) ).collect();

        let mut fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_convex_shapes/";

        let l1 = robot_configuration_module.robot_model_module.links.len();
        let l2 = filenames_without_extension.len();
        for i in 0..l1 {
            let mut found = false;
            for j in 0..l2 {
                if !found && filenames_without_extension[j].clone() == self._links_copy[i].name.clone() {
                    found = true;
                    let full_fp = fp.clone() + filenames[j].as_str();
                    let mut trimesh_engine = TriMeshEngine::new_from_path(full_fp.clone())?;
                    let obb = OBB::new_from_path(full_fp.clone(), None)?;
                    let convex_hull = CollisionObject::new_convex_hull(&trimesh_engine, None);
                    self._link_obbs.push(Some(obb));
                    self._link_convex_hulls.push( Some(convex_hull) );
                }
            }
            if !found {
                self._link_obbs.push( None );
                self._link_convex_hulls.push(None);
            }
        }

        Ok(())
    }

    fn _initialize_always_in_collision_pairs(&mut self, robot_configuration_module: &RobotConfigurationModule, include_all_zeros_config_collisions: bool) {
        let mut loaded_pairs = self._load_always_in_collision_pairs_from_file(robot_configuration_module);
        if loaded_pairs.is_some() {
            self._always_in_collision_pairs = loaded_pairs.unwrap();
        } else {
            println!("{}{}I see that always in collision pairs have not been calculated yet.  Doing that now...{}", color::Fg(color::Cyan), style::Bold, style::Reset);
            let always_in_collision_pairs = self._calculate_always_in_collision_pairs(robot_configuration_module, include_all_zeros_config_collisions, 10_000);
            self._save_always_in_collision_pairs_to_file(robot_configuration_module, &always_in_collision_pairs);
            self._always_in_collision_pairs = always_in_collision_pairs;
        }

        self._always_in_collision_pairs_idxs = self._always_in_collision_pairs.iter().map(
            |x| [robot_configuration_module.robot_model_module.get_link_idx_from_name(&x[0]).unwrap(),
                robot_configuration_module.robot_model_module.get_link_idx_from_name(&x[1]).unwrap()] ).collect();
    }

    fn _initialize_never_in_collision_pairs(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        let mut loaded_pairs = self._load_never_in_collision_pairs_from_file(robot_configuration_module);
        if loaded_pairs.is_some() {
            self._never_in_collision_pairs = loaded_pairs.unwrap();
        } else {
            println!("{}{}I see that never in collision pairs have not been calculated yet.  Doing that now...{}", color::Fg(color::Cyan), style::Bold, style::Reset);
            let never_in_collision_pairs = self._calculate_never_in_collision_pairs(robot_configuration_module, 50_000);
            self._save_never_in_collision_pairs_to_file(robot_configuration_module, &never_in_collision_pairs);
            self._never_in_collision_pairs = never_in_collision_pairs;
        }
        self._never_in_collision_pairs_idxs = self._never_in_collision_pairs.iter().map(
            |x| [robot_configuration_module.robot_model_module.get_link_idx_from_name(&x[0]).unwrap(),
                robot_configuration_module.robot_model_module.get_link_idx_from_name(&x[1]).unwrap() ] ).collect();
    }

    fn _initialize_skip_collision_pair_matrix(&mut self) {
        self._skip_collision_pair_matrix = vec![ vec![ false; self._num_links ]; self._num_links ];
        for i in 0..self._num_links {
            for j in 0..self._num_links {
                if self._always_in_collision_pairs_idxs.contains(&[i, j]) || self._never_in_collision_pairs_idxs.contains(&[i, j]) {
                    self._skip_collision_pair_matrix[i][j] = true;
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _calculate_always_in_collision_pairs(&mut self, robot_configuration_module: &RobotConfigurationModule, include_all_zeros_config_collisions: bool, num_samples: usize) -> Vec< [String ; 2] > {
        let mut out_vec = Vec::new();

        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let robot_configuration_module_base = RobotConfigurationModule::new_base_configuration(robot_name.clone());
        let robot_dof_module_base = RobotDOFModule::new(&robot_configuration_module_base);
        let robot_bounds_module_base = RobotBoundsModule::new(&robot_configuration_module_base, &robot_dof_module_base, None);
        let robot_fk_module_base = RobotFKModule::new(&robot_configuration_module_base, &robot_dof_module_base);

        let mut collision_matrix = vec![ vec![ 0.0 ; self._num_links ] ; self._num_links ];

        for i in 0..num_samples {
            if i % 100 == 0 {
                println!("{}{}Calculating always in collision pairs matrix: sample {} of {} {}", color::Fg(color::Blue), style::Bold, i, num_samples, style::Reset);
            }
            let x = robot_bounds_module_base.uniform_sample_from_bounds();
            let res = self._self_collision_check_obb_naive(&x, &robot_fk_module_base);
            let l = res.len();
            for j in 0..l {
                let link_idx_1 = res[j].0;
                let link_idx_2 = res[j].1;

                collision_matrix[link_idx_1][link_idx_2] +=1.0;
            }
        }

        for i in 0..self._num_links {
            for j in 0..self._num_links {
                if i >= j { continue; }
                if collision_matrix[i][j] >= 0.94*num_samples as f64 {
                    out_vec.push( [self._links_copy[i].name.clone(), self._links_copy[j].name.clone()] )
                }
            }
        }

        if include_all_zeros_config_collisions {
            println!("{}{}NOTE: I'm considering any collisions that occur in the all zeros configuration as being always-in-collision pairs.  \
            If this is incorrect, i.e., the all zeros configuration ACTUALLY has collisions, this will lead to errors!{}", color::Fg(color::Cyan), style::Bold, style::Reset);
            let x = vec_to_dvec( &vec![ 0.0; robot_dof_module_base.num_dofs ] );
            let res = self._self_collision_check_obb_naive(&x, &robot_fk_module_base);
            let l = res.len();
            for i in 0..l {
                if !out_vec.contains( &[self._links_copy[res[i].0].name.clone(), self._links_copy[res[i].1].name.clone()] ) {
                    out_vec.push([self._links_copy[res[i].0].name.clone(), self._links_copy[res[i].1].name.clone()]);
                }
            }
        }

        return out_vec;
    }

    fn _calculate_never_in_collision_pairs(&mut self, robot_configuration_module: &RobotConfigurationModule, num_samples: usize) -> Vec< [String;2] > {
        let mut out_vec = Vec::new();

        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let robot_configuration_module_base = RobotConfigurationModule::new_base_configuration(robot_name.clone());
        let robot_dof_module_base = RobotDOFModule::new(&robot_configuration_module_base);
        let robot_bounds_module_base = RobotBoundsModule::new(&robot_configuration_module_base, &robot_dof_module_base, None);
        let robot_fk_module_base = RobotFKModule::new(&robot_configuration_module_base, &robot_dof_module_base);

        let mut collision_matrix = vec![ vec![ 0.0 ; self._num_links ] ; self._num_links ];

        for i in 0..num_samples {
            if i % 1000 == 0 {
                println!("{}{}Calculating never in collision pairs matrix: sample {} of {} {}", color::Fg(color::Blue), style::Bold, i, num_samples, style::Reset);
            }
            let x = robot_bounds_module_base.uniform_sample_from_bounds();
            let res = self._self_collision_check_convex_hull_naive(&x, &robot_fk_module_base);
            let l = res.len();
            for j in 0..l {
                let link_idx_1 = res[j].0;
                let link_idx_2 = res[j].1;

                collision_matrix[link_idx_1][link_idx_2] +=1.0;
            }
        }

        for i in 0..self._num_links {
            for j in 0..self._num_links {
                if i >= j { continue; }
                if collision_matrix[i][j] == 0.0 && self._link_convex_hulls[i].is_some() && self._link_convex_hulls[j].is_some() {
                    out_vec.push( [self._links_copy[i].name.clone(), self._links_copy[j].name.clone()] )
                }
            }
        }

        return out_vec;
    }

    fn _self_collision_check_obb_naive(&mut self, x: &DVector<f64>, robot_fk_module: &RobotFKModule) -> Vec< (usize, usize) > {
        let mut out_vec = Vec::new();

        let fk_res = robot_fk_module.compute_fk(x);
        self._set_poses_on_link_obbs(&fk_res);

        let l = self._num_links;
        for i in 0..l {
            for j in 0..l {
                if i >= j { continue; }
                let check1 = self._bounding_sphere_collision_check_between_links(i, j);
                if check1 {
                    let check2 = self._bounding_aabb_collision_check_between_links(i, j);
                    if check2 {
                        let check3 = self._obb_collision_check_between_links(i, j);
                        if check3 { out_vec.push( (i, j) ); }
                    }
                }
            }
        }

        return out_vec;
    }

    fn _self_collision_check_convex_hull_naive(&mut self, x: &DVector<f64>, robot_fk_module: &RobotFKModule) -> Vec< (usize, usize) > {
        let mut out_vec = Vec::new();

        let fk_res = robot_fk_module.compute_fk(x);
        self._set_poses_on_link_obbs(&fk_res);
        self._set_poses_on_link_convex_hulls(&fk_res);

        let l = self._num_links;
        for i in 0..l {
            for j in 0..l {
                if i >= j { continue; }
                let check1 = self._bounding_sphere_collision_check_between_links(i, j);
                if check1 {
                    let check2 = self._bounding_aabb_collision_check_between_links(i, j);
                    if check2 {
                        let check3 = self._convex_hull_collision_check_between_links(i, j);
                        if check3 { out_vec.push( (i, j) ); }
                    }
                }
            }
        }

        return out_vec;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _convex_hull_collision_check_between_links(&mut self, link_idx_1: usize, link_idx_2: usize) -> bool {
        if self._link_convex_hulls[link_idx_1].is_none() || self._link_convex_hulls[link_idx_2].is_none() { return false; }

        return self._link_convex_hulls[link_idx_1].as_ref().unwrap().intersect_check( &self._link_convex_hulls[link_idx_2].as_ref().unwrap() );
    }

    fn _obb_collision_check_between_links(&mut self, link_idx_1: usize, link_idx_2: usize) -> bool {
        if self._link_obbs[link_idx_1].is_none() || self._link_obbs[link_idx_2].is_none() { return false; }

        return self._link_obbs[link_idx_1].as_ref().unwrap().intersect_check( &self._link_obbs[link_idx_2].as_ref().unwrap() );
    }

    fn _bounding_sphere_collision_check_between_links(&mut self, link_idx_1: usize, link_idx_2: usize) -> bool {
        if self._link_obbs[link_idx_1].is_none() || self._link_obbs[link_idx_2].is_none() { return false; }

        self._update_obb_bounding_sphere_if_necessary(link_idx_1);
        self._update_obb_bounding_sphere_if_necessary(link_idx_2);
        return self._link_obbs[link_idx_1].as_ref().unwrap().intersect_check_bounding_sphere_immutable( &self._link_obbs[link_idx_2].as_ref().unwrap() );
    }

    fn _bounding_aabb_collision_check_between_links(&mut self, link_idx_1: usize, link_idx_2: usize) -> bool {
        if self._link_obbs[link_idx_1].is_none() || self._link_obbs[link_idx_2].is_none() { return false; }

        self._update_obb_bounding_aabb_if_necessary(link_idx_1);
        self._update_obb_bounding_aabb_if_necessary(link_idx_2);
        return self._link_obbs[link_idx_1].as_ref().unwrap().intersect_check_bounding_aabb_immutable( &self._link_obbs[link_idx_2].as_ref().unwrap() );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _set_poses_on_link_obbs(&mut self, fk_res: &Vec<Option<ImplicitDualQuaternion>>) {
        let l = fk_res.len();
        for i in 0..l {
            if fk_res[i].is_some() && self._link_obbs[i].is_some() {
                self._link_obbs[i].as_mut().unwrap().set_curr_pose( &fk_res[i].as_ref().unwrap() );
            }
        }
    }

    fn _set_poses_on_link_convex_hulls(&mut self, fk_res: &Vec<Option<ImplicitDualQuaternion>>) {
        let l = fk_res.len();
        for i in 0..l {
            if fk_res[i].is_some() && self._link_convex_hulls[i].is_some() {
                self._link_convex_hulls[i].as_mut().unwrap().set_curr_pose( &fk_res[i].as_ref().unwrap() );
            }
        }
    }

    fn _update_obb_bounding_sphere_if_necessary(&mut self, link_idx: usize) {
        if self._link_obbs[link_idx].as_ref().unwrap().collision_object.bounding_sphere_outdated { self._link_obbs[link_idx].as_mut().unwrap().collision_object.update_bounding_sphere(); }
    }

    fn _update_obb_bounding_aabb_if_necessary(&mut self, link_idx: usize) {
        if self._link_obbs[link_idx].as_ref().unwrap().collision_object.bounding_aabb_outdated { self._link_obbs[link_idx].as_mut().unwrap().collision_object.update_bounding_aabb(); }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _save_always_in_collision_pairs_to_file(&self, robot_configuration_module: &RobotConfigurationModule, always_in_collision_pairs: &Vec<[String; 2]>) {
        let serialized = serde_json::to_string(always_in_collision_pairs).unwrap();
        write_string_to_file_relative_to_robot_directory( robot_configuration_module.robot_model_module.robot_name.clone(), "autogenerated_metadata".to_string(), "always_in_collision_pairs.json".to_string(), serialized, true );
    }

    fn _load_always_in_collision_pairs_from_file(&self, robot_configuration_module: &RobotConfigurationModule) -> Option<Vec<[String; 2]>> {
        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let partial_fp = "autogenerated_metadata/always_in_collision_pairs.json".to_string();
        let json_string = read_file_contents_relative_to_robot_directory(robot_name.clone(), partial_fp);
        if json_string.is_none() { return None; }
        else {
            return Some(serde_json::from_str(&json_string.unwrap()).unwrap());
        }
    }

    fn _save_never_in_collision_pairs_to_file(&self, robot_configuration_module: &RobotConfigurationModule, never_in_collision_pairs: &Vec<[String; 2]>) {
        let serialized = serde_json::to_string(never_in_collision_pairs).unwrap();
        write_string_to_file_relative_to_robot_directory( robot_configuration_module.robot_model_module.robot_name.clone(), "autogenerated_metadata".to_string(), "never_in_collision_pairs.json".to_string(), serialized, true );
    }

    fn _load_never_in_collision_pairs_from_file(&self, robot_configuration_module: &RobotConfigurationModule) -> Option<Vec<[String; 2]>> {
        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let partial_fp = "autogenerated_metadata/never_in_collision_pairs.json".to_string();
        let json_string = read_file_contents_relative_to_robot_directory(robot_name.clone(), partial_fp);
        if json_string.is_none() { return None; }
        else {
            return Some(serde_json::from_str(&json_string.unwrap()).unwrap());
        }
    }
}
*/