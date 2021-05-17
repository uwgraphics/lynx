use crate::utils::utils_collisions::{oriented_bounding_box::OBB, collision_object::CollisionObject, triangle_mesh_engine::TriMeshEngine};
use crate::utils::utils_files_and_strings::{file_utils::*, string_utils::*};
use crate::utils::utils_se3::prelude::ImplicitDualQuaternion;
use termion::{color, style};
use ncollide3d::math::Point;
use serde::{Serialize, Deserialize};
use nalgebra::Vector3;

#[derive(Debug)]
pub struct CollisionEnvironment {
    pub environment_name: String,
    pub environment_obbs: Vec<Vec<CollisionObject>>, // each sub-vector represents parts of individual "objects"
    pub object_names: Vec<String>,
    pub trimesh_engines: Vec<Vec<TriMeshEngine>>,
    pub original_file_directories: Vec<String>,
    pub transforms: Vec<ImplicitDualQuaternion>
}

impl CollisionEnvironment {
    pub fn new(environment_name: &str) -> Result<Self, String> {
        let environment_obbs = Vec::new();
        let object_names = Vec::new();
        let trimesh_engines = Vec::new();
        let original_file_directories = Vec::new();
        let transforms = Vec::new();

        let mut out_self = Self {
            environment_name: environment_name.to_string(),
            environment_obbs,
            object_names,
            trimesh_engines,
            original_file_directories,
            transforms
        };
        out_self._load_environment_obbs(&environment_name.to_string())?;
        out_self._set_bounding_volumes();

        return Ok(out_self);
    }

    pub fn spawn(&self) -> Self {
        let mut environment_obbs = Vec::new();
        let l1 = self.environment_obbs.len();
        for i in 0..l1 {
            let l2 = self.environment_obbs[i].len();
            environment_obbs.push(Vec::new());
            for j in 0..l2 {
                environment_obbs[i].push( self.environment_obbs[i][j].spawn() );
            }
        }

        let object_names = self.object_names.clone();

        let mut _trimesh_engines = Vec::new();
        let l1 = self.trimesh_engines.len();
        for i in 0..l1 {
            let l2 = self.trimesh_engines[i].len();
            _trimesh_engines.push(Vec::new());
            for j in 0..l2 {
                _trimesh_engines[i].push( self.trimesh_engines[i][j].clone() );
            }
        }

        return Self {environment_name: self.environment_name.clone(),
            environment_obbs, object_names,
            trimesh_engines: _trimesh_engines,
            original_file_directories: self.original_file_directories.clone(),
            transforms: self.transforms.clone() };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn save_environment_obbs_metadata(&mut self, environment_name: &str) {
        let mut vertices: Vec<Vec<Vec<Point<f64>>>> = Vec::new();
        let mut indices: Vec<Vec<Vec<Point<usize>>>> = Vec::new();
        for i in 0..self.trimesh_engines.len() {
            vertices.push(Vec::new());
            indices.push(Vec::new());
            for j in 0..self.trimesh_engines[i].len() {
                vertices[i].push( self.trimesh_engines[i][j].vertices.clone() );
                indices[i].push( self.trimesh_engines[i][j].indices.clone() );
            }
        }

        let fp = get_path_to_src() + "assets/mesh_environments/" + environment_name;
        let serialized = serde_json::to_string( &(vertices, indices, self.object_names.clone(), self.original_file_directories.clone(), self.transforms.clone()) ).ok().unwrap();
        write_string_to_file(fp, "metadata.json".to_string(), serialized, true );
    }

    fn _load_environment_obbs(&mut self, environment_name: &String) -> Result<(), String> {
        let all_available_environments = self._get_all_available_environment_names();
        if !all_available_environments.contains(environment_name) {
            println!("{}{}ERROR: environment {} not found as valid environment. {}", color::Fg(color::Red), style::Bold, environment_name, style::Reset);
            return Err(format!("environment {} not found as valid environment.", environment_name));
        }

        let mesh_filenames = self._get_all_mesh_files_in_particular_environment_directory(environment_name);

        let l = mesh_filenames.len();
        for i in 0..l {
            let ext = get_filename_extension(mesh_filenames[i].clone());
            if !(mesh_filenames[i].clone() == "metadata.json".to_string()) && !(ext == "glb") {
                self.object_names.push( get_filename_without_extension(mesh_filenames[i].clone()) );
            }
        }

        println!("{}{}Loading environment {}... {}", color::Fg(color::Cyan), style::Bold, environment_name, style::Reset);

        let metadata_fp = get_path_to_src() + "assets/mesh_environments/" + environment_name.as_str() + "/metadata.json";

        let metadata_exists = check_if_path_exists(metadata_fp.clone());
        let mut loaded_from_metadata = false;
        if metadata_exists {
            let fp = get_path_to_src() + "assets/mesh_environments/" + environment_name.as_str();
            let mut directory_updated_since_metadata_was_created = check_if_file1_modified_after_file2(fp.clone(), metadata_fp.clone());

            if (directory_updated_since_metadata_was_created.is_some() && !directory_updated_since_metadata_was_created.unwrap()) || self.object_names.len() == 0 {
                self._load_environment_from_metadata(environment_name.clone());
                return Ok(());
            }
        }

        let mut count = 0 as usize;
        if !loaded_from_metadata {
            let l = mesh_filenames.len();
            for i in 0..l {
                if mesh_filenames[i] == "metadata.json".to_string() { continue; }
                let ext = get_filename_extension(mesh_filenames[i].clone());
                if ext == "glb" {
                    continue
                };

                self.environment_obbs.push(Vec::new());
                let fp = get_path_to_src() + "assets/mesh_environments/" + environment_name.as_str() + "/" + mesh_filenames[i].as_str();
                let mut t = TriMeshEngine::new_from_path(fp)?;
                if t.vertices.len() == 0 {
                    println!("{}{}WARNING: Object {} in environment {} was empty.  {}", color::Fg(color::Yellow), style::Bold, mesh_filenames[i], environment_name, style::Reset);
                }
                let mut ts = t.split_into_convex_components(0.08, 0);
                let l = ts.len();
                for j in 0..l {
                    self.environment_obbs[count].push( CollisionObject::new_cuboid_from_trimesh_engine(&ts[j], Some( mesh_filenames[i].clone() + format!("_{:?}", j).as_str() )) );
                }
                self.trimesh_engines.push(ts);

                self.original_file_directories.push(environment_name.to_string());
                self.transforms.push(ImplicitDualQuaternion::new_identity());

                count += 1;
            }
            self.save_environment_obbs_metadata(environment_name.as_str());
        }

        Ok(())
    }

    fn _load_environment_from_metadata(&mut self, environment_name: String) {
        let metadata_fp = get_path_to_src() + "assets/mesh_environments/" + environment_name.as_str() + "/metadata.json";

        let json_string = read_file_contents(metadata_fp);
        if json_string.is_none() { return; }

        let metadata: (Vec<Vec<Vec<Point<f64>>>>,Vec<Vec<Vec<Point<usize>>>>, Vec<String>, Vec<String>, Vec<ImplicitDualQuaternion>) = serde_json::from_str(&json_string.unwrap()).unwrap();

        let (vertices, indices, object_names, original_file_directories, transforms) = &metadata;

        self.object_names = object_names.clone();
        self.original_file_directories = original_file_directories.clone();
        self.transforms = transforms.clone();

        let l = vertices.len();
        for i in 0..l {
            self.environment_obbs.push(Vec::new());
            self.trimesh_engines.push(Vec::new());

            let l2 = vertices[i].len();
            for j in 0..l2 {
                let mut t = TriMeshEngine::new( vertices[i][j].clone(), indices[i][j].clone() );
                self.environment_obbs[i].push(CollisionObject::new_cuboid_from_trimesh_engine(&t, Some(self.object_names[i].clone() + format!("_{:?}", j).as_str())));
                self.trimesh_engines[i].push(t);
            }
        }

        for i in 0..l {
            let l2 = self.environment_obbs[i].len();
            for j in 0..l2 {
                self.environment_obbs[i][j].set_curr_pose(&self.transforms[i]);
            }
        }
    }

    fn _set_bounding_volumes(&mut self) {
        let l = self.environment_obbs.len();
        for i in 0..l {
            self.environment_obbs[i].iter_mut().for_each(|x| x.update_bounding_aabb());
            self.environment_obbs[i].iter_mut().for_each(|x| x.update_bounding_sphere());
        }
    }

    fn _get_all_available_environment_names(&self) -> Vec<String> {
        let fp = get_path_to_src() + "assets/mesh_environments/";
        return get_all_files_in_directory(fp);
    }

    fn _get_all_mesh_files_in_particular_environment_directory(&self, environment_name: &String) -> Vec<String> {
        let fp = get_path_to_src() + "assets/mesh_environments/" + environment_name.as_str();
        return get_all_files_in_directory(fp);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn update_object_transform_by_idx(&mut self, idx: usize, new_transform: &ImplicitDualQuaternion) -> Result<(), String> {
        if idx > self.environment_obbs.len() {
            return Err(format!("idx {:?} is too high for number of environment objects ({:?})", idx, self.environment_obbs.len()));
        }

        self.transforms[idx] = new_transform.clone();
        let l = self.environment_obbs[idx].len();
        for i in 0..l {
            self.environment_obbs[idx][i].set_curr_pose(new_transform);
            self.environment_obbs[idx][i].update_all_bounding_volumes();
        }

        Ok(())
    }

    pub fn update_object_transform_by_name(&mut self, name: &str, new_transform: &ImplicitDualQuaternion) -> Result<(), String> {
        let idx = self._get_object_idx_from_object_name(name);
        if idx.is_none() {
            return Err(format!("object name {} was not found in collision environment", name));
        }
        return self.update_object_transform_by_idx(idx.unwrap(), new_transform);
    }

    fn _get_object_idx_from_object_name(&self, name: &str) -> Option<usize> {
        let l = self.object_names.len();
        for i in 0..l {
            if self.object_names[i].as_str() == name {
                return Some(i);
            }
        }
        return None;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn delete_object_by_idx(&mut self, idx: usize) -> Result<(), String> {
        if idx > self.environment_obbs.len() {
            return Err(format!("idx {:?} is too high for number of environment objects ({:?})", idx, self.environment_obbs.len()));
        }

        self.object_names.remove(idx);
        self.transforms.remove(idx);
        self.original_file_directories.remove(idx);
        self.environment_obbs.remove(idx);
        self.trimesh_engines.remove(idx);

        Ok(())
    }

    pub fn delete_object_by_name(&mut self, name: &str) -> Result<(), String> {
        let idx = self._get_object_idx_from_object_name(name);
        if idx.is_none() {
            return Err(format!("object name {} was not found in collision environment", name));
        }
        return self.delete_object_by_idx(idx.unwrap());
    }

    pub fn duplicate_object_by_idx(&mut self, idx: usize) -> Result<(), String> {
        if idx > self.environment_obbs.len() {
            return Err(format!("idx {:?} is too high for number of environment objects ({:?})", idx, self.environment_obbs.len()));
        }

        self.object_names.push(self.object_names[idx].clone());
        self.transforms.push(self.transforms[idx].clone());
        self.original_file_directories.push(self.original_file_directories[idx].clone());
        self.environment_obbs.push(self.environment_obbs[idx].clone());
        self.trimesh_engines.push(self.trimesh_engines[idx].clone());

        Ok(())
    }

    pub fn duplicate_object_by_name(&mut self, name: &str) -> Result<(), String> {
        let idx = self._get_object_idx_from_object_name(name);
        if idx.is_none() {
            return Err(format!("object name {} was not found in collision environment", name));
        }
        return self.duplicate_object_by_idx(idx.unwrap());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn absorb(&mut self, other: &CollisionEnvironment) {
        let l = other.environment_obbs.len();

        for i in 0..l {
            self.environment_obbs.push(other.environment_obbs[i].clone());
            self.object_names.push(other.object_names[i].clone());
            self.trimesh_engines.push(other.trimesh_engines[i].clone());
            self.transforms.push(other.transforms[i].clone());
            self.original_file_directories.push(other.original_file_directories[i].clone());
        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_num_objects(&self) -> usize {
        return self.object_names.len();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        println!("{}{}environment name --> {} {}", style::Bold, color::Fg(color::LightMagenta), style::Reset, self.environment_name);
        let l = self.object_names.len();
        for i in 0..l {
            println!("{}   object {} --> {}", color::Fg(color::Blue), i, style::Reset);
            println!("          name: {:?}", self.object_names[i]);
            println!("          num components: {:?}", self.environment_obbs[i].len());
            println!("          transform: {:?}", self.transforms[i]);
            println!("          original directory: {:?}", self.original_file_directories[i]);
        }
        println!();
    }
}
impl Clone for CollisionEnvironment {
    fn clone(&self) -> Self {
        return self.spawn();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////