use crate::utils::utils_collisions::{oriented_bounding_box::OBB, collision_object::CollisionObject, triangle_mesh_engine::TriMeshEngine};
use crate::utils::utils_files_and_strings::{file_utils::*, string_utils::*};
use termion::{color, style};
use ncollide3d::math::Point;

#[derive(Debug)]
pub struct CollisionEnvironment {
    pub environment_name: String,
    pub environment_obbs: Vec<Vec<CollisionObject>>, // each sub-vector represents parts of individual "objects"
    pub object_names: Vec<String>,
    _trimesh_engines: Vec<Vec<TriMeshEngine>>
}

impl CollisionEnvironment {
    pub fn new(environment_name: &str) -> Result<Self, String> {
        let environment_obbs = Vec::new();
        let object_names = Vec::new();
        let _trimesh_engines = Vec::new();

        let mut out_self = Self { environment_name: environment_name.to_string(), environment_obbs, object_names, _trimesh_engines };
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
        let l1 = self._trimesh_engines.len();
        for i in 0..l1 {
            let l2 = self._trimesh_engines[i].len();
            _trimesh_engines.push(Vec::new());
            for j in 0..l2 {
                _trimesh_engines[i].push( self._trimesh_engines[i][j].clone() );
            }
        }

        return Self {environment_name: self.environment_name.clone(), environment_obbs, object_names, _trimesh_engines};
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _load_environment_obbs(&mut self, environment_name: &String) -> Result<(), String> {
        let all_available_environments = self._get_all_available_environment_names();
        if !all_available_environments.contains(environment_name) {
            println!("{}{}ERROR: environment {} not found as valid environment. {}", color::Fg(color::Red), style::Bold, environment_name, style::Reset);
            return Err(format!("environment {} not found as valid environment.", environment_name));
        }

        let mesh_filenames = self._get_all_mesh_files_in_particular_environment_directory(environment_name);

        let l = mesh_filenames.len();
        for i in 0..l {
            if !(mesh_filenames[i].clone() == "metadata.json".to_string()) {
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

            if directory_updated_since_metadata_was_created.is_some() && !directory_updated_since_metadata_was_created.unwrap() {
                self._load_environment_from_metadata(environment_name.clone());
                return Ok(());
            }
        }

        let mut count = 0 as usize;
        if !loaded_from_metadata {
            let l = mesh_filenames.len();
            for i in 0..l {
                if mesh_filenames[i] == "metadata.json".to_string() { continue; }

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
                self._trimesh_engines.push(ts);
                count += 1;
            }
            self._save_environment_obbs_metadata(environment_name.clone());
        }

        Ok(())
    }

    fn _load_environment_from_metadata(&mut self, environment_name: String) {
        let metadata_fp = get_path_to_src() + "assets/mesh_environments/" + environment_name.as_str() + "/metadata.json";

        let json_string = read_file_contents(metadata_fp);
        if json_string.is_none() { return; }

        let metadata: (Vec<Vec<Vec<Point<f64>>>>,Vec<Vec<Vec<Point<usize>>>>) = serde_json::from_str(&json_string.unwrap()).unwrap();

        let (vertices, indices) = metadata.clone();

        let l = vertices.len();
        for i in 0..l {
            self.environment_obbs.push(Vec::new());
            self._trimesh_engines.push(Vec::new());

            let l2 = vertices[i].len();
            for j in 0..l2 {
                let mut t = TriMeshEngine::new( vertices[i][j].clone(), indices[i][j].clone() );
                self.environment_obbs[i].push(CollisionObject::new_cuboid_from_trimesh_engine(&t, Some(self.object_names[i].clone() + format!("_{:?}", j).as_str())));
                self._trimesh_engines[i].push(t);
            }
        }

    }

    fn _save_environment_obbs_metadata(&mut self, environment_name: String) {
        let mut vertices: Vec<Vec<Vec<Point<f64>>>> = Vec::new();
        let mut indices: Vec<Vec<Vec<Point<usize>>>> = Vec::new();

        for i in 0..self._trimesh_engines.len() {
            vertices.push(Vec::new());
            indices.push(Vec::new());
            for j in 0..self._trimesh_engines[i].len() {
                vertices[i].push( self._trimesh_engines[i][j].vertices.clone() );
                indices[i].push( self._trimesh_engines[i][j].indices.clone() );
            }
        }

        let fp = get_path_to_src() + "assets/mesh_environments/" + environment_name.as_str();
        let serialized = serde_json::to_string( &(vertices, indices) ).ok().unwrap();
        write_string_to_file(fp, "metadata.json".to_string(), serialized, true );
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
}
impl Clone for CollisionEnvironment {
    fn clone(&self) -> Self {
        return self.spawn();
    }
}