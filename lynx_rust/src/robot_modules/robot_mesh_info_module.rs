
/*
use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use crate::utils::utils_files_and_strings::prelude::*;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use crate::robot_modules::link::Link;
use nalgebra::{Rotation3, Vector3, Matrix3, Quaternion, UnitQuaternion};
use std::path::Path;
use collada::document::ColladaDocument;
use xml::Xml::*;
use xml::Xml;

#[derive(Debug, Clone)]
pub struct RobotMeshInfoModule {
    _robot_name: String,
    _has_collision_base_meshes: bool,
    _has_visual_base_meshes: bool,
    _has_collision_triangle_meshes: bool,
    _has_visual_triangle_meshes: bool,
    _link_mesh_visual_offsets: Vec<Option<ImplicitDualQuaternion>>, // mostly for DAE collada files, where the mesh may be shifted off center in the file, even though the origin point of the mesh is somewhere else.  E.g., the Panda DAE links.
    _link_mesh_visual_scalings: Vec<Option<Vector3<f64>>>,
    _link_mesh_urdf_visual_offsets: Vec<Option<ImplicitDualQuaternion>>
}

impl RobotMeshInfoModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule) -> Self {
        let _robot_name = robot_configuration_module.robot_model_module.robot_name.to_string();

        let mut out_self = Self { _robot_name, _has_collision_base_meshes: false, _has_visual_base_meshes: false,
            _has_collision_triangle_meshes: false, _has_visual_triangle_meshes: false,
            _link_mesh_visual_offsets: Vec::new(), _link_mesh_visual_scalings: Vec::new(),
            _link_mesh_urdf_visual_offsets: Vec::new()  };

        out_self.check_and_reset_all_mesh_infos();
        out_self._set_link_mesh_visual_offsets_and_scalings(robot_configuration_module);
        out_self._set_link_urdf_visual_offsets(robot_configuration_module);

        return out_self;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn check_and_reset_all_mesh_infos(&mut self) {
        let path_to_robot_dir = get_path_to_particular_robot_directory(self._robot_name.clone());

        let path = path_to_robot_dir.clone() + "/base_meshes/collision";
        self._has_collision_base_meshes = RobotMeshInfoModule::_check_path(&path);

        let path = path_to_robot_dir.clone() + "/base_meshes/visual";
        self._has_visual_base_meshes = RobotMeshInfoModule::_check_path(&path);

        let path = path_to_robot_dir.clone() + "/autogenerated_metadata/link_triangle_meshes_collision";
        self._has_collision_triangle_meshes = RobotMeshInfoModule::_check_path(&path);

        let path = path_to_robot_dir.clone() + "/autogenerated_metadata/link_triangle_meshes_visual";
        self._has_visual_triangle_meshes = RobotMeshInfoModule::_check_path(&path);
    }

    fn _check_path(path: &String) -> bool {
        let path_exists = check_if_path_exists(path.to_string());
        if path_exists {
            let num_files = get_all_files_in_directory(path.to_string()).len();
            if num_files > 0 {
                return true;
            } else {
                delete_directory_all(path.to_string());
                return false;
            }
        } else {
            return false;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _set_link_mesh_visual_offsets_and_scalings(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        let links = &robot_configuration_module.robot_model_module.links;
        let l = links.len();

        for i in 0..l {
            if links[i].urdf_link.visual_file_name_without_extension.is_none() {
                self._link_mesh_visual_offsets.push(Some(ImplicitDualQuaternion::new_identity()));
                self._link_mesh_visual_scalings.push(Some(Vector3::new(1.,1.,1.)));
            }
            else {
                let filename = links[i].urdf_link.visual[0].filename.as_ref().unwrap();
                let filename_extension = get_filename_extension(filename.to_string());
                match filename_extension.as_str() {
                    "dae" => {
                        let res = Self::_get_single_link_mesh_visual_offset_and_scaling_from_dae(robot_configuration_module.robot_model_module.robot_name.clone(), filename.clone());
                        self._link_mesh_visual_offsets.push( Some(res.0) );
                        self._link_mesh_visual_scalings.push(Some(res.1) );
                    },
                    "DAE" => {
                        let res = Self::_get_single_link_mesh_visual_offset_and_scaling_from_dae(robot_configuration_module.robot_model_module.robot_name.clone(), filename.clone());
                        self._link_mesh_visual_offsets.push( Some(res.0) );
                        self._link_mesh_visual_scalings.push(Some(res.1) );
                    },
                    _ => {
                        self._link_mesh_visual_offsets.push( Some(ImplicitDualQuaternion::new_identity()) );
                        self._link_mesh_visual_scalings.push( Some(Vector3::new(1.,1.,1.) ));
                    }
                }
            }
        }
    }

    fn _get_single_link_mesh_visual_offset_and_scaling_from_dae(robot_name: String, filename: String) -> (ImplicitDualQuaternion, Vector3<f64>) {
        let fp = get_path_to_particular_robot_directory(robot_name) + "/base_meshes/visual/" + filename.as_str();
        let path = Path::new(&fp);
        let doc = ColladaDocument::from_path(&path).expect(format!("Collada file {:?} could not be parsed in robot_mesh_info_module", fp).as_str());

        let mut object_id = "".to_string();

        let o = doc.get_obj_set().unwrap().clone();
        if o.objects.len() > 0 {
            object_id = o.objects[0].id.clone();
        }

        let mut matrix_string = "".to_string();
        let mut translate_string = "".to_string();
        let mut rotateZ_string = "".to_string();
        let mut rotateY_string = "".to_string();
        let mut rotateX_string = "".to_string();
        let mut scale_string = "".to_string();

        let mut string_set = false;

        let v1 = &doc.root_element.children;
        for a in v1 {
            if string_set { break; }
            match a {
                ElementNode(e) => {
                    if e.name == "library_visual_scenes" {
                        let v2 = &e.children;
                        for a in v2 {
                            if string_set { break; }
                            match a {
                                ElementNode(e) => {
                                    if e.name == "visual_scene" {
                                        let v3 = &e.children;
                                        for a in v3 {
                                            if string_set { break; }
                                            match a {
                                                ElementNode(e) => {
                                                    if e.name == "node" {
                                                        let v4 = &e.children;

                                                        let mut url_local = "".to_string();
                                                        let mut matrix_string_local = "".to_string();

                                                        for a in v4 {
                                                            if string_set { break; }
                                                            match a {
                                                                ElementNode(e) => {
                                                                    if e.name == "instance_geometry" {
                                                                        let url = e.attributes.get(&("url".to_string(), None));
                                                                        url_local = url.unwrap().clone();
                                                                    } else if e.name == "matrix" {
                                                                        let v5 = &e.children;
                                                                        for a in v5 {
                                                                            match a {
                                                                                CharacterNode(s) => {
                                                                                    matrix_string_local = s.clone();
                                                                                },
                                                                                _ => { }
                                                                            }
                                                                        } // end of v5 loop
                                                                    } else if e.name == "translate" {
                                                                        let v5 = &e.children;
                                                                        for a in v5 {
                                                                            match a {
                                                                                CharacterNode(s) => {
                                                                                    translate_string = s.clone();
                                                                                },
                                                                                _ => { }
                                                                            }
                                                                        } // end of v5 loop
                                                                    } else if e.name == "scale" {
                                                                        let v5 = &e.children;
                                                                        for a in v5 {
                                                                            match a {
                                                                                CharacterNode(s) => {
                                                                                    scale_string = s.clone();
                                                                                },
                                                                                _ => { }
                                                                            }
                                                                        } // end of v5 loop
                                                                    } else if e.name == "rotate" {
                                                                        let sid_ = e.attributes.get(&("sid".to_string(), None));
                                                                        let v5 = &e.children;
                                                                        for a in v5 {
                                                                            match a {
                                                                                CharacterNode(s) => {
                                                                                    match sid_ {
                                                                                        None => { }
                                                                                        Some(sid) => {
                                                                                            if sid == "rotateX" { rotateX_string = s.clone() }
                                                                                            else if sid == "rotateY" { rotateY_string = s.clone() }
                                                                                            else if sid == "rotateZ" { rotateZ_string = s.clone() }
                                                                                        }
                                                                                    }
                                                                                },
                                                                                _ => { }
                                                                            }
                                                                        }
                                                                    }
                                                                },
                                                                _ => { }
                                                            } // end of v4 match
                                                        } // end of v4 for loop

                                                        // make decision here
                                                        if url_local.contains(&object_id) && matrix_string_local.len() > 0 {
                                                            matrix_string = matrix_string_local.clone();
                                                            string_set = true;
                                                        }

                                                        if url_local.contains(&object_id) && (scale_string.len() > 0 || translate_string.len() > 0 || rotateX_string.len() > 0 || rotateY_string.len() > 0 || rotateZ_string.len() > 0 ) {
                                                            string_set = true;
                                                        }

                                                    }
                                                } // end of v3 element node
                                                _ => { }
                                            } // end of v3 match
                                        } // end of v3 for loop
                                    }
                                } // end of v2 element node
                                _ => { }
                            } // end of v2 match
                        } // end of v2 loop
                    }
                } // end of v1 element node
                _ => { }
            } // end of v1 match
        } // end of v1 for loop

        if !(matrix_string == "") {
            let split: Vec<&str> = matrix_string.split(" ").collect();

            let mut floats = Vec::new();

            let l = split.len();
            for i in 0..l {
                let f = split[i].parse::<f64>();
                if f.is_ok() {
                    floats.push(f.ok().unwrap());
                }
            }

            if !(floats.len() == 16) {
                return (ImplicitDualQuaternion::new_identity(), Vector3::new(1., 1., 1.));
            } else {
                let mut mat = Matrix3::new(0., 0., 0., 0., 0., 0., 0., 0., 0.);
                let mut vec = Vector3::new(0., 0., 0.);
                mat[(0, 0)] = floats[0];
                mat[(0, 1)] = floats[1];
                mat[(0, 2)] = floats[2];
                vec[0] = floats[3];

                mat[(1, 0)] = floats[4];
                mat[(1, 1)] = floats[5];
                mat[(1, 2)] = floats[6];
                vec[1] = floats[7];

                mat[(2, 0)] = floats[8];
                mat[(2, 1)] = floats[9];
                mat[(2, 2)] = floats[10];
                vec[2] = floats[11];

                let scaling1 = Vector3::new(mat[(0, 0)], mat[(1, 0)], mat[(2, 0)]).norm();
                let scaling2 = Vector3::new(mat[(0, 1)], mat[(1, 1)], mat[(2, 1)]).norm();
                let scaling3 = Vector3::new(mat[(0, 2)], mat[(1, 2)], mat[(2, 2)]).norm();
                let scaling = Vector3::new(scaling1, scaling2, scaling3);

                let quat: UnitQuaternion<f64> = UnitQuaternion::from_matrix(&mat);
                let idq = ImplicitDualQuaternion::new(quat, vec);

                return (idq, scaling);
            }
        } else {
            let split: Vec<&str> = translate_string.split(" ").collect();
            let mut translate_floats = Vec::new();
            let l = split.len();
            for i in 0..l {
                let f = split[i].parse::<f64>();
                if f.is_ok() {
                    translate_floats.push(f.ok().unwrap());
                }
            }

            let split: Vec<&str> = rotateX_string.split(" ").collect();
            let mut rotateX_floats = Vec::new();
            let l = split.len();
            for i in 0..l {
                let f = split[i].parse::<f64>();
                if f.is_ok() {
                    rotateX_floats.push(f.ok().unwrap());
                }
            }

            let split: Vec<&str> = rotateY_string.split(" ").collect();
            let mut rotateY_floats = Vec::new();
            let l = split.len();
            for i in 0..l {
                let f = split[i].parse::<f64>();
                if f.is_ok() {
                    rotateY_floats.push(f.ok().unwrap());
                }
            }

            let split: Vec<&str> = rotateZ_string.split(" ").collect();
            let mut rotateZ_floats = Vec::new();
            let l = split.len();
            for i in 0..l {
                let f = split[i].parse::<f64>();
                if f.is_ok() {
                    rotateZ_floats.push(f.ok().unwrap());
                }
            }

            let split: Vec<&str> = scale_string.split(" ").collect();
            let mut scale_floats = Vec::new();
            let l = split.len();
            for i in 0..l {
                let f = split[i].parse::<f64>();
                if f.is_ok() {
                    scale_floats.push(f.ok().unwrap());
                }
            }

            let mut mat = Matrix3::new(0., 0., 0., 0., 0., 0., 0., 0., 0.);
            let mut vec = Vector3::new(0., 0., 0.);

            if rotateX_floats.len() > 2 {
                mat[(0,0)] = rotateX_floats[0];
                mat[(1,0)] = rotateX_floats[1];
                mat[(2,0)] = rotateX_floats[2];
            }
            if rotateY_floats.len() > 2 {
                mat[(0,1)] = rotateY_floats[0];
                mat[(1,1)] = rotateY_floats[1];
                mat[(2,1)] = rotateY_floats[2];
            }
            if rotateZ_floats.len() > 2 {
                mat[(0,2)] = rotateZ_floats[0];
                mat[(1,2)] = rotateZ_floats[1];
                mat[(2,2)] = rotateZ_floats[2];
            }
            if translate_floats.len() > 2 {
                vec[0] = translate_floats[0];
                vec[1] = translate_floats[1];
                vec[2] = translate_floats[2];
            }

            let mut scaling = Vector3::new(1.,1.,1.);
            if scale_floats.len() > 2 {
                scaling[0] = scale_floats[0];
                scaling[1] = scale_floats[1];
                scaling[2] = scale_floats[2];
            }

            let quat: UnitQuaternion<f64> = UnitQuaternion::from_matrix(&mat);
            let idq = ImplicitDualQuaternion::new(quat, vec);

            return (idq, scaling);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _set_link_urdf_visual_offsets(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        let links = &robot_configuration_module.robot_model_module.links;
        for l in links {
            if l.urdf_link.includes_visual_info {
                let v = &l.urdf_link.visual[0];
                let origin_xyz = &v.origin_xyz;
                let origin_rpy = &v.origin_rpy;
                let quat = UnitQuaternion::from_euler_angles(origin_rpy[0], origin_rpy[1], origin_rpy[2]);
                let idq = ImplicitDualQuaternion::new(quat, origin_xyz.into_owned());
                self._link_mesh_urdf_visual_offsets.push(Some(idq));
            } else {
                self._link_mesh_urdf_visual_offsets.push(Some(ImplicitDualQuaternion::new_identity()));
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn has_collision_base_meshes(&self) -> bool {
        return self._has_collision_base_meshes;
    }

    pub fn has_visual_base_meshes(&self) -> bool {
        return self._has_visual_base_meshes;
    }

    pub fn has_collision_triangle_meshes(&self) -> bool {
        return self._has_collision_triangle_meshes;
    }

    pub fn has_visual_triangle_meshes(&self) -> bool {
        return self._has_visual_triangle_meshes;
    }

    pub fn get_link_mesh_visual_offsets(&self) -> &Vec<Option<ImplicitDualQuaternion>> {
        return &self._link_mesh_visual_offsets;
    }

    pub fn get_link_mesh_visual_scalings(&self) -> &Vec<Option<Vector3<f64>>> {
        return &self._link_mesh_visual_scalings
    }

    pub fn get_link_urdf_visual_offsets(&self) -> &Vec<Option<ImplicitDualQuaternion>> {
        return &self._link_mesh_urdf_visual_offsets;
    }
}
*/