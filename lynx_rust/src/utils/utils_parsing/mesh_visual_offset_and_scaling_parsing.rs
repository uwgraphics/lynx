use crate::utils::utils_files_and_strings::prelude::*;
use crate::utils::utils_se3::prelude::ImplicitDualQuaternion;
use nalgebra::{Vector3, UnitQuaternion, Matrix3};
use std::path::Path;
use collada::document::ColladaDocument;
use xml::Xml::*;
use xml::Xml;

#[derive(Debug, Clone)]
pub enum MeshOffetType {
    visual,
    collision
}

pub fn get_single_link_mesh_visual_offset_and_scaling(robot_name: String, filename: String, mesh_offset_type: MeshOffetType) -> (ImplicitDualQuaternion, Vector3<f64>) {
    let ext = get_filename_extension(filename.clone());
    if ext == "DAE" || ext == "dae" {
        return _get_single_link_mesh_visual_offset_and_scaling_from_dae(robot_name, filename.clone(), mesh_offset_type);
    } else {
        return (ImplicitDualQuaternion::new_identity(), Vector3::new(1.,1.,1.));
    }
}

fn _get_single_link_mesh_visual_offset_and_scaling_from_dae(robot_name: String, filename: String, mesh_offset_type: MeshOffetType) -> (ImplicitDualQuaternion, Vector3<f64>) {
    let fp = match mesh_offset_type {
        MeshOffetType::visual => { get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/visual/" + filename.as_str() }
        MeshOffetType::collision => { get_path_to_particular_robot_directory(robot_name.clone()) + "/base_meshes/collision/" + filename.as_str() }
    };
    let path = Path::new(&fp);
    let doc = ColladaDocument::from_path(&path).expect(format!("Collada file {:?} could not be parsed.", fp).as_str());

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

