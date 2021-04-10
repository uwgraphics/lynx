use nalgebra::{Vector3, U3, DVector};
use ncollide3d::math::Point;
use std::fs::File;
use std::io::{BufReader, Write};
use std::path::Path;
use tobj;
use termion::{color, style};
use std::fs::OpenOptions;
use stl_io;
use collada::document::ColladaDocument;
use collada::{*};
use std::fs;
use ncollide3d::procedural;
use nalgebra::geometry::Point3;
use ncollide3d::transformation;
// use meshopt;
use crate::utils::utils_pointclouds::pointcloud_data_utils::*;
use crate::utils::utils_math::geometry_utils::*;
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_math::geometry_utils::*;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;

/*
This struct parses .dae, .obj, and .stl files and converts them into tri meshes
*/
#[derive(Clone, Debug)]
pub struct TriMeshEngine {
    pub vertices: Vec<Point<f64>>,
    pub indices: Vec<Point<usize>>
}

impl TriMeshEngine {
    pub fn new(vertices: Vec<Point<f64>>, indices: Vec<Point<usize>>) -> Self {
        return Self {vertices, indices};
    }

    pub fn new_from_path(fp: String) -> Result<Self, String> {
        let ext = Path::new(fp.as_str()).extension();

        let err = ext.is_none();
        if err {
            println!("{}{}WARNING: {:?} invalid file type in TriMeshParser{}", color::Fg(color::Yellow), style::Bold, fp, style::Reset);
            return Err(format!("{:?} invalid file type in TriMeshParser", fp));
        }

        let ext_unwrap = ext.unwrap();
        if ext_unwrap == "obj" {
            return Self::new_from_obj_path(fp);
        } else if ext_unwrap == "dae" || ext_unwrap == "DAE" {
            return Self::new_from_dae_path(fp);
        } else if (ext_unwrap == "stl") || (ext_unwrap == "STL") {
            return Self::new_from_stl_path(fp);
        } else {
            println!("{}{}WARNING: {:?} invalid file type in TriMeshParser{}", color::Fg(color::Yellow), style::Bold, fp, style::Reset);
            return Err(format!("{:?} invalid file type in TriMeshParser", fp));
        }
    }

    pub fn new_from_dae_path(fp: String) -> Result<Self, String> {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        let path = Path::new(&fp);
        let doc = ColladaDocument::from_path(&path);

        let err = doc.is_err();
        if err {
            println!("{}{}WARNING: invalid .dae file in TriMeshParser{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            return Err("invalid .dae file in TriMeshParser".to_string());
        }

        let doc_unwrap = doc.ok().unwrap();

        let obj_set = doc_unwrap.get_obj_set().unwrap();

        let mut _vertices: Vec<Vec<Point<f64>>> = Vec::new();
        let mut _indices: Vec<Vec<Point<usize>>> = Vec::new();

        // first dump all vertices
        let num_objects = obj_set.objects.len();
        for i in 0..num_objects {
            _vertices.push( Vec::new() );
            _indices.push(Vec::new() );
            let num_verts = obj_set.objects[i].vertices.len();
            for j in 0..num_verts {
                let v = obj_set.objects[i].vertices[j].clone();
                _vertices[i].push( Point::new( v.x as f64, v.y as f64, v.z as f64)  )
            }
        }

        let num_objects = obj_set.objects.len();
        for i in 0..num_objects {
            let num_geom = obj_set.objects[i].geometry.len();
            for j in 0..num_geom {
                let num_meshes = obj_set.objects[i].geometry[j].mesh.len();
                for k in 0..num_meshes {

                    let curr_idx_len = _indices[i].len();
                    let pe = obj_set.objects[i].geometry[j].mesh[k].clone();

                    match pe {
                        PrimitiveElement::Polylist( p ) => {
                            let num_shapes = p.shapes.len();
                            for l in 0..num_shapes {
                                let s = p.shapes[l];
                                match s {
                                    Shape::Triangle(a,b,c) => {
                                        let point = Point::new( a.0 as usize, b.0 as usize, c.0 as usize );
                                        _indices[i].push( point );
                                    },
                                    _ => { }
                                }
                            }
                        },
                        PrimitiveElement::Triangles(t) => {
                            let num_verts = t.vertices.len();
                            for l in 0..num_verts {
                                let point = Point::new( t.vertices[l].0 as usize, t.vertices[l].1 as usize, t.vertices[l].2 as usize);
                                _indices[i].push(point);
                            }
                        }
                    }

                }
            }
        }


        for i in 0..num_objects {
            let curr_len = vertices.len();

            let l = _vertices[i].len();
            for j in 0..l {
                vertices.push(_vertices[i][j].clone());
            }

            let l = _indices[i].len();
            for j in 0..l {
                let new_point = Point::new( _indices[i][j][0] + curr_len, _indices[i][j][1] + curr_len, _indices[i][j][2] + curr_len );
                indices.push(new_point);
            }

        }

        /*
        let idx = 0;
        println!("{:?}", _vertices[idx].len());

        let mut max = 0;
        let l = _indices[idx].len();
        for i in 0..l {
            if _indices[idx][i][0] > max { max = _indices[idx][i][0]; }
            if _indices[idx][i][1] > max { max = _indices[idx][i][1]; }
            if _indices[idx][i][2] > max { max = _indices[idx][i][2]; }
        }
        println!("{:?}", max);
        */

        /*
        println!("{:?}", vertices.len());

        let mut max = 0;
        let l = indices.len();
        for i in 0..l {
            if indices[i][0] > max { max = indices[i][0]; }
            if indices[i][1] > max { max = indices[i][1]; }
            if indices[i][2] > max { max = indices[i][2]; }
        }
        println!("{:?}", max);
        */

        return Ok(Self {vertices, indices})
    }

    pub fn new_from_obj_path(fp: String) -> Result<Self, String> {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();


        let path = Path::new(fp.as_str());
        let obj = tobj::load_obj( &path, true );

        // println!("{:?}", obj.unwrap().0[0].mesh.positions);

        let err = obj.is_err();
        if err {
            println!("{}{}WARNING: invalid .obj file in TriMeshParser{}", color::Fg(color::Yellow), style::Bold, style::Reset);
            return Err("invalid .obj file in TriMeshParser".to_string());
        }

        let obj_unwrap = obj.unwrap();

        let l = obj_unwrap.0.len();

        for i in 0..l {
            let _vertices = obj_unwrap.0[i].mesh.positions.clone();
            let _indices = obj_unwrap.0[i].mesh.indices.clone();

            let vl = _vertices.len();
            let il = _indices.len();

            let curr_length = vertices.len();

            for j in 0..vl/3 {
                let x = _vertices[3*j] as f64;
                let y = _vertices[3*j+1] as f64;
                let z = _vertices[3*j+2] as f64;
                vertices.push( Point::new(x,y,z) );
            }

            for j in 0..il/3 {
                let i1 = _indices[3*j] as usize + curr_length;
                let i2 = _indices[3*j+1] as usize + curr_length;
                let i3 = _indices[3*j+2] as usize + curr_length;
                indices.push( Point::new(i1, i2, i3)  );
            }

        }

        return Ok(Self {vertices, indices})
    }

    pub fn new_from_stl_path(fp: String) -> Result<Self, String> {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        let mut file = OpenOptions::new().read(true).open(fp.clone()).unwrap();
        let stl = stl_io::read_stl(&mut file).unwrap();

        for i in 0..stl.vertices.len() {
            let v = stl.vertices[i].clone();
            vertices.push( Point::new(v[0] as f64, v[1] as f64, v[2] as f64) );
        }

        for i in 0..stl.faces.len() {
            let f = stl.faces[i].clone();
            indices.push(  Point::new( f.vertices[0], f.vertices[1], f.vertices[2] )  );
        }

        if vertices.len() == 0 {
            return Err(format!("zero vertices parsed from {:?} .stl file in TriMeshEngine parser", fp));
        }

        return Ok(Self {vertices, indices})
    }

    pub fn new_from_procedural_trimesh(procedural_trimesh: &procedural::TriMesh<f64>) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        vertices = procedural_trimesh.coords.clone();
        let _indices = procedural_trimesh.indices.clone().unwrap_unified();

        let l = _indices.len();
        for i in 0..l {
            let idx = Point::new( _indices[i][0] as usize, _indices[i][1] as usize, _indices[i][2] as usize );
            indices.push(idx);
        }

        return Self {vertices, indices}
    }

    pub fn transform_vertices(&mut self, transform: &ImplicitDualQuaternion) {
        let l = self.vertices.len();
        for i in 0..l {
            let v = Vector3::new( self.vertices[i][0],  self.vertices[i][1],  self.vertices[i][2] );
            let res = transform.multiply_by_vector3( &v );
            self.vertices[i] = Point3::new( res[0], res[1], res[2] );
        }
    }

    pub fn output_to_stl(&self, out_fp: String) {
        let mut mesh = Vec::new();
        let l = self.indices.len();
        for i in 0..l {
            let normal = stl_io::Normal::new([0.,0.,0.]);
            let _v1 = self.vertices[ self.indices[i][0] ].clone();
            let _v2 = self.vertices[ self.indices[i][1] ].clone();
            let _v3 = self.vertices[ self.indices[i][2] ].clone();

            let v1 = stl_io::Vertex::new( [_v1[0] as f32, _v1[1] as f32, _v1[2] as f32]  );
            let v2 = stl_io::Vertex::new( [_v2[0] as f32, _v2[1] as f32, _v2[2] as f32]  );
            let v3 = stl_io::Vertex::new( [_v3[0] as f32, _v3[1] as f32, _v3[2] as f32]  );

            let triangle = stl_io::Triangle{ normal: normal, vertices: [v1, v2, v3] };
            mesh.push(triangle);
        }

        let mut exists = Path::new(&out_fp).exists();
        if exists {
            fs::remove_file(  Path::new(&out_fp) );
        }
        let mut out_file =  OpenOptions::new().write(true).create(true).open(out_fp.clone()).unwrap();
        stl_io::write_stl(&mut out_file, mesh.iter()).unwrap();
    }

    pub fn output_to_obj(&self, out_fp: String) {
        let mut exists = Path::new(&out_fp).exists();
        if exists {
            fs::remove_file(  Path::new(&out_fp) );
        }
        let mut out_file =  OpenOptions::new().write(true).create(true).open(out_fp.clone()).unwrap();

        let l = self.vertices.len();
        for i in 0..l {
            let out_str = format!("v {} {} {}\n", self.vertices[i][0], self.vertices[i][1], self.vertices[i][2]);
            out_file.write(out_str.as_bytes());
        }

        let l = self.indices.len();
        for i in 0..l {
            let out_str = format!("f {} {} {}\n", self.indices[i][0]+1, self.indices[i][1]+1, self.indices[i][2]+1);
            out_file.write(out_str.as_bytes());
        }
    }

    pub fn to_procedural_trimesh(&self) -> procedural::TriMesh<f64> {
        let mut coords = self.vertices.clone();
        let mut index_buffer: Vec<Point3<u32>> = Vec::new();
        let l = self.indices.len();
        for i in 0..l {
            index_buffer.push( Point3::new(  self.indices[i][0] as u32, self.indices[i][1] as u32, self.indices[i][2] as u32   )  );
        }
        let mut index_buffer_out = procedural::IndexBuffer::Unified(index_buffer);

        let mut out_trimesh = procedural::TriMesh::new( coords, None, None, Option::Some(index_buffer_out) );
        out_trimesh.recompute_normals();
        out_trimesh
    }

    pub fn split_into_convex_components(&self, error: f64, min_num_components: usize) -> Vec<TriMeshEngine> {
        let mut out_vec = Vec::new();

        let h = transformation::hacd( self.to_procedural_trimesh(), error, min_num_components );
        let l = h.0.len();

        for i in 0..l {
            out_vec.push(  Self::new_from_procedural_trimesh( &h.0[i] )  );
        }

        return out_vec;
    }

    pub fn compute_convex_hull(&self) -> TriMeshEngine {
        let ch = transformation::convex_hull(&self.vertices);
        return TriMeshEngine::new_from_procedural_trimesh(&ch);
    }

    pub fn compute_center(&self) -> Vector3<f64> {
        let mut center = Vector3::zeros();
        let mut count = 0.0;

        let mut areas = Vec::new();
        let mut triangle_centers = Vec::new();
        let mut max_area = -std::f64::INFINITY;

        let l = self.indices.len();
        for i in 0..l {
            let mut ti = self.indices[i].clone();
            let verts = ncollide_points_to_vector3_points( &vec![ self.vertices[ti[0]].clone(), self.vertices[ti[1]].clone(), self.vertices[ti[2]].clone()  ] );
            let side_length1 = (&verts[0] - &verts[1]).norm();
            let side_length2 = (&verts[0] - &verts[2]).norm();
            let side_length3 = (&verts[1] - &verts[2]).norm();
            let area = area_of_triangle_from_sidelengths(side_length1, side_length2, side_length3);
            areas.push(area);
            if area > max_area { max_area = area; }
            let mut triangle_center = (&verts[0] + &verts[1] + &verts[2]) / 3.0;
            triangle_centers.push(triangle_center);
        }

        let mut areas_normalized = DVector::from_element(l, 0.0);
        for i in 0..l {
            areas_normalized[i] = ( areas[i] / max_area );
        }

        let mut areas_l1_norm = 0.0;
        for i in 0..l {
            areas_l1_norm += areas_normalized[i].abs();
        }

        for i in 0..l {
            areas_normalized[i] /= areas_l1_norm;
        }

        for i in 0..l {
            center += areas_normalized[i] * &triangle_centers[i];
        }

        center
    }

    /*
    pub fn simplify_mesh(&self) -> Self {
        // TODO: fix this...not working right now
        let mut vertex_buffer = Vec::new();
        let mut index_buffer = Vec::new();

        let l = self.vertices.len();
        for i in 0..l {
            vertex_buffer.push( self.vertices[i][0] );
            vertex_buffer.push( self.vertices[i][1] );
            vertex_buffer.push( self.vertices[i][2] );
        }

        let l = self.indices.len();
        for i in 0..l {
            index_buffer.push(self.indices[i][0] as u32);
            index_buffer.push(self.indices[i][1] as u32);
            index_buffer.push(self.indices[i][2] as u32);
        }

        // println!("{:?}", self.vertices.len());

        let bytes = meshopt::utilities::typed_to_bytes(&vertex_buffer);
        let vda = meshopt::utilities::VertexDataAdapter::new(bytes, 24, 0);
        let vda_unwrap = vda.ok().unwrap();
        // println!("{:?}", vda_unwrap.vertex_count);
        let res = meshopt::simplify(&index_buffer.to_vec(), &vda_unwrap, 2000, 0.001);

        /*
        println!("{:?}", res.len());
        println!("{:?}", res);
        println!("{:?}", index_buffer[res[0] as usize]);
        println!("{:?}", index_buffer[res[1] as usize]);
        println!("{:?}", index_buffer[res[2] as usize]);
        */

        println!("{:?}", res);
        let mut new_indices = Vec::new();
        let mut count = 0;
        for i in 0..res.len() / 3 {
            new_indices.push( Point::new( res[3*i] as usize, res[3*i+1] as usize, res[3*i+2] as usize ) );
        }
        println!("{:?}", new_indices);

        Self { vertices: self.vertices.clone(), indices: new_indices }
    }
    */

    /*
    pub fn simplify_mesh(&self) -> TriMeshEngine {
        let mut out_vertices: Vec<Point<f64>> = self.vertices.clone();
        let mut out_indices: Vec<Point<usize>> = self.indices.clone();
        println!("{:?}", out_indices.len());

        let num_faces = out_indices.len();
        let sampler1 = ThreadRangeIntSampler::new(0, (num_faces - 1) as i32, 1);
        let sampler2 = ThreadRangeIntSampler::new(0, 5, 1);

        let mut remap_to_idxs = vec![usize::max_value(); num_faces];
        let mut remap_from_idxs: Vec<Vec<usize>> = vec![Vec::new(); num_faces];


        for i in 0..100000 {
            let mut delete_idx = 0 as usize;
            let mut replace_idx = 0 as usize;

            let face_idx = sampler1.sample()[0] as usize;
            let edge_idx = sampler2.sample()[0] as usize;

            if edge_idx == 0 {
                delete_idx = out_indices[face_idx][0];
                replace_idx = out_indices[face_idx][1];
            }
            if edge_idx == 1 {
                delete_idx = out_indices[face_idx][1];
                replace_idx = out_indices[face_idx][0];
            }
            if edge_idx == 2 {
                delete_idx = out_indices[face_idx][0];
                replace_idx = out_indices[face_idx][2];
            }
            if edge_idx == 3 {
                delete_idx = out_indices[face_idx][2];
                replace_idx = out_indices[face_idx][0];
            }
            if edge_idx == 4 {
                delete_idx = out_indices[face_idx][1];
                replace_idx = out_indices[face_idx][2];
            }
            if edge_idx == 5 {
                delete_idx = out_indices[face_idx][2];
                replace_idx = out_indices[face_idx][1];
            }

            if !remap_to_idxs[delete_idx] == usize::max_value() {continue;}

            remap_to_idxs[delete_idx] = replace_idx;
            remap_from_idxs[replace_idx].push(delete_idx);

            for d in &remap_from_idxs[delete_idx] {
                remap_to_idxs[*d] = replace_idx;
            }
        }

        for i in 0..num_faces {
            let remap0 = remap_to_idxs[self.indices[i][0]];
            let remap1 = remap_to_idxs[self.indices[i][1]];
            let remap2 = remap_to_idxs[self.indices[i][2]];

            if !(remap0 == usize::max_value()) { out_indices[i][0] = remap0; }
            if !(remap1 == usize::max_value()) { out_indices[i][1] = remap1; }
            if !(remap2 == usize::max_value()) { out_indices[i][2] = remap2; }
        }

        out_indices = out_indices.into_iter().filter(|x| !((x[0] == x[1]) || (x[0] == x[2]) || (x[1] == x[2]))).collect();
        println!("{:?}", out_indices.len());


        /*
        println!("{:?}", self.indices.len());

        for i in 0..2000 {
            let v1 = self.indices[i][0];
            let v2 = self.indices[i][1];

            let l = out_indices.len();
            for i in 0..l {
                if out_indices[i][0] == v1 { out_indices[i][0] = v2; }
                if out_indices[i][1] == v1 { out_indices[i][1] = v2; }
                if out_indices[i][2] == v1 { out_indices[i][2] = v2; }
            }

            // out_indices = out_indices.into_iter().filter(|x| !(x[0] == v1 && x[1] == v1) || (x[0] == v1 && x[2] == v1) || (x[1] == v1 && x[2] == v1)).collect();
        }
        */

        TriMeshEngine {vertices: out_vertices, indices: out_indices}
    }
    */

    pub fn get_volume_of_mesh(&self) -> f64 {
        let mut sum = 0.0;

        let l = self.indices.len();
        for i in 0..l {
            let a = self.vertices[ self.indices[i][0] ].clone();
            let b = self.vertices[ self.indices[i][1] ].clone();
            let c = self.vertices[ self.indices[i][2] ].clone();

            let sv = signed_volume_of_triangle_ncollide(&a, &b, &c);
            sum += sv;
        }

        sum.abs()
    }
}

