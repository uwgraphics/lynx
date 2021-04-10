use urdf_rs::*;
use nalgebra::{Vector3, Matrix3};
use serde::{Serialize, Deserialize};
use crate::utils::utils_parsing::urdf_parsing_utils::recover_real_mesh_file_path_from_ros_style_reference;
use crate::utils::utils_files_and_strings::file_utils::recover_filename_from_full_path;

#[derive(Clone, Serialize, Deserialize)]
pub struct URDFLink {
    pub name: String,
    pub inertial: URDFLinkInertialInfo,
    pub visual: Vec<URDFLinkGeometryInfo>,
    pub collision: Vec<URDFLinkGeometryInfo>,
    pub includes_visual_info: bool,
    pub includes_collision_info: bool
}

impl URDFLink {
    pub fn new_from_urdf_link(link: &Link) -> Self {
        let name = link.name.clone();
        let inertial = URDFLinkInertialInfo::new_from_urdf_link(link);

        let mut visual = Vec::new();
        let l = link.visual.len();
        for i in 0..l {
            visual.push( URDFLinkGeometryInfo::new_from_urdf_visual(&link.visual[i]) );
        }

        let mut collision = Vec::new();
        let l = link.collision.len();
        for i in 0..l {
            collision.push( URDFLinkGeometryInfo::new_from_urdf_collision(&link.collision[i]) );
        }

        let mut includes_visual_info = visual.len() > 0;
        let mut includes_collision_info = collision.len() > 0;

        Self { name, inertial, visual, collision, includes_visual_info, includes_collision_info }
    }

    pub fn new_empty() -> Self {
        return Self{name: "".to_string(), inertial: URDFLinkInertialInfo::new_empty(), visual: Vec::new(), collision: Vec::new(), includes_visual_info: false, includes_collision_info: false };
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct URDFLinkInertialInfo {
    pub included: bool,
    pub origin_xyz: Vector3<f64>,
    pub origin_rpy: Vector3<f64>,
    pub mass: f64,
    pub inertia_matrix: Matrix3<f64>
}

impl URDFLinkInertialInfo {
    pub fn new_from_urdf_link(link: &Link) -> Self {
        let mut included = false;
        let origin_xyz = Vector3::new(  link.inertial.origin.xyz[0], link.inertial.origin.xyz[1], link.inertial.origin.xyz[2]  );
        let origin_rpy = Vector3::new(  link.inertial.origin.rpy[0], link.inertial.origin.rpy[1], link.inertial.origin.rpy[2]  );
        let mass = link.inertial.mass.value.clone();
        let mut inertia_matrix = Matrix3::zeros();

        inertia_matrix[(0,0)] = link.inertial.inertia.ixx;
        inertia_matrix[(1,1)] = link.inertial.inertia.iyy;
        inertia_matrix[(2,2)] = link.inertial.inertia.izz;

        inertia_matrix[(0,1)] = link.inertial.inertia.ixy;
        inertia_matrix[(1,0)] = link.inertial.inertia.ixy;

        inertia_matrix[(0,2)] = link.inertial.inertia.ixz;
        inertia_matrix[(2,0)] = link.inertial.inertia.ixz;

        inertia_matrix[(1,2)] = link.inertial.inertia.iyz;
        inertia_matrix[(2,1)] = link.inertial.inertia.iyz;

        if mass > 0.0 { included = true; }

        Self { included, origin_xyz, origin_rpy, mass, inertia_matrix }
    }

    pub fn new_empty() -> Self {
        return Self { included: false, origin_xyz: Vector3::zeros(), origin_rpy: Vector3::zeros(), mass: 0.0, inertia_matrix: Matrix3::zeros() };
    }
}


#[derive(Clone, Serialize, Deserialize)]
pub struct URDFLinkGeometryInfo {
    pub name: String,
    pub origin_xyz: Vector3<f64>,
    pub origin_rpy: Vector3<f64>,
    pub geometry_type: String,
    pub size: Option<Vector3<f64>>,
    pub radius: Option<f64>,
    pub length: Option<f64>,
    pub ros_filepath: Option<String>,
    pub filename: Option<String>,
    pub scale: Option<Vector3<f64>>
}

impl URDFLinkGeometryInfo {
    pub fn new_from_urdf_visual(arg: &Visual) -> Self {
        let name = arg.name.clone();
        let origin_xyz = Vector3::new(  arg.origin.xyz[0], arg.origin.xyz[1], arg.origin.xyz[2]  );
        let origin_rpy = Vector3::new(  arg.origin.rpy[0], arg.origin.rpy[1], arg.origin.rpy[2]  );
        let mut geometry_type = "".to_string();

        let mut size_: Option<Vector3<f64>> = None;
        let mut radius_: Option<f64> = None;
        let mut length_: Option<f64> = None;
        let mut ros_filepath_: Option<String> = None;
        let mut filename_ : Option<String> = None;
        let mut scale_: Option<Vector3<f64>> = None;

        let g = arg.geometry.clone();
        match g {
            Geometry::Box {size} => {
                geometry_type = "Box".to_string();
                size_ = Some( Vector3::new( size[0], size[1], size[2] ) );
            },
            Geometry::Cylinder {radius, length} => {
                geometry_type = "Cylinder".to_string();
                radius_ = Some(radius);
                length_ = Some(length);
            },
            Geometry::Capsule {radius, length} => {
                geometry_type = "Capsule".to_string();
                radius_ = Some(radius);
                length_ = Some(length);
            },
            Geometry::Sphere { radius } => {
                geometry_type = "Sphere".to_string();
                radius_ = Some(radius);
            },
            Geometry::Mesh {filename, scale} => {
                geometry_type = "Mesh".to_string();
                ros_filepath_ = Some(filename.clone());
                filename_ = recover_filename_from_full_path(filename.clone());
                scale_ = Some(Vector3::new( scale[0], scale[1], scale[2] ));
            }
        }

        Self { name, origin_xyz, origin_rpy, geometry_type, size: size_, radius: radius_,
            length: length_, ros_filepath: ros_filepath_, filename: filename_, scale: scale_ }
    }

    pub fn new_from_urdf_collision(arg: &Collision) -> Self {
        let name = arg.name.clone();
        let origin_xyz = Vector3::new(  arg.origin.xyz[0], arg.origin.xyz[1], arg.origin.xyz[2]  );
        let origin_rpy = Vector3::new(  arg.origin.rpy[0], arg.origin.rpy[1], arg.origin.rpy[2]  );
        let mut geometry_type = "".to_string();

        let mut size_: Option<Vector3<f64>> = None;
        let mut radius_: Option<f64> = None;
        let mut length_: Option<f64> = None;
        let mut ros_filepath_: Option<String> = None;
        let mut filename_ : Option<String> = None;
        let mut scale_: Option<Vector3<f64>> = None;

        let g = arg.geometry.clone();
        match g {
            Geometry::Box {size} => {
                geometry_type = "Box".to_string();
                size_ = Some( Vector3::new( size[0], size[1], size[2] ) );
            },
            Geometry::Cylinder {radius, length} => {
                geometry_type = "Cylinder".to_string();
                radius_ = Some(radius);
                length_ = Some(length);
            },
            Geometry::Capsule {radius, length} => {
                geometry_type = "Capsule".to_string();
                radius_ = Some(radius);
                length_ = Some(length);
            },
            Geometry::Sphere { radius } => {
                geometry_type = "Sphere".to_string();
                radius_ = Some(radius);
            },
            Geometry::Mesh {filename, scale} => {
                geometry_type = "Mesh".to_string();
                ros_filepath_ = Some(filename.clone());
                filename_ = recover_filename_from_full_path(filename.clone());
                scale_ = Some(Vector3::new( scale[0], scale[1], scale[2] ));
            }
        }

        Self { name, origin_xyz, origin_rpy, geometry_type, size: size_, radius: radius_,
            length: length_, ros_filepath: ros_filepath_, filename: filename_, scale: scale_ }
    }
}



