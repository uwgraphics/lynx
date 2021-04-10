
use vpsearch::{*};
use nalgebra::{DVector, Vector3, Vector};
// use crate::lib::utils_mist::nalgebra_utils::vec_to_dvec;
// use crate::lib::utils_rust::file_utils::{*};


pub struct VPSearch3D {
    pub tree: Tree<Point3D>,
    pub points: Vec<Vector3<f64>>
}

impl VPSearch3D {
    pub fn new(v: &Vec<Vector3<f64>>) -> Self {
        let mut init_vec: Vec<Point3D> = Vec::new();
        let mut points_vec: Vec<Vector3<f64>> = Vec::new();
        let l = v.len();
        for i in 0..l {
            points_vec.push(v[i].clone());
            let tmp = Point3D::new(v[i].clone());
            init_vec.push(tmp);
        }

        let tree = Tree::new(&init_vec);
        Self{tree, points: points_vec}
    }

    pub fn new_vec(v: &Vec<Vec<f64>>) -> Self {
        let mut vec3s = Vec::new();
        let l = v.len();
        for i in 0..l {
            vec3s.push( Vector3::new(v[i][0], v[i][1], v[i][2]) );
        }
        return Self::new( &vec3s );
    }

    pub fn get_closest(&self, point: &Vector3<f64>) -> (usize, f64) {
        let (index, _) = self.tree.find_nearest(&Point3D::new(point.clone()));
        let dis = (point - &self.points[index]).norm();
        (index, dis)
    }

    pub fn get_closest_vec(&self, point: &Vec<f64>) -> (usize, f64) {
        let v3 = Vector3::new(point[0], point[1], point[2]);
        self.get_closest(&v3)
    }
}


#[derive(Copy, Clone)]
pub struct Point3D {
    point: Vector3<f64>
}

impl vpsearch::MetricSpace for Point3D {
    type UserData = ();
    type Distance = f64;

    fn distance(&self, other: &Self, _: &Self::UserData) -> Self::Distance {
        return (&self.point - &other.point).norm();
    }
}

impl Point3D {
    pub fn new(point: Vector3<f64>) -> Self {
        Self {point}
    }
}


/*
pub struct VPSearch3D {
    pub tree: Tree<Point3D>,
    pub points: Vec<DVector<f64>>
}

impl VPSearch3D {
    pub fn new(v: &Vec<Vec<f64>>) -> Self {
        let mut init_vec: Vec<Point3D> = Vec::new();
        let mut points_vec: Vec<DVector<f64>> = Vec::new();
        let l = v.len();
        for i in 0..l {
            points_vec.push(vec_to_dvec(&v[i]));
            let tmp = Point3D::new(v[i][0], v[i][1], v[i][2]);
            init_vec.push(tmp);
        }

        let tree = Tree::new(&init_vec);
        Self{tree, points: points_vec}
    }

    pub fn new_from_pointcloud_file(name: &String) -> Self {
        let mut v: Vec<Vec<f64>> = Vec::new();
        let path_to_src = get_path_to_src();
        let fp = path_to_src + "FileIO/point_clouds/" + name;
        let contents = get_file_contents(fp);
        let mut lines = contents.lines();
        for l in lines {
            let s = l.split(",");
            let mut tmp: Vec<f64> = Vec::new();
            for x in s {
                tmp.push(x.parse().unwrap());
            }
            v.push(tmp);
        }
        VPSearch3D::new(&v)
    }

    pub fn get_closest(&self, point: &Vector3<f64>) -> (usize, f64) {
        let (index, _) = self.tree.find_nearest(&Point3D::new(point[0], point[1], point[2]));
        let dis = (point - &self.points[index]).norm();
        (index, dis)
    }

    pub fn get_closest_vec(&self, point: &Vec<f64>) -> (usize, f64) {
        let v3 = Vector3::new(point[0], point[1], point[2]);
        self.get_closest(&v3)
    }
}


pub struct VPSearch2D {
    pub tree: Tree<Point2D>,
    pub points: Vec<DVector<f64>>
}

impl VPSearch2D {
    pub fn new(v: &Vec<Vec<f64>>) -> Self {
        let mut init_vec: Vec<Point2D> = Vec::new();
        let mut points_vec: Vec<DVector<f64>> = Vec::new();
        let l = v.len();
        for i in 0..l {
            points_vec.push(vec_to_dvec(&v[i]));
            let tmp = Point2D::new(v[i][0], v[i][1]);
            init_vec.push(tmp);
        }

        let tree = Tree::new(&init_vec);
        Self{tree, points: points_vec}
    }

    pub fn get_closest(&self, point: &DVector<f64>) -> (usize, f64) {
        let (index, _) = self.tree.find_nearest(&Point2D::new(point[0], point[1]));
        let dis = (point - &self.points[index]).norm();
        (index, dis)
    }

    pub fn get_closest_vec(&self, point: &Vec<f64>) -> (usize, f64) {
        self.get_closest(&vec_to_dvec(point))
    }
}




#[derive(Copy, Clone)]
pub struct Point2D {
    x: f64, y: f64,
}

impl vpsearch::MetricSpace for Point2D {
    type UserData = ();
    type Distance = f64;

    fn distance(&self, other: &Self, _: &Self::UserData) -> Self::Distance {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx*dx + dy*dy).sqrt()
    }
}

impl Point2D {
    pub fn new(x: f64, y: f64) -> Self {
        Self{x, y}
    }
}


#[derive(Copy, Clone)]
pub struct Point3D {
    x: f64, y: f64, z:f64
}

impl vpsearch::MetricSpace for Point3D {
    type UserData = ();
    type Distance = f64;

    fn distance(&self, other: &Self, _: &Self::UserData) -> Self::Distance {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx*dx + dy*dy + dz*dz).sqrt()
    }
}

impl Point3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self{x, y, z}
    }
}




#[derive(Copy, Clone)]
pub struct DVecPoint<'a> {
    v: &'a DVector<f64>
}

impl<'a> vpsearch::MetricSpace for DVecPoint<'a> {
    type UserData = ();
    type Distance = f64;

    fn distance(&self, other: &Self, _: &Self::UserData) -> Self::Distance {
        (self.v - other.v).norm()
    }
}

impl<'a> DVecPoint<'a> {
    pub fn new_from_dvec(vec: &'a DVector<f64>) -> Self {
        Self{v: vec}
    }
}
*/