use urdf_rs::*;
use urdf_rs::utils::{rospack_find, expand_package_path};
use std::fs::OpenOptions;
use stl_io;
use std::path::Path;

fn main() {
    let urdf_robo = urdf_rs::read_file("ur5.urdf").unwrap();
    println!("{:?}", urdf_robo.links);
    let collision0 = urdf_robo.links[0].collision.clone();
    println!("{:?}",collision0[0].geometry  );

    let mut fp = "".to_string();
    let geom = collision0[0].geometry.clone();
    match geom {
        urdf_rs::Geometry::Mesh {filename, scale} => {
            fp = filename.clone();
        },
        _ => println!("nope")
    }

    println!("{:?}", fp.clone());
    let res = expand_package_path(&fp.as_str(), Option::None);
    println!("{:?}", res);

    let mut file = OpenOptions::new().read(true).open(res.clone()).unwrap();

    let stl = stl_io::read_stl(&mut file).unwrap();

    println!("{:?}", stl.faces[0].vertices);
}