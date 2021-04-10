use urdf_rs::*;
use urdf_rs::utils::{rospack_find, expand_package_path};
use collada::document::ColladaDocument;
use collada::{*};
use std::path::Path;

fn main() {
    let urdf_robo = urdf_rs::read_file("ur5.urdf").unwrap();
    println!("{:?}", urdf_robo.links);
    let visual0 = urdf_robo.links[1].visual.clone();
    println!("{:?}",    visual0[0].geometry  );

    let mut fp = "".to_string();
    let geom = visual0[0].geometry.clone();
    match geom {
        urdf_rs::Geometry::Mesh {filename, scale} => {
            fp = filename.clone();
        },
        _ => println!("nope")
    }

    println!("{:?}", fp.clone());
    // let res = expand_package_path(&fp.as_str(), Option::None);

    // println!("{:?}", res);
    let res = rospack_find(&fp.as_str());
    println!("{:?}", res.is_none());

    let res = expand_package_path(&fp.as_str(), Option::None);

    println!("{:?}", res);

    let path = Path::new(&res);
    let test = ColladaDocument::from_path(&path);

    // let cd = test.ok();
    // println!("{:?}", cd.unwrap().get_obj_set().unwrap().objects[0].geometry[0].mesh[0]);

    let pe_unwrap =  test.ok().unwrap();

    let pe = pe_unwrap.get_obj_set().unwrap().objects[0].geometry[0].mesh[0].clone();
    match pe {
        PrimitiveElement::Polylist( p ) => {
            println!("{:?}", p.shapes[1000]);
            let s = p.shapes[1000];
            match s {
                Shape::Triangle(a,b,c) => println!("{:?}", a.0),
                _ => { }
            }
        },
        PrimitiveElement::Triangles(t) => println!("{:?}", t.vertices[0])
    }

    println!("{:?}", pe_unwrap.get_obj_set().unwrap().objects[0].vertices[0].x)
    // println!("{:?}", pe_unwrap.get_obj_set().unwrap().objects[0].geometry[0].mesh.len())
}