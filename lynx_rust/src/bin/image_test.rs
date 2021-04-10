use std::process::Command;
extern crate lynx_lib;
use lynx_lib::utils::utils_image_environments::image_environment::ImageEnvironment;

fn main() {
    let i = ImageEnvironment::new_collision_image(&"block45".to_string());
    println!("{:?}", i.pixel_width);
    println!("{:?}", i.pixel_height);

    println!("{:?}", i.query_image(-0.5, -0.1, false));
}