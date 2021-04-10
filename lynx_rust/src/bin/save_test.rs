use serde::{Serialize, Deserialize};
use std::fs::File;
use std::io::prelude::*;


#[derive(Serialize, Deserialize, Debug)]
struct Point {
    x: i32,
    y: i32,
}

fn main() {
    let point = Point { x: 1, y: 2 };

    let serialized = serde_json::to_string(&point).unwrap();
    println!("serialized = {}", serialized);

    serialized.as_bytes();

    let mut file = File::create("test.json");
    file.unwrap().write(serialized.as_bytes());

    let deserialized: Point = serde_json::from_str(&serialized).unwrap();
    println!("deserialized = {:?}", deserialized);
}