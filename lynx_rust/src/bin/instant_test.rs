
use std::time::Instant;

fn main() {
    let now = Instant::now();
    let now2 = Instant::now();


    println!("{:?}", &now == &now2);
}