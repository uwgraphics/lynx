
use std::time::Instant;
use rayon::prelude::*;

fn main() {
    let a = (0..12);
    println!("{:?}", a);


    let a: Vec<usize> = (0..2).collect();
    let b: Vec<usize> = (0..12).collect();

    let c = a.iter().zip(b.iter());

    let start = Instant::now();
    let mut vec = Vec::new();
    for i in 0..9 {
        for j in 0..100 {
            vec.push( (i,j) );
        }
    }

    println!("{:?}", start.elapsed());
    println!("{:?}", vec.len());

    let a : Vec<usize> = (0..12).collect();

    let start = Instant::now();
    a.par_iter().for_each(|x| {

    });
    println!("{:?}", start.elapsed());

}