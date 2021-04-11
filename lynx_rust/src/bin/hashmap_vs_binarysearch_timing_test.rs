use std::collections::HashMap;
use std::time::Instant;


fn main() {
    let mut pairs = Vec::new();
    for i in 0..1000 {
        for j in 0..1000 {
            pairs.push((i, j) );
        }
    }

    // println!("{:?}", pairs);

    /*
    let mut h = HashMap::new();

    let l = pairs.len();

    let start = Instant::now();
    for i in 0..100000 {
        h.insert( pairs[i], i );
    }
    println!("{:?}", start.elapsed());


    let start = Instant::now();
    let mut res = Some(&0);
    for i in 0..1000 {
        res = h.get(&(5,100));
    }
    println!("{:?}", start.elapsed());
    println!("{:?}", res);
    */

    let seek = (0,1000);
    let mut res = pairs.binary_search(&seek);

    let start = Instant::now();
    for i in 0..1000 {
        res = pairs.binary_search(&seek);
    }
    println!("{:?}", start.elapsed());
    println!("{:?}", res);
}