
use std::cmp::Ordering;

fn main() {

    let mut v = vec![ (0,1), (0,3), (5,2), (4,3), (5,3), (0,0), (1,1), (1,0) ];

    // v.sort_by_key(|x| x.0);

    // println!("{:?}", v);

    /*
    let a = 2;
    let b = 1;

    let c = a.cmp(&b);
    println!("{:?}", c == Ordering::Equal);
    */

    /*
    v.sort_by(|a, b|  {
        a.cmp(&b)
    } );

    println!("{:?}", v);
    */

    v.sort();
    let seek = (0,4);
    let res = v.binary_search_by(|a| a.cmp(&&seek));
    match res {
        Err(i) => { v.insert(i, seek); }, 
        Ok(i) => { v.insert(i, seek); }
    }

    println!("{:?}", v);
    println!("{:?}", res);
}