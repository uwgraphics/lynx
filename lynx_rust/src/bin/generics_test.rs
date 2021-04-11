use std::time::{Instant,Duration};
use std::collections::HashMap;


fn add_var<T> (list: &mut Vec<T>, item: T) {
    list.push(item);
}

fn get_var<T> (list: &mut Vec<T>, idx: usize) -> &T {
    return &mut list[idx];
}

fn main() {

    let mut list = vec![1,2,3];

    add_var( &mut list, 4 );

    println!("{:?}", list);

    let val = get_var(&mut list, 0);
    println!("{:?}", val);


    let mut h: HashMap<&'static str, usize> = HashMap::new();

    h.insert("hi", 0);

    let res = h.get("hi");
    println!("{:?}", res);
}