extern crate lynx_lib;
use lynx_lib::utils::utils_nearest_neighbor::kdtree_utils::KDTree;
use lynx_lib::optimal_motion_planning::prelude::vec_to_dvec;
use std::time::{Instant, Duration};

fn main() {
    /*
    for i in 0..1000 {
        let mut k = KDTree::new_random(200, 1000);

        // k.print_summary();

        let start = Instant::now();
        let res1 = k.get_closest(&vec_to_dvec(&vec![0.0; 200]));
        let stop = start.elapsed();
        // println!("{:?}", res1);
        // println!("{:?}", stop);

        let start = Instant::now();
        let res2 = k.get_closest_brute_force(&vec_to_dvec(&vec![0.0; 200]));
        let stop = start.elapsed();
        // println!("{:?}", res2);
        // println!("{:?}", stop);

        if !(res1 == res2) {
            k.print_summary();
            println!("{:?}", res1);
            println!("{:?}", res2);
        }
    }
    */
    let mut k = KDTree::new_random(100, 50000);

    let start = Instant::now();
    let res = k.get_closest_k(&vec_to_dvec(&vec![0.; 100]), 1);
    let stop = start.elapsed();

    let start = Instant::now();
    let res2 = k.get_closest_k_brute_force(&vec_to_dvec(&vec![0.; 100]), 1);
    let stop2 = start.elapsed();

    println!("{:?}", res);
    println!("{:?}", res2);
    println!("{:?}", stop);
    println!("{:?}", stop2);

    // let mut k = KDTree::new_random(15, 1000);
    // k.print_summary();

    /*
    let start = Instant::now();
    let c = k.get_closest(&vec_to_dvec(&vec![0.; 10]));
    let end = start.elapsed();
    println!("{:?}", end);
    println!("{:?}", c);
    */

}