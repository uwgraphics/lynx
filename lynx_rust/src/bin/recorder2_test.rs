extern crate lynx_lib;
use lynx_lib::utils::utils_recorders::recorder::*;
use lynx_lib::{write_to_recorder, write_to_recorder_option, write_to_recorder_arc_mutex_option};
use std::thread;
use thread_id;
use rayon::prelude::*;
use std::time::{SystemTime, UNIX_EPOCH, Instant};
use std::collections::HashMap;
use std::ops::{DerefMut};


fn main() {
    /*
    let mut r = Recorder::new();
    write_to_recorder!(&mut r, &vec![0.,0.,0.], "woot");
    let mut a: i128 = 0;
    for i in 0..100 {
        for j in 0..100000 {
            a = a + j;
        }
    }
    write_to_recorder!(&mut r, &vec![1.,0.,0.], "woot2");
    for i in 0..100 {
        for j in 0..10000 {
            a = a + j;
        }
    }
    for i in 0..1000 {
        write_to_recorder!(&mut r, &vec![1.,0.,0.], "woot3");
    }

    let mut r2 = Recorder::new();
    for i in 0..1000 {
        write_to_recorder!(&mut r2, &vec![1.,0.,0.], "woot4");
    }

    r.absorb(&r2);

    r.print_summary();
    r.save_to_json_file_general("test");
     */
    let recorder_arc_mutex_option = RecorderArcMutexOption::new_none();
    for i in 0..1000 {
        write_to_recorder_arc_mutex_option!(&recorder_arc_mutex_option, &1.0, "woot");
    }

    println!("got here");

    let mut r2 = RecorderArcMutexOption::new();
    for i in 0..100 {
        write_to_recorder_arc_mutex_option!(&r2, &2.0, "woot2");
    }
    // r2.print_summary();
    /*
    recorder_arc_mutex_option.absorb(&r2);
    recorder_arc_mutex_option.print_summary();
    recorder_arc_mutex_option.save_to_json_file_general("test");
    */

    // let mut r = recorder_arc_mutex_option.0.lock().unwrap();
    // let mut r2 = r.deref_mut();
    // write_to_recorder_option!(&mut r2, &1.0, "woot");
}