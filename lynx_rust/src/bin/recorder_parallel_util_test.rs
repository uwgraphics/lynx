extern crate lynx_lib;
use lynx_lib::utils::utils_recorders::prelude::*;
use lynx_lib::*;
use rayon::prelude::*;

fn main() {
    let central_recorder = RecorderArcMutexOption::new();

    let a = 1.0;
    write_to_recorder_arc_mutex_option!(&central_recorder, &a, "test_float");

    let mut p = RecorderParallelUtil::new_from_recorder_type(&central_recorder);
    let aa : Vec<usize> = (0..12).collect();
    aa.par_iter().for_each(|x| {
        let recorder_on_thread = p.spawn();
        write_to_recorder_arc_mutex_option!(&recorder_on_thread, x, "woah");

        p.collect_recorder_from_thread(recorder_on_thread);
    });
    p.absorb_collected_recorders_into_central_recorder(&central_recorder);
    central_recorder.print_summary();

    
}