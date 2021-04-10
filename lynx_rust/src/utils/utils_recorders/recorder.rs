use crate::utils::utils_files_and_strings::file_utils::*;
use crate::utils::utils_files_and_strings::string_utils::float_to_string;
use crate::utils::utils_recorders::recorder_utils::get_type_of;
use std::time::{SystemTime, UNIX_EPOCH, Instant, Duration};
use serde::{Serialize, Deserialize};
use termion::{style, color};
use std::collections::HashMap;
use std::sync::{Mutex, Arc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Recorder {
    pub labels: Vec<String>,
    pub time_stamps: Vec<f64>,
    pub adjusted_time_stamps: Vec<f64>,
    pub variable_types: Vec<String>,
    pub variable_names: Vec<String>,
    pub thread_ids: Vec<usize>,
    pub adjusted_thread_ids: Vec<usize>,
    pub variable_json_strings: Vec<String>,
    pub thread_id_to_adjusted_thread_id_hashmap: HashMap<usize, usize>,
    pub curr_thread_count_idx: usize,
    pub total_subtract_time: f64
}
impl Recorder {
    pub fn new() -> Self {
        let labels = Vec::new();
        let time_stamps = Vec::new();
        let adjusted_time_stamps = Vec::new();
        let variable_types = Vec::new();
        let variable_names = Vec::new();
        let thread_ids = Vec::new();
        let adjusted_thread_ids = Vec::new();
        let variable_json_strings = Vec::new();
        let thread_id_to_adjusted_thread_id_hashmap = HashMap::new();
        let curr_thread_count_idx = 0;
        let total_subtract_time = 0.0;

        let mut out_self = Self { labels, time_stamps, adjusted_time_stamps,
            variable_types, variable_names, thread_ids, adjusted_thread_ids, variable_json_strings,
            thread_id_to_adjusted_thread_id_hashmap, curr_thread_count_idx, total_subtract_time };

        write_to_recorder!(&mut out_self, &"none".to_string(), "recorder_start", Instant::now());

        return out_self;
    }

    pub fn load_from_json_file_standard(solver: &str, robot: &str, file_name: &str) -> Result<Self, String> {
        let fp_to_dir = get_path_to_src() + "/fileIO/recorder_outputs/" + solver + "/" + robot;
        return load_from_json_file!(Recorder, &fp_to_dir, file_name);
    }

    pub fn load_from_json_file_general(file_name: &str) -> Result<Self, String> {
        let fp_to_dir = get_path_to_src() + "/fileIO/recorder_outputs/general";
        return load_from_json_file!(Recorder, &fp_to_dir, file_name);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn save_to_json_file_standard(&self, solver: &str, robot: &str, file_name: &str) {
        let fp_to_dir = get_path_to_src() + "/fileIO/recorder_outputs/" + solver + "/" + robot;

        dump_to_json_file!(&self, fp_to_dir.as_str(), file_name);
    }

    pub fn save_to_json_file_general(&self, file_name: &str) {
        let fp_to_dir = get_path_to_src() + "/fileIO/recorder_outputs/general";

        dump_to_json_file!(&self, fp_to_dir.as_str(), file_name);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) -> Result<(), String> {
        let initial_time = self.get_initial_time()?;

        println!();
        println!("{}{}Recorder Summary{}", style::Bold, color::Fg(color::LightCyan), style::Reset);
        println!("{}{}//////////////////////////////////////////////////////////////////////////////{}", style::Bold, color::Fg(color::LightCyan), style::Reset);
        let l = self.time_stamps.len();
        for i in 0..l {
            let adjusted_time_stamp_duration = Duration::from_secs_f64((self.adjusted_time_stamps[i] - initial_time).max(0.0));
            let time_stamp_duration = Duration::from_secs_f64((self.time_stamps[i] - initial_time).max(0.0));

            println!("{}  {:?} ---> {}", color::Fg(color::Blue), i, style::Reset);
            println!("{}    (a) label: {:?} {}", color::Fg(color::LightWhite), self.labels[i], style::Reset);
            println!("{}    (b) adjusted time stamp: {:?}, time stamp: {:?}, true time stamp: {:?} {}", color::Fg(color::LightWhite), adjusted_time_stamp_duration, time_stamp_duration, self.time_stamps[i], style::Reset);
            println!("{}    (c) variable name: {:?}, variable type: {:?} {}", color::Fg(color::LightWhite), self.variable_names[i], self.variable_types[i], style::Reset);
            println!("{}    (d) adjusted thread id: {:?}, thread id: {:?} {}", color::Fg(color::LightWhite), self.adjusted_thread_ids[i], self.thread_ids[i], style::Reset);
            println!("{}    (e) variable: {:?} {}", color::Fg(color::LightWhite), self.variable_json_strings[i], style::Reset);
        }

        let final_time = self.get_final_time()?;
        let final_time_stamp_duration = Duration::from_secs_f64((final_time - initial_time).max(0.0));
        let num_threads = self.thread_id_to_adjusted_thread_id_hashmap.keys().len();

        println!("{}{}Overall Recorder Summary ---> final time stamp: {:?}, num threads: {:?} {}", style::Bold, color::Fg(color::LightCyan), final_time_stamp_duration, num_threads, style::Reset);
        println!("{}{}//////////////////////////////////////////////////////////////////////////////{}", style::Bold, color::Fg(color::LightCyan), style::Reset);
        println!();

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn absorb(&mut self, other: &Recorder) -> Result<(), String> {
        let final_adjusted_time = self.adjusted_time_stamps[self.adjusted_time_stamps.len() - 1];
        let other_start_time = other.get_initial_time()?;
        let l = other.time_stamps.len();
        for i in 0..l {
            let added_adjusted_time = final_adjusted_time + (other.adjusted_time_stamps[i] - other_start_time);
            let binary_search_res = self.adjusted_time_stamps.binary_search_by(|x| x.partial_cmp(&added_adjusted_time).unwrap());
            match binary_search_res {
                Ok(j) => {
                    self.labels.insert(j, other.labels[i].clone());
                    self.time_stamps.insert(j, other.time_stamps[i]);
                    self.adjusted_time_stamps.insert(j, added_adjusted_time);
                    // self.subtract_times.insert(j, other.subtract_times[i]);
                    self.variable_types.insert(j, other.variable_types[i].clone());
                    self.variable_names.insert(j, other.variable_names[i].clone());
                    self.thread_ids.insert(j, other.thread_ids[i]);
                    let adjusted_thread_id = self.get_adjusted_thread_id(other.thread_ids[i]);
                    self.adjusted_thread_ids.insert(j, adjusted_thread_id);
                    self.variable_json_strings.insert(j, other.variable_json_strings[i].clone());
                },
                Err(j) => {
                    self.labels.insert(j, other.labels[i].clone());
                    self.time_stamps.insert(j, other.time_stamps[i]);
                    self.adjusted_time_stamps.insert(j, added_adjusted_time);
                    // self.subtract_times.insert(j, other.subtract_times[i]);
                    self.variable_types.insert(j, other.variable_types[i].clone());
                    self.variable_names.insert(j, other.variable_names[i].clone());
                    self.thread_ids.insert(j, other.thread_ids[i]);
                    let adjusted_thread_id = self.get_adjusted_thread_id(other.thread_ids[i]);
                    self.adjusted_thread_ids.insert(j, adjusted_thread_id);
                    self.variable_json_strings.insert(j, other.variable_json_strings[i].clone());
                }
            }
        }
        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    /*
    pub fn map_thread_ids_to_readable_counting_numbers(&mut self) {
        let mut h : HashMap<usize, usize> = HashMap::new();
        let mut curr_count = 0 as usize;

        let l = self.thread_ids.len();
        for i in 0..l {
            let curr_thread_id = self.thread_ids[i];
            let map_option = h.get(&curr_thread_id);
            if map_option.is_none() {
                h.insert( curr_thread_id, curr_count );
                self.thread_ids[i] = curr_count;
                curr_count += 1;
            } else {
                self.thread_ids[i] = *map_option.unwrap();
            }
        }
    }

    pub fn get_thread_ids_to_readable_counting_numbers_hashmap(&self) -> HashMap<usize, usize> {
        let mut h : HashMap<usize, usize> = HashMap::new();
        let mut curr_count = 0 as usize;

        let l = self.thread_ids.len();
        for i in 0..l {
            let curr_thread_id = self.thread_ids[i];
            let map_option = h.get(&curr_thread_id);
            if map_option.is_none() {
                h.insert( curr_thread_id, curr_count );
                curr_count += 1;
            }
        }

        return h;
    }
    */

    pub fn get_adjusted_thread_id(&mut self, thread_id: usize) -> usize {
        let o = self.thread_id_to_adjusted_thread_id_hashmap.get(&thread_id);
        if o.is_none() {
            self.thread_id_to_adjusted_thread_id_hashmap.insert(thread_id, self.curr_thread_count_idx);
            self.curr_thread_count_idx += 1;
            return self.curr_thread_count_idx - 1;
        } else {
            return *o.unwrap();
        }
    }

    pub fn get_initial_time(&self) -> Result<f64, String> {
        if self.time_stamps.is_empty() {
            return Err("time stamps is empty in get_initial_time".to_string());
        }
        return Ok(self.time_stamps[0]);
    }

    pub fn get_final_time(&self) -> Result<f64, String> {
        if self.adjusted_time_stamps.is_empty() {
            return Err("time stamps is empty in get_initial_time".to_string());
        }
        let l = self.adjusted_time_stamps.len();
        return Ok(self.adjusted_time_stamps[l-1]);
    }
}

#[derive(Debug, Clone)]
pub struct RecorderOption(pub Option<Recorder>);
impl RecorderOption {
    pub fn new() -> Self {
        return RecorderOption(Some(Recorder::new()));
    }

    pub fn new_none() -> Self {
        return RecorderOption(None);
    }

    pub fn load_from_json_file_standard(solver: &str, robot: &str, file_name: &str) -> Result<Self, String> {
        let recorder = Recorder::load_from_json_file_standard(solver, robot, file_name)?;
        return Ok(RecorderOption(Some(recorder)));
    }

    pub fn load_from_json_file_general(file_name: &str) -> Result<Self, String> {
        let recorder = Recorder::load_from_json_file_general(file_name)?;
        return Ok(RecorderOption(Some(recorder)));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn save_to_json_file_standard(&self, solver: &str, robot: &str, file_name: &str) {
        if self.0.is_none() {
            println!("{}{}RecorderOption was None, so nothing was saved to a json file.{}", style::Bold, color::Fg(color::Yellow), style::Reset);
            return;
        }

        self.0.as_ref().unwrap().save_to_json_file_standard(solver, robot, file_name);
    }

    pub fn save_to_json_file_general(&self, file_name: &str) {
        if self.0.is_none() {
            println!("{}{}RecorderOption was None, so nothing was saved to a json file.{}", style::Bold, color::Fg(color::Yellow), style::Reset);
            return;
        }

        self.0.as_ref().unwrap().save_to_json_file_general(file_name);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) -> Result<(), String> {
        return if self.0.is_none() {
            println!("{}{}RecorderOption is none.{}", style::Bold, color::Fg(color::LightCyan), style::Reset);
            Ok(())
        } else {
            self.0.as_ref().unwrap().print_summary()
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn absorb(&mut self, other: &RecorderOption) {
        if self.0.is_none() { return; }
        if other.0.is_none() { return; }

        self.0.as_mut().unwrap().absorb(other.0.as_ref().unwrap());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    /*
    pub fn map_thread_ids_to_readable_counting_numbers(&mut self) {
        if self.0.is_none() { return; }
        self.0.as_mut().unwrap().map_thread_ids_to_readable_counting_numbers();
    }
    */
}

#[derive(Debug, Clone)]
pub struct RecorderArcMutexOption{
    pub recorder_arc_mutex_option: Arc<Mutex<RecorderOption>>,
    _none: bool
}

impl RecorderArcMutexOption {
    pub fn new() -> Self {
        let recorder_arc_mutex_option = Arc::new(Mutex::new( RecorderOption::new() ));
        return RecorderArcMutexOption { recorder_arc_mutex_option, _none: false };
    }

    pub fn new_none() -> Self {
        let recorder_arc_mutex_option = Arc::new(Mutex::new(RecorderOption::new_none()));
        return RecorderArcMutexOption { recorder_arc_mutex_option, _none: true }
    }

    pub fn load_from_json_file_standard(solver: &str, robot: &str, file_name: &str) -> Result<Self, String> {
        let recorder = Recorder::load_from_json_file_standard(solver, robot, file_name)?;
        let recorder_arc_mutex_option = Arc::new(Mutex::new(RecorderOption( Some(recorder) )) );
        return Ok(Self { recorder_arc_mutex_option, _none: false });
    }

    pub fn load_from_json_file_general(file_name: &str) -> Result<Self, String> {
        let recorder = Recorder::load_from_json_file_general(file_name)?;
        let recorder_arc_mutex_option = Arc::new(Mutex::new(RecorderOption( Some(recorder) )) );
        return Ok(Self { recorder_arc_mutex_option, _none: false });
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn save_to_json_file_standard(&self, solver: &str, robot: &str, file_name: &str) {
        if self.is_none() {
            println!("{}{}RecorderMutexOption was None, so nothing was saved to a json file.{}", style::Bold, color::Fg(color::Yellow), style::Reset);
            return;
        }

        self.recorder_arc_mutex_option.lock().unwrap().0.as_ref().unwrap().save_to_json_file_standard(solver, robot, file_name);
    }

    pub fn save_to_json_file_general(&self, file_name: &str) {
        if self.is_none() {
            println!("{}{}RecorderMutexOption was None, so nothing was saved to a json file.{}", style::Bold, color::Fg(color::Yellow), style::Reset);
            return;
        }

        self.recorder_arc_mutex_option.lock().unwrap().0.as_ref().unwrap().save_to_json_file_general(file_name);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) -> Result<(), String> {
        return if self.is_none() {
            println!("{}{}RecorderMutexOption is none.{}", style::Bold, color::Fg(color::LightCyan), style::Reset);
            Ok(())
        } else {
            self.recorder_arc_mutex_option.lock().unwrap().0.as_ref().unwrap().print_summary()
        }
    }

    pub fn is_none(&self) -> bool {
        return self._none;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn absorb(&self, other: &RecorderArcMutexOption) {
        if self.is_none() { return; }
        if other.is_none() { return; }

        self.recorder_arc_mutex_option.lock().unwrap().0.as_mut().unwrap().absorb(other.recorder_arc_mutex_option.lock().unwrap().0.as_ref().unwrap());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    /*
    pub fn map_thread_ids_to_readable_counting_numbers(&mut self) {
        if self.0.is_none() { return; }
        self.0.as_mut().unwrap().map_thread_ids_to_readable_counting_numbers();
    }
    */
}
unsafe impl Send for RecorderArcMutexOption { }
unsafe impl Sync for RecorderArcMutexOption { }
