
////////////////////////////////////////////////////////////////////////////////////////////////////

#[macro_use] #[macro_export]
macro_rules! get_lynx_vars_tuple_mut_ref_from_ident {
    ($lynx_vars: expr, $i: ident) => {
        {
            &mut $lynx_vars.$i
        };
    }
}

#[macro_use] #[macro_export]
macro_rules! get_lynx_vars_tuple_ref_from_ident {
    ($lynx_vars: expr, $i: ident) => {
        {
            &$lynx_vars.$i
        };
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[macro_use] #[macro_export]
macro_rules! add_lynx_var {
    ($lynx_vars: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            if $lynx_vars.all_var_types_and_names.contains( &(stringify!($i), $variable_name) ) {
                let out_val: Result<(), String> = Err(format!("variable {} of type {} already exists, not added.", $variable_name, stringify!($i)));
                out_val
            } else {
                let mut tuple = get_lynx_vars_tuple_mut_ref_from_ident!( $lynx_vars, $i );
                tuple.0.push($variable);
                tuple.1.insert( $variable_name, tuple.2);
                tuple.2 += 1;
                $lynx_vars.all_var_types_and_names.push( ( stringify!($i), $variable_name ) );
                let out_val : Result<(), String> = Ok(());
                out_val
            }
        }
    };
}

/*
#[macro_use] #[macro_export]
macro_rules! set_lynx_var_via_idx {
    ($lynx_vars: expr, $i: ident, $idx: expr, $variable: expr) => {
        {
            let mut tuple = get_lynx_vars_tuple_mut_ref_from_ident!( $lynx_vars, $i );
            if $idx >= tuple.0.len() {
                let out_val : Result<(), String> = Err(format!("idx {} is too high to set a value of type {} (length is {}))", $idx, stringify!($i), tuple.0.len()));
                out_val
            } else {
                tuple.0[$idx] = $variable;
                let out_val : Result<(), String> = Ok(());
                out_val
            }
        }
    };
}
*/

#[macro_use] #[macro_export]
macro_rules! set_lynx_var {
    ($lynx_vars: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            let mut tuple = get_lynx_vars_tuple_mut_ref_from_ident!( $lynx_vars, $i );
            let idx = tuple.1.get( $variable_name );
            if idx.is_none() {
                let out_val : Result<(), String> = Err(format!("variable {} of type {} not found, could not be set.", $variable_name, stringify!($i)));
                out_val
            } else {
                // let out_val : Result<(), String> = set_lynx_var_via_idx!($lynx_vars, $i, *idx.unwrap(), $variable);
                // out_val
                tuple.0[*idx.unwrap()] = $variable;
                let out_val : Result<(), String> = Ok(());
                out_val
            }
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! set_or_add_lynx_var {
    ($lynx_vars: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            let res = get_lynx_var_idx!($lynx_vars, $i, $variable_name);

            if res.is_ok() {
                set_lynx_var!($lynx_vars, $i, $variable_name, $variable);
                let out_val : Result<(), String> = Ok(());
                out_val
            } else {
                add_lynx_var!($lynx_vars, $i, $variable_name, $variable);
                let out_val : Result<(), String> = Ok(());
                out_val
            }
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! get_lynx_var_idx {
    ($lynx_vars: expr, $i: ident, $variable_name: expr) => {
        {
            let mut tuple = get_lynx_vars_tuple_mut_ref_from_ident!( $lynx_vars, $i );
            let idx = tuple.1.get( $variable_name );
            if idx.is_none() {
                // println!("ERROR: variable {} of type {} not found in lynx vars.", $variable_name, stringify!($i));
                let out_val : Result<usize, String> = Err(format!("variable {} of type {} not found.", $variable_name, stringify!($i)));
                out_val
            } else {
                let out_val : Result<usize, String> = Ok(*idx.unwrap());
                out_val
            }
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! get_lynx_var_ref {
    ($lynx_vars: expr, $i: ident, $variable_name: expr) => {
        {
            let tuple = get_lynx_vars_tuple_ref_from_ident!( $lynx_vars, $i );
            let idx = tuple.1.get( $variable_name );
            if idx.is_none() {
                // println!("ERROR: variable {} of type {} not found in lynx vars.", $variable_name, stringify!($i));
                let out_val : Result<&$i, String> = Err(format!("variable {} of type {} not found.", $variable_name, stringify!($i)));
                out_val
            } else {
                let out_val : Result<&$i, String> = Ok( &tuple.0[*idx.unwrap()] );
                out_val
            }
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! get_lynx_var_mut_ref {
    ($lynx_vars: expr, $i: ident, $variable_name: expr) => {
        {
            let mut tuple = get_lynx_vars_tuple_mut_ref_from_ident!( $lynx_vars, $i );
            let idx = tuple.1.get( $variable_name );
            if idx.is_none() {
                // println!("ERROR: variable {} of type {} not found in lynx vars.", $variable_name, stringify!($i));
                let out_val : Result<&mut $i, String> = Err(format!("variable {} of type {} not found.", $variable_name, stringify!($i)));
                out_val
            } else {
                let out_val : Result<&mut $i, String> = Ok( &mut tuple.0[*idx.unwrap()] );
                out_val
            }
        }
    };
}

/*
#[macro_use] #[macro_export]
macro_rules! get_lynx_var_ref_via_idx {
    ($lynx_vars: expr, $i: ident, $idx: expr) => {
        {
            let tuple = get_lynx_vars_tuple_ref_from_ident!( $lynx_vars, $i );
            if $idx >= tuple.0.len() {
                let out_val : Result<&$i, String> = Err(format!("idx {} is too high to get a variable of type {} (length is {}))", $idx, stringify!($i), tuple.0.len()));
                out_val
            } else {
                let out_val : Result<&$i, String> = Ok( &tuple.0[$idx] );
                out_val
            }
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! get_lynx_var_mut_ref_via_idx {
    ($lynx_vars: expr, $i: ident, $idx: expr) => {
        {
            let tuple = get_lynx_vars_tuple_mut_ref_from_ident!( $lynx_vars, $i );
            if $idx >= tuple.0.len() {
                let out_val : Result<&mut $i, String> = Err(format!("idx {} is too high to get a variable of type {} (length is {}))", $idx, stringify!($i), tuple.0.len()));
                out_val
            } else {
                let out_val : Result<&mut $i, String> = Ok( &mut tuple.0[$idx] );
                out_val
            }
        }
    };
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////

#[macro_use] #[macro_export]
macro_rules! add_lynx_var_parallel {
    ($lynx_vars_parallel: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            let num_threads = $lynx_vars_parallel.get_num_threads();
            let mut par_iter_mut = $lynx_vars_parallel.get_par_iter_mut();
            par_iter_mut.for_each(|x| {
                add_lynx_var!( x, $i, $variable_name, $variable.clone() );
            });
            let out_val : Result<(), String> = Ok(());
            out_val
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! set_lynx_var_parallel {
    ($lynx_vars_parallel: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            let num_threads = $lynx_vars_parallel.get_num_threads();
            let mut par_iter_mut = $lynx_vars_parallel.get_par_iter_mut();
            par_iter_mut.for_each(|x| {
                set_lynx_var!( x, $i, $variable_name, $variable.clone() );
            });
            let out_val : Result<(), String> = Ok(());
            out_val
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! set_or_add_lynx_var_parallel {
    ($lynx_vars_parallel: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            let num_threads = $lynx_vars_parallel.get_num_threads();
            let mut par_iter_mut = $lynx_vars_parallel.get_par_iter_mut();
            par_iter_mut.for_each(|x| {
                set_or_add_lynx_var!( x, $i, $variable_name, $variable.clone() );
            });
            let out_val : Result<(), String> = Ok(());
            out_val
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! get_lynx_var_idx_parallel {
    ($lynx_vars_parallel: expr, $i: ident, $variable_name: expr) => {
        {
            let lynx_vars = $lynx_vars_parallel.get_first_ref();
            let out = get_lynx_var_idx!(lynx_vars, $i, $variable_name);
            out
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! get_lynx_var_ref_parallel {
    ($lynx_vars_parallel: expr, $i: ident, $variable_name: expr) => {
        {
            let lynx_vars = $lynx_vars_parallel.get_first_ref();
            let out = get_lynx_var_ref!(lynx_vars, $i, $variable_name);
            out
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! get_lynx_var_mut_ref_parallel {
    ($lynx_vars_parallel: expr, $i: ident, $variable_name: expr) => {
        {
            let lynx_vars = $lynx_vars_parallel.get_first_mut_ref();
            let out = get_lynx_var_mut_ref!(lynx_vars, $i, $variable_name);
            out
        }
    };
}

////////////////////////////////////////////////////////////////////////////////////////////////////


/*
NOTE:
Any functions that use this macro will need

use rayon::prelude::*;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! add_lynx_var_generic {
    ($lynx_vars_generic: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            match $lynx_vars_generic {
                LynxVarsGeneric::SingleThreaded(v) => add_lynx_var!(v, $i, $variable_name, $variable),
                LynxVarsGeneric::SingleThreadedMutRef(v) => add_lynx_var!(v, $i, $variable_name, $variable),
                LynxVarsGeneric::Parallel(v) => add_lynx_var_parallel!(v, $i, $variable_name, $variable),
            }
        }
    };
}

/*
NOTE:
Any functions that use this macro will need

use rayon::prelude::*;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! set_lynx_var_generic {
    ($lynx_vars_generic: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            match $lynx_vars_generic {
                LynxVarsGeneric::SingleThreaded(v) => set_lynx_var!(v, $i, $variable_name, $variable),
                LynxVarsGeneric::SingleThreadedMutRef(v) => set_lynx_var!(v, $i, $variable_name, $variable),
                LynxVarsGeneric::Parallel(v) => set_lynx_var_parallel!(v, $i, $variable_name, $variable),
            }
        }
    };
}

/*
NOTE:
Any functions that use this macro will need

use rayon::prelude::*;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! set_or_add_lynx_var_generic {
    ($lynx_vars_generic: expr, $i: ident, $variable_name: expr, $variable: expr) => {
        {
            match $lynx_vars_generic {
                LynxVarsGeneric::SingleThreaded(v) => set_or_add_lynx_var!(v, $i, $variable_name, $variable),
                LynxVarsGeneric::SingleThreadedMutRef(v) => set_or_add_lynx_var!(v, $i, $variable_name, $variable),
                LynxVarsGeneric::Parallel(v) => set_or_add_lynx_var_parallel!(v, $i, $variable_name, $variable),
            }
        }
    };
}

/*
NOTE:
Any functions that use this macro will need

use rayon::prelude::*;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! get_lynx_var_idx_generic {
    ($lynx_vars_generic: expr, $i: ident, $variable_name: expr) => {
        {
            match $lynx_vars_generic {
                LynxVarsGeneric::SingleThreaded(v) => get_lynx_var_idx!(v, $i, $variable_name),
                LynxVarsGeneric::SingleThreadedMutRef(v) => get_lynx_var_idx!(v, $i, $variable_name),
                LynxVarsGeneric::Parallel(v) => get_lynx_var_idx_parallel!(v, $i, $variable_name),
            }
        }
    };
}

/*
NOTE:
Any functions that use this macro will need

use rayon::prelude::*;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! get_lynx_var_ref_generic {
    ($lynx_vars_generic: expr, $i: ident, $variable_name: expr) => {
        {
            match $lynx_vars_generic {
                LynxVarsGeneric::SingleThreaded(v) => get_lynx_var_ref!(v, $i, $variable_name),
                LynxVarsGeneric::SingleThreadedMutRef(v) => get_lynx_var_ref!(v, $i, $variable_name),
                LynxVarsGeneric::Parallel(v) => get_lynx_var_ref_parallel!(v, $i, $variable_name),
            }
        }
    };
}

/*
NOTE:
Any functions that use this macro will need

use rayon::prelude::*;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! get_lynx_var_mut_ref_generic {
    ($lynx_vars_generic: expr, $i: ident, $variable_name: expr) => {
        {
            match $lynx_vars_generic {
                LynxVarsGeneric::SingleThreaded(v) => get_lynx_var_mut_ref!(v, $i, $variable_name),
                LynxVarsGeneric::SingleThreadedMutRef(v) => get_lynx_var_mut_ref!(v, $i, $variable_name),
                LynxVarsGeneric::Parallel(v) => get_lynx_var_mut_ref_parallel!(v, $i, $variable_name),
            }
        }
    };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[macro_use] #[macro_export]
macro_rules! load_from_json_string {
    ($json_string: expr, $type: ident) => {
        {
            let out : $type = serde_json::from_str($json_string).unwrap();
            out
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! dump_to_json_file {
    ($variable: expr, $fp_to_dir: expr, $file_name: expr) => {
       // let p = $crate::utils::utils_files_and_strings::file_utils::get_path_to_src();
       let serialized = serde_json::to_string($variable).unwrap();
       $crate::utils::utils_files_and_strings::file_utils::write_string_to_file($fp_to_dir.to_string(), $file_name.to_string(), serialized, true);
    };
}

#[macro_use] #[macro_export]
macro_rules! load_from_json_file {
    ($type: ident, $fp_to_dir: expr, $file_name: expr) => {
        {
            let file_contents_string_option = $crate::utils::utils_files_and_strings::file_utils::read_file_contents_separated_args($fp_to_dir.to_string(), $file_name.to_string());
            if file_contents_string_option.is_none() {
                let out : Result< $type, String > = Err("file contents was None in load_from_json_file".to_string());
                out
            } else {
                let file_contents_string = file_contents_string_option.unwrap();
                let out : Result<$type, String> = Ok(serde_json::from_str(&file_contents_string).unwrap());
                out
            }
        }
    };
}

#[macro_use] #[macro_export]
macro_rules! dump_to_json_file_relative_to_robot_directory {
    ($variable: expr, $robot_name: expr, $partial_fp_to_dir: expr, $file_name: expr) => {
       let serialized = serde_json::to_string($variable).unwrap();
       $crate::utils::utils_files_and_strings::robot_folder_utils::write_string_to_file_relative_to_robot_directory($robot_name.to_string(), $partial_fp_to_dir.to_string(), $file_name.to_string(), serialized, true);
    };
}

#[macro_use] #[macro_export]
macro_rules! load_from_json_file_relative_to_robot_directory {
    ($type: ident, $robot_name: expr, $partial_fp_to_dir: expr, $file_name: expr) => {
        {
            let file_contents_string_option = $crate::utils::utils_files_and_strings::robot_folder_utils::read_file_contents_separated_args_relative_to_robot_directory($robot_name.to_string(), $partial_fp_to_dir.to_string(), $file_name.to_string());
            if file_contents_string_option.is_none() {
                let out : Result< $type, String > = Err("file contents was None in load_from_json_file_relative_to_robot_directory".to_string());
                out
            } else {
                let file_contents_string = file_contents_string_option.unwrap();
                let out : Result<$type, String> = Ok(serde_json::from_str(&file_contents_string).unwrap());
                out
            }
        }
    };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
NOTE:
Any functions that use this macro will need

use std::time::{Instant, SystemTime, UNIX_EPOCH};
use thread_id;
use std::ops::DerefMut;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! write_to_recorder_arc_mutex_option {
    ($recorder_arc_mutex_option: expr, $variable: expr, $label: expr) => {
        if !$recorder_arc_mutex_option.is_none() {
            let subtract_time_start = Instant::now();
            let mut recorder_option_mutex_guard = $recorder_arc_mutex_option.recorder_arc_mutex_option.lock().unwrap();
            let mut recorder_option = recorder_option_mutex_guard.deref_mut();
            write_to_recorder_option!(recorder_option, $variable, $label, subtract_time_start);
            drop(recorder_option_mutex_guard);
        }
    };
}

/*
NOTE:
Any functions that use this macro will need

use std::time::{Instant, SystemTime, UNIX_EPOCH};
use thread_id;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! write_to_recorder_option {
    ($recorder_option: expr, $variable: expr, $label: expr, $subtract_time_start: expr) => {
        if $recorder_option.0.is_some() {
            let mut recorder_option_local : &mut RecorderOption = $recorder_option;
            let mut recorder : &mut Recorder = recorder_option_local.0.as_mut().unwrap();
            write_to_recorder!(recorder, $variable, $label, $subtract_time_start);
        }
    };
}

/*
NOTE:
Any functions that use this macro will need

use std::time::{Instant, SystemTime, UNIX_EPOCH};
use thread_id;

within scope.
*/
#[macro_use] #[macro_export]
macro_rules! write_to_recorder {
    ($recorder: expr, $variable: expr, $label: expr, $subtract_time_start: expr) => {
        // let subtract_time_start = Instant::now();

        $recorder.labels.push($label.to_string());

        let start = SystemTime::now();
        let since_the_epoch = start.duration_since(UNIX_EPOCH);
        $recorder.time_stamps.push( since_the_epoch.as_ref().unwrap().as_secs_f64() );

        let variable_type = $crate::utils::utils_recorders::recorder_utils::get_type_of($variable);
        $recorder.variable_types.push(variable_type);
        $recorder.variable_names.push(stringify!($variable).to_string());

        let tid = thread_id::get();
        $recorder.thread_ids.push( tid );
        let adjusted_thread_id = $recorder.get_adjusted_thread_id(tid);
        $recorder.adjusted_thread_ids.push(adjusted_thread_id);

        let serialized = serde_json::to_string($variable).unwrap();
        $recorder.variable_json_strings.push(serialized);

        $recorder.total_subtract_time += $subtract_time_start.elapsed().as_secs_f64();
        let total_subtract_time = $recorder.total_subtract_time;
        $recorder.adjusted_time_stamps.push( since_the_epoch.as_ref().unwrap().as_secs_f64() - total_subtract_time );
        // $recorder.subtract_times.push( subtract_time_start.elapsed().as_secs_f64() );
    };
}

////////////////////////////////////////////////////////////////////////////////////////////////////


pub mod utils;
pub mod robot_modules;
pub mod path_planning;
pub mod prelude;
