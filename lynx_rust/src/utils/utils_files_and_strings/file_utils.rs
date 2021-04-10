use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::fs::read_dir;
use std::path::Path;
use termion::{color, style};
use std::fs;
use crate::utils::utils_files_and_strings::string_utils::*;


pub fn get_path_to_src() -> String {
    let path = env::current_dir().unwrap();
    let s = path.to_str().unwrap();
    let s1 = String::from(s);
    let path_to_src = s1 + "/../";
    path_to_src
}

pub fn read_file_contents(fp: String) -> Option<String> {
    let mut file = File::open(fp.as_str());
    if file.is_err() { return None; }
    let mut contents = String::new();
    let res = file.unwrap().read_to_string(&mut contents);
    if res.is_err() { return None; }
    Some( contents )
}

pub fn read_file_contents_separated_args(fp_to_dir: String, file_name: String) -> Option<String> {
    let fp = fp_to_dir + "/" + file_name.as_str();
    return read_file_contents(fp);
}

pub fn write_string_to_file(fp_to_dir: String, file_name: String, out_string: String, create_any_necessary_directories: bool) {
    if create_any_necessary_directories {
        create_directories_recursively(fp_to_dir.clone());
    }
    let fp = fp_to_dir + "/" + file_name.as_str();
    let mut file = File::create(fp.clone());
    if file.is_err() { return; }
    file.unwrap().write(out_string.as_bytes());
}

pub fn get_all_files_in_directory(fp: String) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    let it = read_dir(fp.clone());
    if it.is_err() {
        println!("{}{}WARNING: filepath {} does not exist{}", color::Fg(color::Yellow), style::Bold, fp, style::Reset);
        return out;
    }
    for i in it.unwrap() {
        out.push(i.unwrap().file_name().into_string().unwrap());
    }
    out
}

pub fn get_all_files_in_directory_with_given_extension(fp: String, extension: String) -> Vec<String> {
    /* do not include dot in extension */
    let mut out_files = Vec::new();

    let all_files = get_all_files_in_directory(fp.clone());
    let l = all_files.len();
    for i in 0..l {
        let ext = Path::new(&all_files[i]).extension();
        if ext.is_some() {
            if ext.unwrap() == extension.as_str() {
                out_files.push( all_files[i].clone() );
            }
        }
    }

    out_files
}

pub fn get_path_to_robots_folder() -> String {
    return get_path_to_src() + "robots/";
}

pub fn recover_filename_from_full_path(fp: String) -> Option<String> {
    let split: Vec<&str> = fp.split("/").collect();
    let l = split.len();
    if l == 0 { return None; }

    return Some( split[l-1].to_string() );
}

pub fn get_filename_without_extension(filename: String) -> String {
    let split: Vec<&str> = filename.split(".").collect();
    return split[0].to_string();
}

pub fn create_directories_recursively(fp: String) {
    // will create all new directories, even if some don't already exist along the way
    fs::create_dir_all(fp);
}

pub fn check_if_path_exists(fp: String) -> bool {
    return Path::new(fp.as_str()).exists() ;
}

pub fn check_if_file1_modified_after_file2(fp1: String, fp2: String) -> Option<bool> {
    // if one or both of the files do not exist, returns None

    let metadata1 = fs::metadata(fp1.clone());
    let metadata2 = fs::metadata(fp2.clone());

    if metadata1.is_err() || metadata2.is_err() { return None; }

    if metadata1.unwrap().modified().unwrap().lt(   &metadata2.unwrap().modified().unwrap()  ) {
        return Some(false);
    } else {
        return Some(true);
    }
}

pub fn check_when_file_was_last_modified_in_seconds(fp: String) -> Option<f64> {
    let metadata = fs::metadata(fp.clone());

    if metadata.is_err() { return None; }

    let t = metadata.unwrap().modified().unwrap().elapsed();
    if t.is_err() { return None; }

    return Some(t.ok().unwrap().as_secs_f64());
}

pub fn delete_directory_all(fp: String) {
    fs::remove_dir_all(fp);
}

pub fn get_all_files_in_directory_that_include_a_given_substring(fp: String, substring: String) -> Vec<String> {
    let mut out_vec = Vec::new();

    let all_files = get_all_files_in_directory(fp.clone());

    let l = all_files.len();
    for i in 0..l {
        if all_files[i].contains(&substring) {
            out_vec.push( all_files[i].clone() );
        }
    }

    return out_vec;
}

pub fn get_all_files_in_directory_with_extension_that_include_a_given_substring(fp: String, substring: String, extension: String) -> Vec<String> {
    let mut first_vec = Vec::new();

    let all_files = get_all_files_in_directory(fp.clone());

    let l = all_files.len();
    for i in 0..l {
        if all_files[i].contains(&substring) {
            first_vec.push( all_files[i].clone() );
        }
    }

    let mut out_vec = Vec::new();

    let l = first_vec.len();
    for i in 0..l {
        let ext = Path::new(&first_vec[i]).extension();
        if ext.is_some() {
            if ext.unwrap() == extension.as_str() {
                out_vec.push( first_vec[i].clone() );
            }
        }
    }

    return out_vec;
}

pub fn get_all_files_in_directory_with_given_base_string(fp: String, basestring: String) -> Vec<String> {
    let mut out_vec = Vec::new();

    let all_files = get_all_files_in_directory(fp.clone());

    let l = all_files.len();
    for i in 0..l {
        let split: Vec<&str> = all_files[i].split("_").collect();
        let l2 = split.len();
        let mut check_string_ = split[0].to_string();

        if l2 > 1 {
            check_string_ = "".to_string();
            for j in 0..(l2 - 1) {
                check_string_ += split[j];
                if !(j == l2 - 2) {
                    check_string_ += "_";
                }
            }
        }

        let check_string = get_filename_without_extension(check_string_);
        if check_string == basestring.clone() {
            out_vec.push(all_files[i].clone());
        }
    }

    return out_vec;
}

pub fn get_all_files_in_directory_with_extension_with_given_base_string(fp: String, basestring: String, extension: String) -> Vec<String> {
    let mut out_vec = Vec::new();

    let res = get_all_files_in_directory_with_given_base_string(fp.clone(), basestring.clone());
    let l = res.len();
    for i in 0..l {
        let ext = Path::new(&res[i]).extension();
        if ext.is_some() {
            if ext.unwrap().to_str().unwrap().to_string() == extension.clone() {
                out_vec.push(res[i].clone());
            }
        }
    }

    return out_vec;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
