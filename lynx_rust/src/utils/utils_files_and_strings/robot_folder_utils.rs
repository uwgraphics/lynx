use crate::utils::utils_files_and_strings::file_utils::*;
use termion::{color, style};
use std::fs;

pub fn get_all_robot_options() -> Vec<String> {
    let fp = get_path_to_robots_folder();
    let all_robots = get_all_files_in_directory(fp);
    return all_robots
}

pub fn check_if_robot_is_valid_choice(robot_name: &str) -> bool {
    let fp = get_path_to_src() + "/robots/" + robot_name;
    let path_exists = check_if_path_exists(fp);
    return path_exists;
}

pub fn robot_directory_includes_urdf(robot_name: String) -> bool {
    let fp = get_path_to_robots_folder() + robot_name.as_str();

    let all_files = get_all_files_in_directory_with_given_extension(fp, "urdf".to_string());

    if all_files.len() > 0 { return true; }
    else { return false; }
}

pub fn robot_directory_includes_base_meshes(robot_name: String) -> bool {
    let fp = get_path_to_robots_folder() + robot_name.as_str();

    let all_files = get_all_files_in_directory(fp.clone());

    return all_files.contains(&"base_meshes".to_string());
}

pub fn robot_directory_includes_convex_subcomponents(robot_name: String) -> bool {
    let fp = get_path_to_robots_folder() + robot_name.as_str() + "/autogenerated_metadata";

    let all_files = get_all_files_in_directory(fp.clone());

    return all_files.contains(&"link_convex_subcomponents".to_string());
}

pub fn get_path_to_urdf(robot_name: String) -> Option<String> {
    let fp = get_path_to_robots_folder() + robot_name.as_str();

    let all_files = get_all_files_in_directory_with_given_extension(fp, "urdf".to_string());

    if all_files.len() > 0 {
        return Some( get_path_to_robots_folder() + robot_name.as_str() + "/" + all_files[0].as_str() );
    } else {
        return None;
    }
}

pub fn get_path_to_particular_robot_directory(robot_name: String) -> String {
    return get_path_to_robots_folder() + robot_name.as_str();
}

pub fn create_directories_recursively_relative_to_robot_directory(robot_name: String, partial_fp: String) {
    // no need to start fp with slash
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    create_directories_recursively(full_fp);
}

pub fn check_if_path_exists_relative_to_robot_directory(robot_name: String, partial_fp: String) -> bool {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return check_if_path_exists(full_fp);
}

pub fn check_if_file1_modified_after_file2_relative_to_robot_directory(robot_name: String, partial_fp1: String, partial_fp2: String) -> Option<bool> {
    let full_fp1 = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp1.as_str();
    let full_fp2 = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp2.as_str();
    return check_if_file1_modified_after_file2(full_fp1, full_fp2);
}

pub fn check_when_file_was_last_modified_in_seconds_relative_to_robot_directory(robot_name: String, partial_fp: String) -> Option<f64> {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return check_when_file_was_last_modified_in_seconds(full_fp);
}

pub fn delete_directory_all_relative_to_robot_directory(robot_name: String, partial_fp: String) {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    fs::remove_dir_all(full_fp);
}

pub fn get_all_files_in_directory_relative_to_robot_directory(robot_name: String, partial_fp: String) -> Vec<String> {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return get_all_files_in_directory(full_fp);
}

pub fn get_all_files_in_directory_with_extension_relative_to_robot_directory(robot_name: String, partial_fp: String, extension: String) -> Vec<String> {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return get_all_files_in_directory_with_given_extension(full_fp, extension);
}

pub fn get_all_files_in_directory_that_include_a_given_substring_relative_to_robot_directory(robot_name: String, partial_fp: String, substring: String) -> Vec<String> {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return get_all_files_in_directory_that_include_a_given_substring(full_fp, substring);
}

pub fn get_all_files_in_directory_with_extension_that_include_a_given_substring_relative_to_robot_directory(robot_name: String, partial_fp: String, substring: String, extension: String) -> Vec<String> {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return get_all_files_in_directory_with_extension_that_include_a_given_substring(full_fp, substring, extension);
}

pub fn get_all_files_in_directory_with_given_base_string_relative_to_robot_directory(robot_name: String, partial_fp: String, basestring: String) -> Vec<String> {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return get_all_files_in_directory_with_given_base_string(full_fp, basestring);
}

pub fn get_all_files_in_directory_with_extension_with_given_base_string_relative_to_robot_directory(robot_name: String, partial_fp: String, basestring: String, extension: String) -> Vec<String> {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return get_all_files_in_directory_with_extension_with_given_base_string(full_fp, basestring, extension);
}

pub fn read_file_contents_relative_to_robot_directory(robot_name: String, partial_fp: String) -> Option<String> {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp.as_str();
    return read_file_contents(full_fp);
}

pub fn read_file_contents_separated_args_relative_to_robot_directory(robot_name: String, partial_fp_to_dir: String, file_name: String) -> Option<String> {
    let partial_fp = partial_fp_to_dir + "/" + file_name.as_str();
    return read_file_contents_relative_to_robot_directory(robot_name, partial_fp);
}

pub fn write_string_to_file_relative_to_robot_directory(robot_name: String, partial_fp_to_dir: String, file_name: String, out_string: String, create_any_necessary_directories: bool) {
    let full_fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/" + partial_fp_to_dir.as_str();
    write_string_to_file(full_fp, file_name, out_string, create_any_necessary_directories);
}

pub fn check_when_configuration_file_was_last_modified_in_seconds(robot_name: String, configuration_name: String) -> Option<f64> {
        if configuration_name == "baseconfig".to_string()  { return None; }

        let partial_fp = "autogenerated_metadata/configurations/".to_string() + configuration_name.as_str() + ".json";

        return check_when_file_was_last_modified_in_seconds_relative_to_robot_directory(robot_name.to_string(), partial_fp);
    }

pub fn was_configuration_file_just_modified(robot_name: String, configuration_name: String) -> bool {
    let res = check_when_configuration_file_was_last_modified_in_seconds(robot_name, configuration_name);
    if res.is_none() { return false; }
    let t = res.unwrap();
    if t < 0.8 { return true; }
    else { return false; }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
