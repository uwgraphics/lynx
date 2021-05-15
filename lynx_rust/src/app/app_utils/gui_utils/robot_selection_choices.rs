use crate::utils::utils_files_and_strings::prelude::*;

pub fn get_all_robot_and_configuration_choices() -> Vec<(String, Option<String>)> {
    let mut out_vec = Vec::new();

    let all_robots = get_all_robot_options();
    for r in all_robots {
        out_vec.push((r.clone(), None));
        let all_configuration_choices = _get_all_configuration_choices_for_given_robot(&r);
        for c in all_configuration_choices {
            out_vec.push((r.clone(), Some(c.clone()) ));
        }
    }

    out_vec.sort();
    return out_vec;
}

fn _get_all_configuration_choices_for_given_robot(robot_name: &String) -> Vec<String> {
    let mut out_vec = Vec::new();
    let fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/configurations";
    let path_exists = check_if_path_exists(fp.clone());
    if path_exists {
        let all_files = get_all_files_in_directory(fp.clone());
        for a in all_files {
            out_vec.push(get_filename_without_extension(a));
        }
    }

    return out_vec;
}

pub fn get_all_robot_set_choices() -> Vec<String> {
    let mut out_vec = Vec::new();
    let fp = get_path_to_src() + "/robot_sets";

    let path_exists = check_if_path_exists(fp.clone());
    if path_exists {
        let all_files = get_all_files_in_directory(fp.clone());
        for a in all_files {
            out_vec.push( get_filename_without_extension(a.clone()) );
        }
    }

    out_vec.sort();
    return out_vec;
}