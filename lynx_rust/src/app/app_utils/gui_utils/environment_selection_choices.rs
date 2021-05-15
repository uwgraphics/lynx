use crate::utils::utils_files_and_strings::prelude::*;

pub fn get_all_environment_choices() -> Vec<String> {
    let fp = get_path_to_src() + "/assets/mesh_environments";
    let mut a = get_all_files_in_directory(fp);
    a.sort();
    return a;
}