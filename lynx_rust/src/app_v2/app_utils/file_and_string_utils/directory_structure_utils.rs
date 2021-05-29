

pub fn get_file_path_from_dummy_assets_to_src() -> String {
    return "../../".to_string();
}

pub fn get_file_path_from_dummy_assets_to_robot_dir(robot_name: &String) -> String {
    return get_file_path_from_dummy_assets_to_src() + "robots/" + robot_name.as_str();
}

pub fn get_file_path_from_dummy_assets_to_mesh_environment_dir(environment_name: &String) -> String {
    return get_file_path_from_dummy_assets_to_src() + "assets/mesh_environments/" + environment_name.as_str();
}