use crate::app::app_states::app_states_enum::AppState;
use crate::prelude::LinkGeometryType;

pub struct CurrentMainGUIValues {
    pub curr_selected_robot_and_configuration: Option<(String, Option<String>)>,
    pub curr_selected_robot_set: Option<String>,
    pub curr_selected_environment: Option<String>,
    pub save_environment_text: String,
    pub environment_selectable: bool,
    pub save_environment_window_open: bool,
    pub hide_main_gui: bool,
    pub hide_application_gui: bool,
    pub curr_app_state: Option<AppState>,
    pub display_self_collision_information: bool,
    pub display_multi_robot_collision_information: bool,
    pub display_environment_collision_information: bool,
    pub display_collision_information: bool,
    pub save_joint_state_window_open: bool,
    pub save_joint_state_string: String,
    pub load_joint_state_window_open: bool,
    pub load_joint_state_string: String
}
impl CurrentMainGUIValues {
    pub fn new() -> Self {
        Self {
            curr_selected_robot_and_configuration: None,
            curr_selected_robot_set: None,
            curr_selected_environment: None,
            save_environment_text: "".to_string(),
            environment_selectable: true,
            save_environment_window_open: false,
            hide_main_gui: false,
            hide_application_gui: false,
            curr_app_state: None,
            display_self_collision_information: false,
            display_multi_robot_collision_information: false,
            display_environment_collision_information: false,
            display_collision_information: false,
            save_joint_state_window_open: false,
            save_joint_state_string: "".to_string(),
            load_joint_state_window_open: false,
            load_joint_state_string: "".to_string()
        }
    }
}




