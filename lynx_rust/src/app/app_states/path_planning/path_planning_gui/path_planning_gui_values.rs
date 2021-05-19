use crate::app::app_states::path_planning::path_planners_enum::PathPlannerSelection;

pub struct PathPlanningGUIValues {
    pub start_visible: bool,
    pub goal_visible: bool,
    pub curr_path_planner: PathPlannerSelection,
    pub path_planning_query_save_window_open: bool,
    pub path_planning_query_save_string: String,
    pub path_planning_query_load_window_open: bool,
    pub path_planning_query_load_string: String,
}

impl PathPlanningGUIValues {
    pub fn new() -> Self {
        Self {
            start_visible: true,
            goal_visible: true,
            curr_path_planner: PathPlannerSelection::SPRINT,
            path_planning_query_save_window_open: false,
            path_planning_query_save_string: "".to_string(),
            path_planning_query_load_window_open: false,
            path_planning_query_load_string: "".to_string()
        }
    }
}