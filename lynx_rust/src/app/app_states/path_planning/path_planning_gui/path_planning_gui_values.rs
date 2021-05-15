use crate::app::app_states::path_planning::path_planners_enum::PathPlannerSelection;

pub struct PathPlanningGUIValues {
    pub start_visible: bool,
    pub goal_visible: bool,
    pub curr_path_planner: PathPlannerSelection
}

impl PathPlanningGUIValues {
    pub fn new() -> Self {
        Self {
            start_visible: true,
            goal_visible: false,
            curr_path_planner: PathPlannerSelection::SPRINT
        }
    }
}