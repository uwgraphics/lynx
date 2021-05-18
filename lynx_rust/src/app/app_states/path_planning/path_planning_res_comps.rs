use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use nalgebra::DVector;

pub struct PathPlanningPlaybackPack {
    pub arclength_curr_value: f64,
    pub curr_solution: Option<LinearSplinePath>,
    pub display_playback_path: bool
}

impl PathPlanningPlaybackPack {
    pub fn new() -> Self {
        Self {
            arclength_curr_value: 0.0,
            curr_solution: None,
            display_playback_path: false
        }
    }

    pub fn get_path_mut_ref_and_arclength_curr_value_mut_ref(&mut self) -> Option< (&mut LinearSplinePath, &mut f64) > {
        if self.curr_solution.is_none() { return None; }

        return Some( (self.curr_solution.as_mut().unwrap(), &mut self.arclength_curr_value) );
    }
}

pub struct PathPlanningStartAndGoalStatePack {
    pub start_state: Option<DVector<f64>>,
    pub goal_state: Option<DVector<f64>>
}

impl PathPlanningStartAndGoalStatePack {
    pub fn new() -> Self {
        Self {
            start_state: None,
            goal_state: None
        }
    }
}