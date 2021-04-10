use crate::utils::utils_paths::linear_spline_path::LinearSplinePath;
use crate::utils::utils_files_and_strings::fileIO_directory_utils::*;

#[derive(Clone, Debug)]
pub enum PathPlannerResult {
    SolutionFound(LinearSplinePath),
    SolutionNotFoundButPartialSolutionReturned(LinearSplinePath),
    SolutionNotFound(String)
}

impl PathPlannerResult {
    pub fn output_to_file(&self, solver: &str, robot: &str, file_name: &str) {
        match self {
            PathPlannerResult::SolutionFound(s) => { output_dvec_path_to_file(&s.waypoints, solver, robot, file_name) }
            PathPlannerResult::SolutionNotFoundButPartialSolutionReturned(s) => { output_dvec_path_to_file(&s.waypoints, solver, robot, file_name) }
            PathPlannerResult::SolutionNotFound(_) => { println!("PathPlannerResult contained no solution, so nothing outputted to file.") }
        }
    }
}

