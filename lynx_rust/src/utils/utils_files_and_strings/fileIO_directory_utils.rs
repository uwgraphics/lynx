// MAY NEED TO CLEAN THIS UP

use crate::utils::utils_files_and_strings::file_utils::*;
use nalgebra::DVector;
use crate::utils::utils_files_and_strings::string_utils::float_to_string;

pub fn output_dvec_path_to_file(path: &Vec<DVector<f64>>, solver: &str, robot: &str, file_name: &str) {
    let mut out_str = "".to_string();
    let l = path.len();
    for i in 0..l {
            let l2 = path[i].len();
            for j in 0..l2-1 {
                out_str += format!("{:?},", path[i][j]).as_str();
            }
            out_str += format!("{:?}", path[i][l2-1]).as_str();
            out_str += "\n";
        }

    let fp = get_path_to_src() + "/fileIO/path_outputs/" + solver + "/" + robot;

    write_string_to_file(fp, file_name.to_string(), out_str, true);
}

pub fn output_multiple_dvec_paths_to_file(paths: Vec<&Vec<DVector<f64>>>, solver: &str, robot: &str, file_name: &str) {
    let mut combined_path = Vec::new();

    let l = paths.len();
    for i in 0..l {
        let l2 = paths[i].len();
        for j in 0..l2 {
            combined_path.push( paths[i][j].clone() );
        }
    }

    output_dvec_path_to_file(&combined_path, solver, robot, file_name);
}

pub fn output_path_optimization_solution_cost_timeline_to_file(best_cost_values_chronological_order: &Vec<f64>, best_cost_values_chronological_order_timestamps: &Vec<f64>, solver: &str, robot: &str, file_name: &str) {
    let mut out_str = "".to_string();

    let l = best_cost_values_chronological_order.len();
    for i in 0..l {
        out_str += float_to_string(best_cost_values_chronological_order[i]).as_str();
        out_str += ",";
        out_str += float_to_string(best_cost_values_chronological_order_timestamps[i]).as_str();
        out_str += "\n";
    }

    let fp = get_path_to_src() + "/fileIO/path_optimization_cost_timeline_outputs/" + solver + "/" + robot;
    write_string_to_file(fp, file_name.to_string(), out_str, true);
}