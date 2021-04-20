use crate::utils::utils_collisions::collision_multiple_results::*;
use termion::{color, style};

#[derive(Debug, Clone)]
pub struct VecOfIntersectCheckMultipleResult {
    _intersect_check_multiple_results: Vec<IntersectCheckMultipleResult>,
    _labels: Vec<String>
}
impl VecOfIntersectCheckMultipleResult {
    pub fn new_empty() -> Self {
        return Self { _intersect_check_multiple_results: Vec::new(), _labels: Vec::new() }
    }

    pub fn new_no_intersections_found(num: usize, stopped_at_first_detected: bool) -> Self {
        let mut out_vec = Vec::new();
        let mut labels = Vec::new();
        for _ in 0..num {
            out_vec.push( IntersectCheckMultipleResult::NoIntersectionsFound(IntersectionCheckMultipleInfo::new(stopped_at_first_detected)) );
            labels.push("default".to_string());
        }
        return Self { _intersect_check_multiple_results: out_vec, _labels: labels }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_new_result(&mut self, result: IntersectCheckMultipleResult, label: String) {
        self._intersect_check_multiple_results.push(result);
        self._labels.push(label);
    }

    pub fn in_collision(&self) -> bool {
        let mut in_collision = false;
        let l = self._intersect_check_multiple_results.len();
        for i in 0..l {
            match self._intersect_check_multiple_results[i] {
                IntersectCheckMultipleResult::IntersectionFound(_) => { return true; }
                IntersectCheckMultipleResult::NoIntersectionsFound(_) => {}
            }
        }

        return in_collision;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_intersect_check_multiple_results_ref(&self) -> &Vec<IntersectCheckMultipleResult> {
        return &self._intersect_check_multiple_results;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let mut count = 0;
        for r in &self._intersect_check_multiple_results {
            println!("{}{}Result {:?}, {} ---> {}", style::Bold, color::Fg(color::Magenta), count, self._labels[count], style::Reset);
            r.print_summary();
            count += 1;
        }
    }
}


#[derive(Debug, Clone)]
pub struct VecOfDistanceCheckMultipleResult {
    _distance_check_multiple_results: Vec<DistanceCheckMultipleResult>,
    _labels: Vec<String>
}
impl VecOfDistanceCheckMultipleResult {
    pub fn new_empty() -> Self {
        return Self { _distance_check_multiple_results: Vec::new(), _labels: Vec::new() }
    }

    pub fn new_no_intersections_found(num: usize, stopped_at_first_detected: bool) -> Self {
        let mut out_vec = Vec::new();
        let mut labels = Vec::new();
        for _ in 0..num {
            out_vec.push( DistanceCheckMultipleResult::NoIntersectionsFound(DistanceCheckMultipleInfo::new(stopped_at_first_detected)) );
            labels.push("default".to_string());
        }
        return Self { _distance_check_multiple_results: out_vec, _labels: labels }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_new_result(&mut self, result: DistanceCheckMultipleResult, label: String) {
        self._distance_check_multiple_results.push(result);
        self._labels.push(label);
    }

    pub fn in_collision(&self) -> bool {
        let mut in_collision = false;
        let l = self._distance_check_multiple_results.len();
        for i in 0..l {
            match self._distance_check_multiple_results[i] {
                DistanceCheckMultipleResult::IntersectionFound(_) => { return true; }
                DistanceCheckMultipleResult::NoIntersectionsFound(_) => {}
            }
        }

        return in_collision;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_distance_check_multiple_results_ref(&self) -> &Vec<DistanceCheckMultipleResult> {
        return &self._distance_check_multiple_results;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let mut count = 0;
        for r in &self._distance_check_multiple_results {
            println!("{}{}Result {:?}, {} ---> {}", style::Bold, color::Fg(color::Magenta), count, self._labels[count], style::Reset);
            r.print_summary();
            count += 1;
        }
    }

}


#[derive(Debug, Clone)]
pub struct VecOfContactCheckMultipleResult {
    _contact_check_multiple_results: Vec<ContactCheckMultipleResult>,
    _labels: Vec<String>
}
impl VecOfContactCheckMultipleResult {
    pub fn new_empty() -> Self {
        return Self { _contact_check_multiple_results: Vec::new(), _labels: Vec::new() }
    }

    pub fn new_no_intersections_found(num: usize, stopped_at_first_detected: bool, margin: Option<f64>) -> Self {
        let mut out_vec = Vec::new();
        let mut labels = Vec::new();
        for _ in 0..num {
            out_vec.push( ContactCheckMultipleResult::NoIntersectionsFound(ContactCheckMultipleInfo::new(stopped_at_first_detected, margin)) );
            labels.push("default".to_string());
        }
        return Self { _contact_check_multiple_results: out_vec, _labels: labels }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_new_result(&mut self, result: ContactCheckMultipleResult, label: String) {
        self._contact_check_multiple_results.push(result);
        self._labels.push(label);
    }

    pub fn in_collision(&self) -> bool {
        let mut in_collision = false;
        let l = self._contact_check_multiple_results.len();
        for i in 0..l {
            match self._contact_check_multiple_results[i] {
                ContactCheckMultipleResult::IntersectionFound(_) => { return true; }
                ContactCheckMultipleResult::NoIntersectionsFound(_) => {}
            }
        }

        return in_collision;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_distance_check_multiple_results_ref(&self) -> &Vec<ContactCheckMultipleResult> {
        return &self._contact_check_multiple_results;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let mut count = 0;
        for r in &self._contact_check_multiple_results {
            println!("{}{}Result {:?}, {} ---> {}", style::Bold, color::Fg(color::Magenta), count, self._labels[count], style::Reset);
            r.print_summary();
            count += 1;
        }
    }
}