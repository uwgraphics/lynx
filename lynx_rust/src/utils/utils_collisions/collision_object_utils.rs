use crate::utils::utils_collisions::collision_object::*;
use crate::utils::utils_collisions::collision_check_tensor::*;
use crate::utils::utils_recorders::stopwatch::Stopwatch;
use nalgebra::{Point3, Vector3};
use ncollide3d::query::{Proximity, PointQuery, Contact, Ray, RayCast, PointProjection};
use std::time::{Instant, Duration};
use std::fmt;


#[derive(Debug, Clone)]
pub enum IntersectCheckMultipleResult {
    IntersectionFound(IntersectionCheckMultipleInfo),
    NoIntersectionsFound(IntersectionCheckMultipleInfo)
}

impl IntersectCheckMultipleResult {
    pub fn print_summary(&self) {
        match self {
            IntersectCheckMultipleResult::IntersectionFound(c) => {
                println!("Intersection Found");
                c._stopwatch.print_summary();
                for i in 0..c._intersection_names.len() {
                    println!("collision {:?} ---> {:?}, {:?} stopped at first detected: {:?}", i, c._intersection_names[i], c._intersection_idxs[i], c._stopped_at_first_detected);
                }
            },
            IntersectCheckMultipleResult::NoIntersectionsFound(c) => {
                println!("Intersection Not Found");
                c._stopwatch.print_summary();
                for i in 0..c._intersection_names.len() {
                    println!("collision {:?} ---> {:?}, {:?}", i, c._intersection_names[i], c._intersection_idxs[i]);
                }
            }
        }
        println!();
    }

    pub fn get_intersect_check_multiple_info_ref(&self) -> &IntersectionCheckMultipleInfo {
        match self {
            IntersectCheckMultipleResult::IntersectionFound(info) => return info,
            IntersectCheckMultipleResult::NoIntersectionsFound(info) => return info
        }
    }

    pub fn is_in_collision(&self) -> bool {
            return match self {
                IntersectCheckMultipleResult::NoIntersectionsFound(_) => { false }
                IntersectCheckMultipleResult::IntersectionFound(_) => { true }
            }
        }
}

#[derive(Debug, Clone)]
pub struct IntersectionCheckMultipleInfo {
    _intersection_names: Vec< [String; 2] >,
    _intersection_idxs: Vec<  [ [usize; 2]; 2 ]  >,
    _stopped_at_first_detected: bool,
    _stopwatch: Stopwatch
}

impl IntersectionCheckMultipleInfo {
    pub fn new(stopped_at_first_detected: bool) -> Self {
        return Self{ _intersection_names: Vec::new(), _intersection_idxs: Vec::new(), _stopped_at_first_detected: stopped_at_first_detected, _stopwatch: Stopwatch::new() };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_intersection(&mut self, collision_obj_1_name: String, collision_obj_1_idxs: [usize; 2], collision_obj_2_name: String, collision_obj_2_idxs: [usize; 2] ) {
        self._intersection_names.push( [collision_obj_1_name, collision_obj_2_name] );
        self._intersection_idxs.push(  [ collision_obj_1_idxs, collision_obj_2_idxs ] );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_intersection_names(&self) -> &Vec< [String; 2] > {
        return &self._intersection_names;
    }

    pub fn get_intersection_idxs(&self) -> &Vec<  [ [usize; 2]; 2 ]  > {
        return &self._intersection_idxs;
    }

    pub fn get_stopwatch(&self) -> &Stopwatch {
        return &self._stopwatch;
    }
}

pub fn intersect_check_between_multiple_collision_objects(group1: &Vec<Vec<CollisionObject>>, group2: &Vec<Vec<CollisionObject>>, stop_at_first_detected: bool, skip_collision_check_tensor: Option<&BoolCollisionCheckTensor>) -> Result<IntersectCheckMultipleResult, String> {
    let check_for_collision_skips = skip_collision_check_tensor.is_some();
    let mut intersections_info = IntersectionCheckMultipleInfo::new(stop_at_first_detected);

    let l1 = group1.len();
    let l3 = group2.len();

    for i in 0..l1 {
        let l2 = group1[i].len();
        for j in 0..l2 {
            for k in 0..l3 {
                let l4 = group2[k].len();
                for l in 0..l4 {
                    if !check_for_collision_skips || !skip_collision_check_tensor.as_ref().unwrap().get_is_skip([i, j], [k, l])? {
                        if group1[i][j].active && group2[k][l].active {
                            if group1[i][j].bounding_aabb_outdated { return Err("bounding aabb was outdated in intersect_check_between_multiple_collision_objects".to_string()); }

                            if group2[k][l].bounding_aabb_outdated { return Err("bounding aabb was outdated in intersect_check_between_multiple_collision_objects".to_string()); }

                            let bounding_aabb_intersect_check = group1[i][j].intersect_check_bounding_aabb_immutable(&group2[k][l]);
                            if bounding_aabb_intersect_check {
                                let intersect_check = group1[i][j].intersect_check(&group2[k][l]);
                                if intersect_check {
                                    intersections_info.add_intersection(group1[i][j].name.clone(), [i, j], group2[k][l].name.clone(), [k, l]);
                                    if stop_at_first_detected {
                                        intersections_info._stopwatch.add_time_marker(None);
                                        return Ok(IntersectCheckMultipleResult::IntersectionFound(intersections_info));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    intersections_info._stopwatch.add_time_marker(None);
    if intersections_info.get_intersection_names().is_empty() {
        return Ok( IntersectCheckMultipleResult::NoIntersectionsFound( intersections_info ) );
    } else {
        return Ok( IntersectCheckMultipleResult::IntersectionFound( intersections_info ) );
    }
}

pub fn intersect_check_between_multiple_collision_objects_subset(subset_check_idxs: &Vec<  [ [usize; 2]; 2 ]  >, group1: &Vec<Vec<CollisionObject>>, group2: &Vec<Vec<CollisionObject>>, stop_at_first_detected: bool, skip_collision_check_tensor: Option<&BoolCollisionCheckTensor>) -> Result<IntersectCheckMultipleResult, String> {
    let check_for_collision_skips = skip_collision_check_tensor.is_some();
    let mut intersections_info = IntersectionCheckMultipleInfo::new(stop_at_first_detected);

    for c in subset_check_idxs {
        if group1[c[0][0]][c[0][1]].active && group2[c[1][0]][c[1][1]].active {
            if group1[c[0][0]][c[0][1]].bounding_aabb_outdated { return Err("bounding aabb was outdated in intersect_check_between_multiple_collision_objects".to_string()); }

            if group2[c[1][0]][c[1][1]].bounding_aabb_outdated { return Err("bounding aabb was outdated in intersect_check_between_multiple_collision_objects".to_string()); }

            let bounding_aabb_intersect_check = group1[c[0][0]][c[0][1]].intersect_check_bounding_aabb_immutable(&group2[c[1][0]][c[1][1]]);
            if bounding_aabb_intersect_check {
                let intersect_check = group1[c[0][0]][c[0][1]].intersect_check(&group2[c[1][0]][c[1][1]]);
                if intersect_check {
                    intersections_info.add_intersection(group1[c[0][0]][c[0][1]].name.clone(), c[0], group2[c[1][0]][c[1][1]].name.clone(), c[1]);
                    if stop_at_first_detected {
                        intersections_info._stopwatch.add_time_marker(None);
                        return Ok(IntersectCheckMultipleResult::IntersectionFound(intersections_info));
                    }
                }
            }
        }
    }

    intersections_info._stopwatch.add_time_marker(None);

    if intersections_info.get_intersection_names().is_empty() {
        return Ok( IntersectCheckMultipleResult::NoIntersectionsFound( intersections_info ) );
    } else {
        return Ok( IntersectCheckMultipleResult::IntersectionFound( intersections_info ) );
    }
}



#[derive(Debug, Clone)]
pub enum DistanceCheckMultipleResult {
    IntersectionFound(DistanceCheckMultipleInfo),
    NoIntersectionsFound(DistanceCheckMultipleInfo)
}

impl DistanceCheckMultipleResult {
    pub fn print_summary(&self) {
        match self {
            DistanceCheckMultipleResult::IntersectionFound(c) => {
                println!("Intersection Found");
                c._stopwatch.print_summary();
                for i in 0..c._distance_check_names.len() {
                    println!("pair {:?} ---> {:?}, {:?}, {:?}, {:?}", i, c._distance_check_names[i], c._distance_check_idxs[i], c._distance_check_distance_ratio_with_respect_to_average[i], c._distance_check_distances[i]);
                }
            },
            DistanceCheckMultipleResult::NoIntersectionsFound(c) => {
                println!("Intersection Not Found");
                c._stopwatch.print_summary();
                for i in 0..c._distance_check_names.len() {
                    println!("pair {:?} ---> {:?}, {:?}, {:?}, {:?}", i, c._distance_check_names[i], c._distance_check_idxs[i], c._distance_check_distance_ratio_with_respect_to_average[i], c._distance_check_distances[i]);
                }
            }
        }
        println!();
    }

    pub fn get_closest_n_distance_idxs(&self, n: usize, maximum_distance: f64) -> Vec< [ [usize; 2]; 2 ] > {
        match self {
            DistanceCheckMultipleResult::NoIntersectionsFound(info) => return info.get_closest_n_distance_idxs(n, maximum_distance),
            DistanceCheckMultipleResult::IntersectionFound(info) => return info.get_closest_n_distance_idxs(n, maximum_distance)
        }
    }

    pub fn get_distance_check_multiple_info_ref(&self) -> &DistanceCheckMultipleInfo {
        match self {
            DistanceCheckMultipleResult::IntersectionFound(info) => return info,
            DistanceCheckMultipleResult::NoIntersectionsFound(info) => return info
        }
    }

    pub fn is_in_collision(&self) -> bool {
        return match self {
            DistanceCheckMultipleResult::NoIntersectionsFound(_) => { false }
            DistanceCheckMultipleResult::IntersectionFound(_) => { true }
        }
    }
}

#[derive(Debug, Clone)]
pub struct DistanceCheckMultipleInfo {
    _distance_check_names:  Vec< [String; 2] >,
    _distance_check_idxs: Vec<  [ [usize; 2]; 2 ]  >,
    _distance_check_distances: Vec<f64>,
    _distance_check_distance_ratio_with_respect_to_average: Vec<f64>,
    _num_intersections: usize,
    _stopped_at_first_detected_intersection: bool,
    _stopwatch: Stopwatch
}

impl DistanceCheckMultipleInfo {
    pub fn new(set_stopped_at_first_detected_intersection: bool) -> Self {
        return Self { _distance_check_names: Vec::new(), _distance_check_idxs: Vec::new(),
            _distance_check_distances: Vec::new(), _distance_check_distance_ratio_with_respect_to_average: Vec::new(), _num_intersections: 0,
            _stopped_at_first_detected_intersection: set_stopped_at_first_detected_intersection, _stopwatch: Stopwatch::new() };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_distance(&mut self, collision_obj_1_name: String, collision_obj_1_idxs: [usize; 2], collision_obj_2_name: String, collision_obj_2_idxs: [usize; 2], distance: f64, average_distance_tensor: Option<&FloatCollisionCheckTensor>) -> Result<(), String> {
        let mut distance_ratio_with_respect_to_average = distance;
        if average_distance_tensor.is_some() {
            distance_ratio_with_respect_to_average = distance / average_distance_tensor.as_ref().unwrap().get_mean_at_given_idxs(collision_obj_1_idxs, collision_obj_2_idxs)?;
        }

        let add_idx_res = self._distance_check_distance_ratio_with_respect_to_average.binary_search_by(|x| x.partial_cmp(&distance_ratio_with_respect_to_average).unwrap());
        match add_idx_res {
            Ok(i) => {
                self._distance_check_names.insert(i, [collision_obj_1_name, collision_obj_2_name]);
                self._distance_check_idxs.insert(i, [ collision_obj_1_idxs, collision_obj_2_idxs ]);
                self._distance_check_distances.insert(i, distance);
                self._distance_check_distance_ratio_with_respect_to_average.insert(i, distance_ratio_with_respect_to_average);
            },
            Err(i) => {
                self._distance_check_names.insert(i, [collision_obj_1_name, collision_obj_2_name]);
                self._distance_check_idxs.insert(i, [ collision_obj_1_idxs, collision_obj_2_idxs ]);
                self._distance_check_distances.insert(i, distance);
                self._distance_check_distance_ratio_with_respect_to_average.insert(i, distance_ratio_with_respect_to_average);
            }
        }
        if distance == 0.0 { self._num_intersections += 1 }

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_distance_check_names(&self) -> &Vec< [String; 2] > {
        return &self._distance_check_names;
    }

    pub fn get_distance_check_idxs(&self) -> &Vec<  [ [usize; 2]; 2 ]  > {
        return &self._distance_check_idxs;
    }

    pub fn get_distance_check_idxs_less_than_margin_distance(&self, margin: f64) -> Vec<  [ [usize; 2]; 2 ]  > {
        if self.get_distance_check_idxs().len() == 0 { return Vec::new(); }

        let mut out_vec = self._distance_check_idxs.clone();

        let margin_binary_search_res = self._distance_check_distances.binary_search_by(|x| margin.partial_cmp(x).unwrap() );
        match margin_binary_search_res {
            Ok(i) => { out_vec.truncate(i); return out_vec; },
            Err(i) => { out_vec.truncate(i); return out_vec; }
        }
    }

    pub fn get_distance_check_distances(&self) -> &Vec< f64  > {
        return &self._distance_check_distances;
    }

    pub fn get_num_intersection(&self) -> usize {
        return self._num_intersections;
    }

    pub fn get_closest_n_distance_idxs(&self, n: usize, maximum_distance: f64) -> Vec< [ [usize; 2]; 2 ] > {
        let mut out_vec = Vec::new();

        let l = self._distance_check_distances.len();
        for i in 0..l {
            if i >= n { break; }
            if self._distance_check_distances[i] > maximum_distance { break; }

            out_vec.push( self._distance_check_idxs[i] );
        }

        out_vec
    }

    pub fn get_stopwatch(&self) -> &Stopwatch {
        return &self._stopwatch;
    }
}

pub fn distance_check_between_multiple_collision_objects(group1: &Vec<Vec<CollisionObject>>, group2: &Vec<Vec<CollisionObject>>, stop_at_first_detected_intersection: bool, skip_collision_check_tensor: Option<&BoolCollisionCheckTensor>, average_distance_tensor: Option<&FloatCollisionCheckTensor>) -> Result<DistanceCheckMultipleResult, String> {
    let check_for_collision_skips = skip_collision_check_tensor.is_some();
    let mut distance_check_multiple_info = DistanceCheckMultipleInfo::new(stop_at_first_detected_intersection);

    let l1 = group1.len();
    let l3 = group2.len();

    for i in 0..l1 {
        let l2 = group1[i].len();
        for j in 0..l2 {
            for k in 0..l3 {
                let l4 = group2[k].len();
                for l in 0..l4 {
                    if !check_for_collision_skips || !skip_collision_check_tensor.as_ref().unwrap().get_is_skip([i, j], [k, l])? {
                        if group1[i][j].active && group2[k][l].active {
                            let distance = group1[i][j].distance_check(&group2[k][l]);
                            distance_check_multiple_info.add_distance(group1[i][j].name.clone(), [i, j], group2[k][l].name.clone(), [k, l], distance, average_distance_tensor);
                            if distance == 0.0 && stop_at_first_detected_intersection {
                                distance_check_multiple_info._stopwatch.add_time_marker(None);
                                return Ok(DistanceCheckMultipleResult::IntersectionFound(distance_check_multiple_info));
                            }
                        }
                    }
                }
            }
        }
    }

    distance_check_multiple_info._stopwatch.add_time_marker(None);
    if distance_check_multiple_info.get_num_intersection() == 0 {
        return Ok( DistanceCheckMultipleResult::NoIntersectionsFound(distance_check_multiple_info) );
    } else {
        return Ok( DistanceCheckMultipleResult::IntersectionFound(distance_check_multiple_info) );
    }
}

pub fn distance_check_between_multiple_collision_objects_subset(subset_check_idxs: &Vec<  [ [usize; 2]; 2 ]  >, group1: &Vec<Vec<CollisionObject>>, group2: &Vec<Vec<CollisionObject>>, stop_at_first_detected_intersection: bool, skip_collision_check_tensor: Option<&BoolCollisionCheckTensor>, average_distance_tensor: Option<&FloatCollisionCheckTensor>) -> Result<DistanceCheckMultipleResult, String> {
    let check_for_collision_skips = skip_collision_check_tensor.is_some();
    let mut distance_check_multiple_info = DistanceCheckMultipleInfo::new(stop_at_first_detected_intersection);

    for c in subset_check_idxs {
        if !check_for_collision_skips || !skip_collision_check_tensor.as_ref().unwrap().get_is_skip( c[0], c[1] )? {
            if group1[c[0][0]][c[0][1]].active && group2[c[1][0]][c[1][1]].active {
                let distance = group1[c[0][0]][c[0][1]].distance_check(&group2[c[1][0]][c[1][1]]);
                distance_check_multiple_info.add_distance(group1[c[0][0]][c[0][1]].name.clone(), c[0], group2[c[1][0]][c[1][1]].name.clone(), c[1], distance, average_distance_tensor);
                if distance == 0.0 && stop_at_first_detected_intersection {
                    distance_check_multiple_info._stopwatch.add_time_marker(None);
                    return Ok(DistanceCheckMultipleResult::IntersectionFound(distance_check_multiple_info));
                }
            }
        }
    }

    distance_check_multiple_info._stopwatch.add_time_marker(None);
    if distance_check_multiple_info.get_num_intersection() == 0 {
        return Ok( DistanceCheckMultipleResult::NoIntersectionsFound(distance_check_multiple_info) );
    } else {
        return Ok( DistanceCheckMultipleResult::IntersectionFound(distance_check_multiple_info) );
    }
}



#[derive(Debug, Clone)]
pub enum ContactCheckMultipleResult {
    IntersectionFound(ContactCheckMultipleInfo),
    NoIntersectionsFound(ContactCheckMultipleInfo)
}

impl ContactCheckMultipleResult {
    pub fn print_summary(&self) {
        match self {
            ContactCheckMultipleResult::IntersectionFound(c) => {
                println!("Intersection Found");
                c._stopwatch.print_summary();
                for i in 0..c._contact_check_names.len() {
                    println!("{:?} ---> {:?}, {:?}, {:?}, {:?}", i, c._contact_check_names[i], c._contact_check_idxs[i], c._contact_check_depths_with_respect_to_average[i],c._contact_check_contacts[i]);
                }
            },
            ContactCheckMultipleResult::NoIntersectionsFound(c) => {
                println!("Intersection Not Found");
                c._stopwatch.print_summary();
                for i in 0..c._contact_check_names.len() {
                    println!("{:?} ---> {:?}, {:?}, {:?}, {:?}", i, c._contact_check_names[i], c._contact_check_idxs[i], c._contact_check_depths_with_respect_to_average[i], c._contact_check_contacts[i]);
                }
            }
        }
    }

    pub fn get_closest_n_contact_idxs(&self, n: usize) -> Vec< [ [usize; 2]; 2 ] > {
        match self {
            ContactCheckMultipleResult::IntersectionFound(info) => return info.get_closest_n_contact_idxs(n),
            ContactCheckMultipleResult::NoIntersectionsFound(info) => return info.get_closest_n_contact_idxs(n)
        }
    }

    pub fn get_contact_check_multiple_info_ref(&self) -> &ContactCheckMultipleInfo {
        match self {
            ContactCheckMultipleResult::IntersectionFound(info) => return info,
            ContactCheckMultipleResult::NoIntersectionsFound(info) => return info
        }
    }

    pub fn is_in_collision(&self) -> bool {
        return match self {
            ContactCheckMultipleResult::NoIntersectionsFound(_) => { false }
            ContactCheckMultipleResult::IntersectionFound(_) => { true }
        }
    }
}

#[derive(Debug, Clone)]
pub struct ContactCheckMultipleInfo {
    _contact_check_names: Vec< [String; 2] >,
    _contact_check_idxs: Vec<  [ [usize; 2]; 2 ]  >,
    _contact_check_contacts: Vec<Contact<f64>>,
    _contact_check_depths_with_respect_to_average: Vec<f64>,
    _num_intersections: usize,
    _stopped_at_first_detected_intersection: bool,
    _margin: Option<f64>,
    _stopwatch: Stopwatch
}

impl ContactCheckMultipleInfo {
    pub fn new(stopped_at_first_detected_intersection: bool, margin: Option<f64>) -> Self {
        return Self { _contact_check_names: Vec::new(), _contact_check_idxs: Vec::new(),
            _contact_check_contacts: Vec::new(), _contact_check_depths_with_respect_to_average: Vec::new(), _num_intersections: 0,
            _stopped_at_first_detected_intersection: stopped_at_first_detected_intersection, _margin: margin, _stopwatch: Stopwatch::new() };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_contact(&mut self, collision_obj_1_name: String, collision_obj_1_idxs: [usize; 2], collision_obj_2_name: String, collision_obj_2_idxs: [usize; 2], contact: &Contact<f64>, average_distance_tensor: Option<&FloatCollisionCheckTensor>) -> Result<(), String> {
        let mut distance_ratio_with_respect_to_average = -contact.depth;
        if average_distance_tensor.is_some() {
            distance_ratio_with_respect_to_average = -contact.depth / average_distance_tensor.as_ref().unwrap().get_mean_at_given_idxs(collision_obj_1_idxs, collision_obj_2_idxs)?;
        }


        // let add_idx_res = self._contact_check_contacts.binary_search_by(|x| contact.depth.partial_cmp(&x.depth).unwrap());
        let add_idx_res = self._contact_check_depths_with_respect_to_average.binary_search_by(|x| x.partial_cmp(&distance_ratio_with_respect_to_average).unwrap());
        match add_idx_res {
            Ok(i) => {
                self._contact_check_names.insert(i, [collision_obj_1_name, collision_obj_2_name]);
                self._contact_check_idxs.insert(i, [ collision_obj_1_idxs, collision_obj_2_idxs ]);
                self._contact_check_contacts.insert(i, contact.clone());
                self._contact_check_depths_with_respect_to_average.insert(i, distance_ratio_with_respect_to_average);
            },
            Err(i) => {
                self._contact_check_names.insert(i, [collision_obj_1_name, collision_obj_2_name]);
                self._contact_check_idxs.insert(i, [ collision_obj_1_idxs, collision_obj_2_idxs ]);
                self._contact_check_contacts.insert(i, contact.clone());
                self._contact_check_depths_with_respect_to_average.insert(i, distance_ratio_with_respect_to_average);
            }
        }
        if contact.depth >= 0.0 { self._num_intersections += 1 }

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_contact_check_names(&self) -> &Vec< [String; 2] > {
        return &self._contact_check_names;
    }

    pub fn get_contact_check_idxs(&self) -> &Vec<  [ [usize; 2]; 2 ]  > {
        return &self._contact_check_idxs;
    }

    pub fn get_contact_check_idxs_in_intersection(&self) -> Vec<  [ [usize; 2]; 2 ]  > {
        return self.get_contact_check_idxs_greater_than_margin(0.0);
    }

    pub fn get_contact_check_idxs_greater_than_margin(&self, margin: f64) -> Vec<  [ [usize; 2]; 2 ]  > {
        if self._contact_check_idxs.len() == 0 { return Vec::new(); }

        let mut out_vec = self._contact_check_idxs.clone();

        let contact_binary_search_res = self._contact_check_contacts.binary_search_by(|x| x.depth.partial_cmp(&margin).unwrap() );
        match contact_binary_search_res {
            Ok(i) => { out_vec.truncate(i); return out_vec; },
            Err(i) => { out_vec.truncate(i); return out_vec; }
        }
    }

    pub fn get_contact_check_contacts(&self) -> &Vec< Contact<f64>  > {
        return &self._contact_check_contacts;
    }

    pub fn get_contact_check_depths_with_respect_to_average(&self) -> &Vec<f64> {
        return &self._contact_check_depths_with_respect_to_average;
    }

    pub fn get_num_intersection(&self) -> usize {
        return self._num_intersections;
    }

    pub fn get_stopwatch(&self) -> &Stopwatch {
        return &self._stopwatch;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_closest_n_contact_idxs(&self, n: usize) -> Vec< [ [usize; 2]; 2 ] > {
        let mut out_vec = Vec::new();

        let l = self._contact_check_depths_with_respect_to_average.len();
        for i in 0..l {
            if i >= n { return out_vec; }

            out_vec.push( self._contact_check_idxs[i] );
        }

        out_vec
    }
}

pub fn contact_check_between_multiple_collision_objects(group1: &Vec<Vec<CollisionObject>>, group2: &Vec<Vec<CollisionObject>>, stop_at_first_detected_intersection: bool, margin: Option<f64>, skip_collision_check_tensor: Option<&BoolCollisionCheckTensor>, average_distance_tensor: Option<&FloatCollisionCheckTensor>) -> Result<ContactCheckMultipleResult, String> {
    let check_for_collision_skips = skip_collision_check_tensor.is_some();
    let mut contact_check_multiple_info = ContactCheckMultipleInfo::new(stop_at_first_detected_intersection, margin);

    let l1 = group1.len();
    let l3 = group2.len();

    for i in 0..l1 {
        let l2 = group1[i].len();
        for j in 0..l2 {
            for k in 0..l3 {
                let l4 = group2[k].len();
                for l in 0..l4 {
                    if !check_for_collision_skips || !skip_collision_check_tensor.as_ref().unwrap().get_is_skip( [i,j], [k, l] )? {
                        if group1[i][j].active && group2[k][l].active {
                            let contact = group1[i][j].contact_check(&group2[k][l], margin);
                            if contact.is_some() {
                                contact_check_multiple_info.add_contact(group1[i][j].name.clone(), [i, j], group2[k][l].name.clone(), [k, l], &contact.as_ref().unwrap(), average_distance_tensor);
                                if stop_at_first_detected_intersection && contact.as_ref().unwrap().depth >= 0.0 {
                                    contact_check_multiple_info._stopwatch.add_time_marker(None);
                                    return Ok(ContactCheckMultipleResult::IntersectionFound(contact_check_multiple_info));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    contact_check_multiple_info._stopwatch.add_time_marker(None);
    if contact_check_multiple_info.get_num_intersection() == 0 {
        return Ok( ContactCheckMultipleResult::NoIntersectionsFound(contact_check_multiple_info) );
    } else {
        return Ok( ContactCheckMultipleResult::IntersectionFound(contact_check_multiple_info) );
    }
}

pub fn contact_check_between_multiple_collision_objects_subset(subset_check_idxs: &Vec<  [ [usize; 2]; 2 ]  >, group1: &Vec<Vec<CollisionObject>>, group2: &Vec<Vec<CollisionObject>>, stop_at_first_detected_intersection: bool, margin: Option<f64>, skip_collision_check_tensor: Option<&BoolCollisionCheckTensor>, average_distance_tensor: Option<&FloatCollisionCheckTensor>) -> Result<ContactCheckMultipleResult, String> {
    let check_for_collision_skips = skip_collision_check_tensor.is_some();
    let mut contact_check_multiple_info = ContactCheckMultipleInfo::new(stop_at_first_detected_intersection, margin);

    for c in subset_check_idxs {
        if !check_for_collision_skips || !skip_collision_check_tensor.as_ref().unwrap().get_is_skip( c[0], c[1] )? {
            if group1[c[0][0]][c[0][1]].active && group2[c[1][0]][c[1][1]].active {
                let start = Instant::now();
                let contact = group1[c[0][0]][c[0][1]].contact_check(&group2[c[1][0]][c[1][1]], margin);
                if contact.is_some() {
                    contact_check_multiple_info.add_contact(group1[c[0][0]][c[0][1]].name.clone(), c[0], group2[c[1][0]][c[1][1]].name.clone(), c[1], &contact.as_ref().unwrap(), average_distance_tensor);
                    if stop_at_first_detected_intersection && contact.as_ref().unwrap().depth >= 0.0 {
                        contact_check_multiple_info._stopwatch.add_time_marker(None);
                        return Ok(ContactCheckMultipleResult::IntersectionFound(contact_check_multiple_info));
                    }
                }
            }
        }
    }

    contact_check_multiple_info._stopwatch.add_time_marker(None);

    if contact_check_multiple_info.get_num_intersection() == 0 {
        return Ok( ContactCheckMultipleResult::NoIntersectionsFound(contact_check_multiple_info) );
    } else {
        return Ok( ContactCheckMultipleResult::IntersectionFound(contact_check_multiple_info) );
    }
}




#[derive(Clone, Debug)]
pub enum RayIntersectCheckMultipleResult {
    IntersectionFound(RayIntersectCheckMultipleInfo),
    NoIntersectionFound(RayIntersectCheckMultipleInfo)
}

#[derive(Clone, Debug)]
pub struct RayIntersectCheckMultipleInfo {
    _intersection_names: Vec< String>,
    _intersection_idxs: Vec<  [ usize; 2]  >,
    _stopped_at_first_detected: bool,
    _stopwatch: Stopwatch
}

impl RayIntersectCheckMultipleInfo {
    pub fn new(stopped_at_first_detected: bool) -> Self {
        return Self{ _intersection_names: Vec::new(), _intersection_idxs: Vec::new(), _stopped_at_first_detected: stopped_at_first_detected, _stopwatch: Stopwatch::new() };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_ray_intersection(&mut self, collision_obj_name: String, collision_obj_idxs: [usize; 2] ) {
        self._intersection_names.push( collision_obj_name );
        self._intersection_idxs.push(  collision_obj_idxs );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_ray_intersection_names(&self) -> &Vec< String > {
        return &self._intersection_names;
    }

    pub fn get_ray_intersection_idxs(&self) -> &Vec<  [ usize; 2]  > {
        return &self._intersection_idxs;
    }

    pub fn get_stopwatch(&self) -> &Stopwatch {
        return &self._stopwatch;
    }
}

pub fn ray_intersect_check_between_multiple_collision_objects(group: &Vec<Vec<CollisionObject>>, ray_origin: &Point3<f64>, ray_direction: &Vector3<f64>, stop_at_first_detected: bool) -> Result<RayIntersectCheckMultipleResult, String> {
    let mut ray_intersect_check_multiple_info = RayIntersectCheckMultipleInfo::new(stop_at_first_detected);

    let l = group.len();
    for i in 0..l {
        let l2 = group[i].len();
        for j in 0..l2 {
            if group[i][j].active {
                let ray_intersect_check = group[i][j].intersects_ray(ray_origin, ray_direction);
                if ray_intersect_check {
                    ray_intersect_check_multiple_info.add_ray_intersection(group[i][j].name.clone(), [i, j]);
                    if stop_at_first_detected {
                        ray_intersect_check_multiple_info._stopwatch.add_time_marker(None);
                        return Ok(RayIntersectCheckMultipleResult::IntersectionFound(ray_intersect_check_multiple_info));
                    }
                }
            }
        }
    }

    ray_intersect_check_multiple_info._stopwatch.add_time_marker(None);
    if ray_intersect_check_multiple_info.get_ray_intersection_names().is_empty() {
        return Ok( RayIntersectCheckMultipleResult::NoIntersectionFound(ray_intersect_check_multiple_info) );
    } else {
        return Ok(RayIntersectCheckMultipleResult::IntersectionFound(ray_intersect_check_multiple_info));
    }
}

pub fn ray_intersect_check_between_multiple_collision_objects_subset(subset_check_idxs: Vec<  [ usize; 2]  >, group: &Vec<Vec<CollisionObject>>, ray_origin: &Point3<f64>, ray_direction: &Vector3<f64>, stop_at_first_detected: bool) -> Result<RayIntersectCheckMultipleResult, String> {
    let mut ray_intersect_check_multiple_info = RayIntersectCheckMultipleInfo::new(stop_at_first_detected);

    for c in subset_check_idxs {
        if group[c[0]][c[1]].active {
            let ray_intersect_check = group[c[0]][c[1]].intersects_ray(ray_origin, ray_direction);
            if ray_intersect_check {
                ray_intersect_check_multiple_info.add_ray_intersection(group[c[0]][c[1]].name.clone(), c);
                if stop_at_first_detected {
                    ray_intersect_check_multiple_info._stopwatch.add_time_marker(None);
                    return Ok(RayIntersectCheckMultipleResult::IntersectionFound(ray_intersect_check_multiple_info));
                }
            }
        }
    }

    ray_intersect_check_multiple_info._stopwatch.add_time_marker(None);
    if ray_intersect_check_multiple_info.get_ray_intersection_names().is_empty() {
        return Ok( RayIntersectCheckMultipleResult::NoIntersectionFound(ray_intersect_check_multiple_info) );
    } else {
        return Ok(RayIntersectCheckMultipleResult::IntersectionFound(ray_intersect_check_multiple_info));
    }
}





#[derive(Clone, Debug)]
pub enum ContainsPointCheckMultipleResult {
    PointContainerFound(ContainsPointCheckMultipleInfo),
    NoPointContainerFound(ContainsPointCheckMultipleInfo)
}

#[derive(Clone, Debug)]
pub struct ContainsPointCheckMultipleInfo {
    _container_names: Vec< String>,
    _container_idxs: Vec<  [ usize; 2]  >,
    _stopped_at_first_detected_containment: bool,
    _stopwatch: Stopwatch
}

impl ContainsPointCheckMultipleInfo {
    pub fn new(stopped_at_first_detected_containment: bool) -> Self {
        return Self{ _container_names: Vec::new(), _container_idxs: Vec::new(), _stopped_at_first_detected_containment: stopped_at_first_detected_containment, _stopwatch: Stopwatch::new() };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_point_container(&mut self, collision_obj_name: String, collision_obj_idxs: [usize; 2] ) {
        self._container_names.push( collision_obj_name );
        self._container_idxs.push(  collision_obj_idxs );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_container_names(&self) -> &Vec< String > {
        return &self._container_names;
    }

    pub fn get_container_idxs(&self) -> &Vec<  [ usize; 2]  > {
        return &self._container_idxs;
    }
}

pub fn contains_point_check_between_multiple_collision_objects(group: &Vec<Vec<CollisionObject>>, point: &Point3<f64>, stop_at_first_detected: bool) -> Result<ContainsPointCheckMultipleResult, String> {
    let mut contains_point_check_multiple_info = ContainsPointCheckMultipleInfo::new(stop_at_first_detected);

    let l = group.len();
    for i in 0..l {
        let l2 = group[i].len();
        for j in 0..l2 {
            if group[i][j].active {
                let contains_point_check = group[i][j].contains_point(point);
                if contains_point_check {
                    contains_point_check_multiple_info.add_point_container(group[i][j].name.clone(), [i, j]);
                    if stop_at_first_detected {
                        contains_point_check_multiple_info._stopwatch.add_time_marker(None);
                        return Ok(ContainsPointCheckMultipleResult::PointContainerFound(contains_point_check_multiple_info));
                    }
                }
            }
        }
    }

    contains_point_check_multiple_info._stopwatch.add_time_marker(None);
    if contains_point_check_multiple_info.get_container_names().is_empty() {
        return Ok( ContainsPointCheckMultipleResult::NoPointContainerFound(contains_point_check_multiple_info) );
    } else {
        return Ok(ContainsPointCheckMultipleResult::PointContainerFound(contains_point_check_multiple_info));
    }
}

pub fn contains_point_check_between_multiple_collision_objects_subset(subset_check_idxs: Vec<  [ usize; 2]  >, group: &Vec<Vec<CollisionObject>>, point: &Point3<f64>, stop_at_first_detected: bool) -> Result<ContainsPointCheckMultipleResult, String> {
    let mut contains_point_check_multiple_info = ContainsPointCheckMultipleInfo::new(stop_at_first_detected);

    for c in subset_check_idxs {
        if group[c[0]][c[1]].active {
            let contains_point_check = group[c[0]][c[1]].contains_point(point);
            if contains_point_check {
                contains_point_check_multiple_info.add_point_container(group[c[0]][c[1]].name.clone(), c);
                if stop_at_first_detected {
                    contains_point_check_multiple_info._stopwatch.add_time_marker(None);
                    return Ok(ContainsPointCheckMultipleResult::PointContainerFound(contains_point_check_multiple_info));
                }
            }
        }
    }

    contains_point_check_multiple_info._stopwatch.add_time_marker(None);
    if contains_point_check_multiple_info.get_container_names().is_empty() {
        return Ok( ContainsPointCheckMultipleResult::NoPointContainerFound(contains_point_check_multiple_info) );
    } else {
        return Ok(ContainsPointCheckMultipleResult::PointContainerFound(contains_point_check_multiple_info));
    }
}





