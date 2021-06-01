use crate::utils::utils_collisions::collision_object::*;
use crate::utils::utils_collisions::collision_check_tensor::*;
use crate::utils::utils_collisions::collision_multiple_results::*;
use crate::utils::utils_recorders::stopwatch::Stopwatch;
use nalgebra::{Point3, Vector3};
use ncollide3d::query::{Proximity, PointQuery, Contact, Ray, RayCast, PointProjection};
use std::time::{Instant, Duration};
use std::fmt;


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
                                        intersections_info.get_stopwatch_mut_ref().add_time_marker(None);
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

    intersections_info.get_stopwatch_mut_ref().add_time_marker(None);
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
                        intersections_info.get_stopwatch_mut_ref().add_time_marker(None);
                        return Ok(IntersectCheckMultipleResult::IntersectionFound(intersections_info));
                    }
                }
            }
        }
    }

    intersections_info.get_stopwatch_mut_ref().add_time_marker(None);

    if intersections_info.get_intersection_names().is_empty() {
        return Ok( IntersectCheckMultipleResult::NoIntersectionsFound( intersections_info ) );
    } else {
        return Ok( IntersectCheckMultipleResult::IntersectionFound( intersections_info ) );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

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
                                distance_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
                                return Ok(DistanceCheckMultipleResult::IntersectionFound(distance_check_multiple_info));
                            }
                        }
                    }
                }
            }
        }
    }

    distance_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
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
                    distance_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
                    return Ok(DistanceCheckMultipleResult::IntersectionFound(distance_check_multiple_info));
                }
            }
        }
    }

    distance_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
    if distance_check_multiple_info.get_num_intersection() == 0 {
        return Ok( DistanceCheckMultipleResult::NoIntersectionsFound(distance_check_multiple_info) );
    } else {
        return Ok( DistanceCheckMultipleResult::IntersectionFound(distance_check_multiple_info) );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

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
                                    contact_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
                                    return Ok(ContactCheckMultipleResult::IntersectionFound(contact_check_multiple_info));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    contact_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
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
                        contact_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
                        return Ok(ContactCheckMultipleResult::IntersectionFound(contact_check_multiple_info));
                    }
                }
            }
        }
    }

    contact_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);

    if contact_check_multiple_info.get_num_intersection() == 0 {
        return Ok( ContactCheckMultipleResult::NoIntersectionsFound(contact_check_multiple_info) );
    } else {
        return Ok( ContactCheckMultipleResult::IntersectionFound(contact_check_multiple_info) );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

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
                        ray_intersect_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
                        return Ok(RayIntersectCheckMultipleResult::IntersectionFound(ray_intersect_check_multiple_info));
                    }
                }
            }
        }
    }

    ray_intersect_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
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
                    ray_intersect_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
                    return Ok(RayIntersectCheckMultipleResult::IntersectionFound(ray_intersect_check_multiple_info));
                }
            }
        }
    }

    ray_intersect_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
    if ray_intersect_check_multiple_info.get_ray_intersection_names().is_empty() {
        return Ok( RayIntersectCheckMultipleResult::NoIntersectionFound(ray_intersect_check_multiple_info) );
    } else {
        return Ok(RayIntersectCheckMultipleResult::IntersectionFound(ray_intersect_check_multiple_info));
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

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
                        contains_point_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
                        return Ok(ContainsPointCheckMultipleResult::PointContainerFound(contains_point_check_multiple_info));
                    }
                }
            }
        }
    }

    contains_point_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
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
                    contains_point_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
                    return Ok(ContainsPointCheckMultipleResult::PointContainerFound(contains_point_check_multiple_info));
                }
            }
        }
    }

    contains_point_check_multiple_info.get_stopwatch_mut_ref().add_time_marker(None);
    if contains_point_check_multiple_info.get_container_names().is_empty() {
        return Ok( ContainsPointCheckMultipleResult::NoPointContainerFound(contains_point_check_multiple_info) );
    } else {
        return Ok(ContainsPointCheckMultipleResult::PointContainerFound(contains_point_check_multiple_info));
    }
}





