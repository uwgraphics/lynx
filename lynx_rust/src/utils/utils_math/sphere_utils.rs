use nalgebra::{Vector3};

pub fn volume_of_overlap_of_two_spheres(distance_between_sphere_centers: f64, radius1: f64, radius2: f64) -> f64 {
    let cl = (radius1 - radius2).abs();
    let cu = radius1 + radius2;

    if cl >= distance_between_sphere_centers || cu <= distance_between_sphere_centers {
        return 0.0;
    }

    let t1 = std::f64::consts::PI / (12.0 * distance_between_sphere_centers);
    let t2 = (radius1 + radius2 - distance_between_sphere_centers).powi(2);
    let t3 = (distance_between_sphere_centers.powi(2) + 2.0*distance_between_sphere_centers*(radius1 + radius2) - 3.0*(radius1 - radius2).powi(2));
    return t1 * t2* t3;
}

pub fn volume_of_overlap_of_two_spheres_by_coordinates(origin1: &Vector3<f64>, origin2: &Vector3<f64>, radius1: f64, radius2: f64) -> f64 {
    let distance_between_sphere_centers = (origin1 - origin2).norm();
    return volume_of_overlap_of_two_spheres(distance_between_sphere_centers, radius1, radius2);
}