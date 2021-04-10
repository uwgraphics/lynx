use crate::utils::utils_collisions::oriented_bounding_box::OBB;



pub fn combine_two_obbs(a: &mut OBB, b: &mut OBB) -> OBB {
    return a.combine(b);
}

pub fn combine_two_obbs_immutable(a: &OBB, b: &OBB) -> OBB {
    return a.combine_immutable(b);
}

pub fn combine_multiple_obbs(obbs: &mut Vec<&mut OBB>) -> OBB {
    let mut out_obb = OBB::new_empty();
    let l = obbs.len();
    for i in 0..l {
        // obbs.iter_mut();
        // obbs[i].update_max_and_mins(true);
        out_obb = out_obb.combine(obbs[i]);
    }
    return out_obb;
}

pub fn get_combined_volume(a: &mut OBB, b: &mut OBB) -> f64 {
    if a.corners_outdated { a.update_max_and_mins(true);  a.corners_outdated = false; }
    if b.corners_outdated { b.update_max_and_mins(true); b.corners_outdated = false; }

    return get_combined_volume_immutable(a, b);
}

pub fn get_combined_volume_immutable(a: &OBB, b: &OBB) -> f64 {
    let mut dim_maximums = [0.,0.,0.];
    let mut dim_minimums = [0.,0.,0.];

    for i in 0..3 {
        dim_maximums[i] = a.dim_maximums[i].max(b.dim_maximums[i]);
    }
    for i in 0..3 {
        dim_minimums[i] = a.dim_minimums[i].min(b.dim_minimums[i]);
    }

    let mut extents = [0., 0., 0.];
    extents[0] = (dim_maximums[0] - dim_minimums[0]);
    extents[1] = (dim_maximums[1] - dim_minimums[1]);
    extents[2] = (dim_maximums[2] - dim_minimums[2]);

    let mut combined_volume = extents[0] * extents[1] * extents[2];

    return combined_volume;
}

pub fn get_combined_volume_multiple(obbs: &mut Vec<&mut OBB>) -> f64 {
    let mut dim_maximums = [-std::f64::INFINITY,-std::f64::INFINITY,-std::f64::INFINITY];
    let mut dim_minimums = [std::f64::INFINITY,std::f64::INFINITY,std::f64::INFINITY];

    /*
    for i in 0..3 {
        dim_maximums[i] = a.dim_maximums[i].max(b.dim_maximums[i]);
    }
    for i in 0..3 {
        dim_minimums[i] = a.dim_minimums[i].min(b.dim_minimums[i]);
    }
    */

    let l = obbs.len();
    for i in 0..l {
        if obbs[i].corners_outdated {  obbs[i].update_max_and_mins(true);  }
        for j in 0..3 {
            if obbs[i].dim_maximums[j] > dim_maximums[j] {  dim_maximums[j] = obbs[i].dim_maximums[j]; }
            if obbs[i].dim_minimums[j] < dim_minimums[j] {  dim_minimums[j] = obbs[i].dim_minimums[j]; }
        }
    }

    let mut extents = [0., 0., 0.];
    extents[0] = (dim_maximums[0] - dim_minimums[0]);
    extents[1] = (dim_maximums[1] - dim_minimums[1]);
    extents[2] = (dim_maximums[2] - dim_minimums[2]);

    let mut combined_volume = extents[0] * extents[1] * extents[2];

    return combined_volume;
}

pub fn get_combined_volume_ratio(a: &mut OBB, b: &mut OBB) -> f64 {
    /*  ratio of the sums of the original volumes divided by the volume of the combined OBB.  A larger number indicates a better fit.  */
    let mut original_volume = a.volume + b.volume;
    let mut combined_volume = get_combined_volume(a, b);
    return original_volume / combined_volume;
}

pub fn get_combined_volume_ratio_immutable(a: &OBB, b: &OBB) -> f64 {
    /*  ratio of the sums of the original volumes divided by the volume of the combined OBB.  A larger number indicates a better fit.  */
    let mut original_volume = a.volume + b.volume;
    let mut combined_volume = get_combined_volume_immutable(a, b);
    return original_volume / combined_volume;
}

pub fn get_combined_volume_ratio_multiple(obbs: &mut Vec<&mut OBB>) -> f64 {
    /*  ratio of the sums of the original volumes divided by the volume of the combined OBB.  A larger number indicates a better fit.  */
    let mut original_volume = 0.0;
    let l = obbs.len();
    for i in 0..l {
        original_volume += obbs[i].volume;
    }
    let mut combined_volume = get_combined_volume_multiple(obbs);
    return original_volume / combined_volume;
}