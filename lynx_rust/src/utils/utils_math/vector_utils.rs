use nalgebra::{DVector, DMatrix};
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_math::nalgebra_utils::get_random_dvec;
use crate::utils::utils_sampling::float_vec_sampler_traits::FloatVecSampler;

// PROBABLY NEED TO CLEAN UP OR DEPRECATE THESE FUNCTIONS

pub fn get_orthogonal_vector(vec: &DVector<f64>) -> Result<DVector<f64>, String> {
    let len = vec.len();
    let mut out_vec = DVector::from_element(len, 0.0);

    let sampler = RangeFloatVecSampler::new(-1.0, 1.0, len - 1);
    let r = sampler.float_vec_sampler_sample()?;

    let mut dot_product = 0.0;
    for i in 0..len-1 {
        dot_product += (r[i] * vec[i]);
        out_vec[i] = r[i];
    }

    let mut x = 1.0;
    if !(vec[len-1] == 0.0) {
        x = -dot_product / vec[len-1];
    } else {
        x = -dot_product / 0.0000000000000001;
    }

    out_vec[len-1] = x;

    // out_vec = (1.0 / out_vec.norm()) * &out_vec;

    out_vec = normalize_vector(&out_vec);

    return Ok(out_vec);

}

pub fn project_vector_onto_plane(spanning_vector1: &DVector<f64>, spanning_vector2: &DVector<f64>, vector_to_project: &DVector<f64>) -> DVector<f64> {
    let params = project_vector_onto_plane_return_params(spanning_vector1, spanning_vector2, vector_to_project);

    // NOTE: this is not returned normalized by default
    return params.0 * spanning_vector1 + params.1 * spanning_vector2;
}

pub fn project_vector_onto_plane_return_params(spanning_vector1: &DVector<f64>, spanning_vector2: &DVector<f64>, vector_to_project: &DVector<f64>) -> (f64, f64) {
    let mut A = DMatrix::from_element(2,2,0.0);
    let mut b = DVector::from_element(2, 0.0);

    let l = spanning_vector1.len();
    for i in 0..l {
        A[(0,0)] += 2.0 * spanning_vector1[i].powi(2);
        A[(0,1)] += 2.0 * spanning_vector1[i] * spanning_vector2[i];
        b[0] += -2.0 * spanning_vector1[i] * vector_to_project[i];

        A[(1,0)] += 2.0 * spanning_vector1[i] * spanning_vector2[i];
        A[(1,1)] += 2.0 * spanning_vector2[i].powi(2);
        b[1] += -2.0 * spanning_vector2[i] * vector_to_project[i];
    }

    let P = A.pseudo_inverse(0.00001).unwrap();

    let mut x_star = negate_vector(&(&P * &b));

    return (x_star[0], x_star[1]);
}

pub fn negate_vector(vec: &DVector<f64>) -> DVector<f64> {
    let len = vec.len();
    let mut out_vec = DVector::from_element(len, 0.0);

    for i in 0..len {
        out_vec[i] = -vec[i];
    }

    out_vec
}

pub fn normalize_vector(vec: &DVector<f64>) -> DVector<f64> {
    let n = vec.norm();
    if n == 0.0 {
        return vec.clone();
    }
    let mut out_vec = (1.0 / n) * vec;
    return out_vec
}

pub fn get_orthogonal_vector_within_plane(vec: &DVector<f64>, other_vec: &DVector<f64>) -> DVector<f64> {
    /*
    This function returns a vector orthogonal to vec that lies within the plane spanned by
    vec and other_vec.

    NOTE: vec and other_vec are just intended to be DIRECTION vectors that span a plane (i.e., the center of the "plane" they are spanning
    is at the origin).  It should NOT be thought of as "the point of the vectors is on the plane".  This could be a point
    of confusion.  Instead, think about these vectors as FULLY lying on the plane that they span.
    */

    let l = vec.len();
    let mut X = 0.0;
    for i in 0..l {
        X += vec[i]*vec[i];
    }

    let mut Y = 0.0;
    for i in 0..l {
        Y += vec[i]*other_vec[i];
    }
    Y += 0.00000000001;

    let b = (-X) / Y;

    let mut out_vec = vec + b*other_vec;

    return normalize_vector(&out_vec);

}

pub fn facing_same_direction_with_normalize(vec1: &DVector<f64>, vec2: &DVector<f64>) -> bool {
    let n_vec1 = normalize_vector(vec1);
    let n_vec2 = normalize_vector(vec2);

    return facing_same_direction(&n_vec1, &n_vec2);
}

pub fn facing_same_direction(vec1: &DVector<f64>, vec2: &DVector<f64>) -> bool {
    let d = vec1.dot(vec2);
    if d > 0.0 {
        return true;
    } else {
        return false;
    }
}

pub fn angle_between_vectors(vec1: &DVector<f64>, vec2: &DVector<f64>) -> f64 {
    /*
    returns value in radians
    */
    let n1 = normalize_vector(vec1);
    let n2 = normalize_vector(vec2);

    let d = n1.dot(&n2).min(1.0).max(-1.0);
    return d.acos();
}

pub fn angle_between_vectors_degrees(vec1: &DVector<f64>, vec2: &DVector<f64>) -> f64 {
    /*
    returns value in degrees
    */
    let n1 = normalize_vector(vec1);
    let n2 = normalize_vector(vec2);

    let d = n1.dot(&n2).min(1.0).max(-1.0);
    return d.acos().to_degrees();
}

pub fn angle_between_vectors_no_normalize(vec1: &DVector<f64>, vec2: &DVector<f64>) -> f64 {
    let d = vec1.dot(vec2).min(1.0).max(-1.0);
    return d.acos();
}

pub fn angle_between_vectors_degrees_no_normalize(vec1: &DVector<f64>, vec2: &DVector<f64>) -> f64 {
    let d = vec1.dot(vec2).min(1.0).max(-1.0);
    return d.acos().to_degrees();
}

pub fn proj(v: &DVector<f64>, u: &DVector<f64>) -> DVector<f64> {
    /*
    projects the vector v orthogonally onto u
    */

    return proj_scalar(v, u) * u;
}

pub fn proj_scalar(v: &DVector<f64>, u: &DVector<f64>) -> f64 {
    let n = v.dot(u);
    let d = u.dot(u);
    return (n/d);
}

pub fn get_orthonormal_basis(initial_vector: &DVector<f64>, basis_dim: usize) -> Result<Vec<DVector<f64>>, String> {
    let dim = initial_vector.len();
    let basis_dim_copy = basis_dim.min(dim);
    let sampler = RangeFloatVecSampler::new(-1.0, 1.0, dim);

    let mut out_vecs = vec![initial_vector.clone()];
    for i in 0..basis_dim_copy - 1 {
        out_vecs.push(sampler.float_vec_sampler_sample()?);
    }

    for i in 0..basis_dim_copy {
        let tmp = out_vecs[i].clone();
        for j in 0..i {
            let tmp = out_vecs[j].clone();
            out_vecs[i] -= &proj(&tmp, &tmp)
        }
    }

    for i in 0..basis_dim_copy {
        out_vecs[i] = normalize_vector(&out_vecs[i]);
    }

    return Ok(out_vecs);
}

pub fn get_orthonormal_basis_multiple_vectors(initial_vectors: &Vec<DVector<f64>>, basis_dim: usize) -> Result<Vec<DVector<f64>>, String> {
    let dim = initial_vectors[0].len();
    let basis_dim_copy = basis_dim.min(dim);
    let sampler = RangeFloatVecSampler::new(-1.0, 1.0, dim);

    let mut out_vecs = Vec::new();
    let l = initial_vectors.len();
    for i in 0..l {
        out_vecs.push(initial_vectors[i].clone());
    }

    for i in 0..basis_dim_copy - l {
        out_vecs.push(sampler.float_vec_sampler_sample()?);
    }

    for i in 0..basis_dim_copy {
        let tmp = out_vecs[i].clone();
        for j in 0..i {
            let tmp2 = out_vecs[j].clone();
            out_vecs[i] -= &proj(&tmp, &tmp2)
        }
    }

    for i in 0..basis_dim_copy {
        out_vecs[i] = normalize_vector(&out_vecs[i]);
    }

    return Ok(out_vecs);
}

pub fn get_orthonormal_basis_matrix(initial_vector: &DVector<f64>, basis_dim: usize) -> Result<DMatrix<f64>, String> {
    let mut out_mat = DMatrix::from_element(initial_vector.len(), basis_dim, 0.0);
    let vecs = get_orthonormal_basis(initial_vector, basis_dim)?;

    let l1 = vecs.len();
    for i in 0..l1 {
        let l2 = vecs[i].len();
        for j in 0..l2 {
            out_mat[(j,i)] = vecs[i][j];
        }
    }

    return Ok(out_mat);
}

pub fn get_orthonormal_basis_multiple_vectors_matrix(initial_vectors: &Vec<DVector<f64>>, basis_dim: usize) -> Result<DMatrix<f64>, String> {
    let mut out_mat = DMatrix::from_element(initial_vectors[0].len(), basis_dim, 0.0);
    let vecs = get_orthonormal_basis_multiple_vectors(initial_vectors, basis_dim)?;

    let l1 = vecs.len();
    for i in 0..l1 {
        let l2 = vecs[i].len();
        for j in 0..l2 {
            out_mat[(j,i)] = vecs[i][j];
        }
    }

    return Ok(out_mat);
}