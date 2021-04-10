use nalgebra::{DVector, Vector3};

pub fn vec_to_dvec(x: &Vec<f64>) -> DVector<f64> {
    let l = x.len();
    let mut out_vec: DVector<f64> = DVector::from_element(l, 0.);
    for i in 0..l {
        out_vec[i] = x[i];
    }
    out_vec
}

// Returns vector with values between -1 and 1
pub fn get_random_dvec(d: usize) -> DVector<f64> {
    let ret: DVector<f64> = 2.0*DVector::new_random(d) - DVector::from_element(d, 1.0);
    ret
}

pub fn vector3_to_dvec(x: &Vector3<f64>) -> DVector<f64> {
    let mut out = DVector::from_element(3, 0.0);
    out[0] = x[0];
    out[1] = x[1];
    out[2] = x[2];
    return out;
}

pub fn dvec_to_vector3(x: &DVector<f64>) -> Vector3<f64> {
    let mut out = Vector3::zeros();
    out[0] =x[0]; out[1] = x[1]; out[2] = x[2];
    return out;
}