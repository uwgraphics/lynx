use nalgebra::{DVector};

pub fn proj_scalar(q: &DVector<f64>, q_a: &DVector<f64>, q_b: &DVector<f64>) -> f64 {
    let q_b_minus_q_a = (q_b - q_a);
    let q_minus_q_a = (q - q_a);
    return _proj_scalar(&q_minus_q_a, &q_b_minus_q_a)
}

pub fn proj(q: &DVector<f64>, q_a: &DVector<f64>, q_b: &DVector<f64>) -> DVector<f64> {
    let q_b_minus_q_a = (q_b - q_a);
    let q_minus_q_a = (q - q_a);

    let proj_scalar = _proj_scalar(&q_minus_q_a, &q_b_minus_q_a);

    return q_a + proj_scalar * &q_b_minus_q_a;
}

pub fn proj_plus(q: &DVector<f64>, q_a: &DVector<f64>, q_b: &DVector<f64>) -> DVector<f64> {
    let q_b_minus_q_a = (q_b - q_a);
    let q_minus_q_a = (q - q_a);

    let proj_scalar = _proj_scalar(&q_minus_q_a, &q_b_minus_q_a);

    return q_a + proj_scalar.max(0.0) * &q_b_minus_q_a;
}

pub fn heaviside(val: f64) -> f64 {
    if val < 0.0 { return 0.0 }
    else { return 1.0 }
}

fn _proj_scalar(q_minus_q_a: &DVector<f64>, q_b_minus_q_a: &DVector<f64>) -> f64 {
    return q_minus_q_a.dot( &q_b_minus_q_a ) /  q_b_minus_q_a.dot( &q_b_minus_q_a ) ;
}
