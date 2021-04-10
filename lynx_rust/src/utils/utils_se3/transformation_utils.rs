use nalgebra::{Vector3, UnitQuaternion, Quaternion, Vector4, Vector6, Point3};
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;

pub fn quaternion_log(q: UnitQuaternion<f64>) -> Vector3<f64> {
    let mut out_vec: Vector3<f64> = Vector3::new(q.i, q.j, q.k);
    if q.w.abs() < 1.0 {
        let a = q.w.acos();
        let sina = a.sin();
        if sina.abs() >= 0.005 {
            let c = a / sina;
            out_vec *= c;
        }
    }
    out_vec
}

pub fn quaternion_exp(v: Vector3<f64>) -> UnitQuaternion<f64> {
    let mut qv: Vector4<f64> = Vector4::new(1.0, v[0], v[1], v[2]);
    let a = qv.norm();
    let sina = a.sin();
    if sina.abs() >= 0.005 {
        let c = sina/a;
        qv *= c;
    }
    UnitQuaternion::from_quaternion(Quaternion::new(a.cos(), qv[1], qv[2], qv[3]))
}

pub fn quaternion_disp(q: UnitQuaternion<f64>, q_prime: UnitQuaternion<f64>) -> Vector3<f64> {
    quaternion_log( q.inverse()*q_prime )
}

pub fn quaternion_disp_q(q: UnitQuaternion<f64>, q_prime: UnitQuaternion<f64>) -> UnitQuaternion<f64> {
    q.inverse()*q_prime
}

pub fn angle_between(q: UnitQuaternion<f64>, q_prime: UnitQuaternion<f64>) -> f64 {
    quaternion_disp(q, q_prime).norm() * 2.0
}

pub fn implicit_dual_quaternion_log(idq: &ImplicitDualQuaternion) -> (Vector3<f64>, Vector3<f64>) {
    let h_v = Vector3::new( idq.quat.i,idq.quat.j, idq.quat.k  );
    let s: f64 = h_v.norm();
    let c = idq.quat.w;
    let phi = s.atan2(c);
    let mut a = 0.0;
    if s > 0.0 { a = phi / s; }
    let rot_vec_diff = a * &h_v;

    let mut mu_r = 0.0;
    let mut mu_d = 0.0;

    if s < 0.00000000000001 {
        mu_r = 1.0 - (phi.powi(2) / 3.0) - (phi.powi(4) / 45.0);
    } else {
        mu_r = (c * phi) / s;
    }

    if phi < 0.000000000001 {
        mu_d = (1.0 / 3.0) + (phi.powi(2) / 45.0) + ((2.0 * phi.powi(4)) / 945.0);
    } else {
        mu_d = (1.0 - mu_r) / (phi.powi(2));
    }

    let tmp = (&idq.translation / 2.0);
    let mut translation_diff = mu_d * ( &tmp.dot(&rot_vec_diff) ) * &rot_vec_diff + mu_r * &tmp + &tmp.cross(&rot_vec_diff);

    return (rot_vec_diff, translation_diff);
}

pub fn implicit_dual_quaternion_exp(w: &Vector3<f64>, v: &Vector3<f64>) -> ImplicitDualQuaternion {
    let phi = w.norm();
    let s = phi.sin();
    let c = phi.cos();
    let gamma = w.dot(v);

    let mut mu_r = 0.0;
    let mut mu_d = 0.0;

    if phi < 0.00000001 {
        mu_r = 1.0 - phi.powi(2)/6.0 + phi.powi(4) / 120.0;
        mu_d = 4.0/3.0 - 4.0*phi.powi(2)/15.0 + 8.0*phi.powi(4)/315.0;
    } else {
        mu_r = s / phi;
        mu_d = (2.0 - c*(2.0*mu_r)) / phi.powi(2);
    }

    let h_v: Vector3<f64> = mu_r * w;
    let mut quat_ = Quaternion::new(c, h_v[0], h_v[1], h_v[2]);
    let mut quat = UnitQuaternion::from_quaternion(quat_);

    let mut translation = 2.0 * mu_r * (&h_v.cross(v)) + c*(2.0*mu_r)*v + mu_d*gamma*w;

    return ImplicitDualQuaternion::new(quat, translation);
}

pub fn implicit_dual_quaternion_disp_idq(idq: &ImplicitDualQuaternion, idq_prime: &ImplicitDualQuaternion) -> ImplicitDualQuaternion {
    return idq.inverse().multiply(&idq_prime);
}

pub fn implicit_dual_quaternion_disp(idq: &ImplicitDualQuaternion, idq_prime: &ImplicitDualQuaternion) -> (Vector3<f64>, Vector3<f64>) {
    return implicit_dual_quaternion_log(&idq.inverse().multiply(&idq_prime));
}

pub fn implicit_dual_quaternion_disp_concat(idq: &ImplicitDualQuaternion, idq_prime: &ImplicitDualQuaternion) -> Vector6<f64> {
    let res = implicit_dual_quaternion_log(&idq.inverse().multiply(&idq_prime));
    return Vector6::new( res.0[0], res.0[1], res.0[2], res.1[0], res.1[1], res.1[2]  );
}

pub fn implicit_dual_quaternion_disp_l2_magnitude(idq: &ImplicitDualQuaternion, idq_prime: &ImplicitDualQuaternion) -> f64 {
    let res = implicit_dual_quaternion_log(&idq.inverse().multiply(&idq_prime));
    return ( res.0[0].powi(2) + res.0[1].powi(2) + res.0[2].powi(2) + res.1[0].powi(2) + res.1[1].powi(2) + res.1[2].powi(2) ).sqrt();
}

/*
transforms point_in_world_coordinate_system by the combined translation and rotation seen from idq1 to idq2 to a new point in the world coordinate system
*/
pub fn implicit_dual_quaternion_vector3_displacement_transform(idq1: &ImplicitDualQuaternion, idq2: &ImplicitDualQuaternion, point_in_world_coordinate_system: &Vector3<f64>) -> Vector3<f64> {
    let translation_disp = &idq2.translation - &idq1.translation;
    let p_tmp = point_in_world_coordinate_system + translation_disp;

    let p_centered = &p_tmp - &idq1.translation;
    let q_disp = quaternion_disp_q(idq1.quat.clone(), idq2.quat.clone());

    return (q_disp * p_centered) + &idq1.translation;
}

pub fn implicit_dual_quaternion_point3_displacement_transform(idq1: &ImplicitDualQuaternion, idq2: &ImplicitDualQuaternion, point_in_world_coordinate_system: &Point3<f64>) -> Point3<f64> {
    let translation_disp = &idq2.translation - &idq1.translation;
    let p_tmp = point_in_world_coordinate_system + translation_disp;

    let p_centered = &p_tmp - &idq1.translation;
    let q_disp = quaternion_disp_q(idq1.quat.clone(), idq2.quat.clone());

    return (q_disp * p_centered) + &idq1.translation;
}


