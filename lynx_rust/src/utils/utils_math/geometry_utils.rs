use nalgebra::{DVector, Vector2, Vector3};
use crate::utils::utils_math::nalgebra_utils::vec_to_dvec;
use ncollide3d::math::Point;


pub fn closest_point_on_2_lines_dvecs(u1: &DVector<f64>, u2: &DVector<f64>, v1: &DVector<f64>, v2: &DVector<f64>) -> (f64, f64) {
    let u = u2 - u1;
    let v = v2 - v1;
    let rho = v1 - u1;
    let uv = u.dot(&v);
    let uu = u.dot(&u);
    let vv = v.dot(&v);
    let urho = u.dot(&rho);
    let vrho = v.dot(&rho);

    println!("{}", uv);

    let mut vt = (vrho*uu - urho*uv) / (uv*uv - vv*uu).max(0.00000000001);
    let mut ut = (uv * vt + urho) / uu.max(0.00000000001);

    // ut = ut.max(0.0).min(1.0);
    // vt = vt.max(0.0).min(1.0);

    return (ut, vt)
}

pub fn closest_point_on_2_lines(u1: &Vec<f64>, u2: &Vec<f64>, v1: &Vec<f64>, v2: &Vec<f64>) -> (f64, f64)  {
    closest_point_on_2_lines_dvecs(&vec_to_dvec(u1), &vec_to_dvec(u2), &vec_to_dvec(v1), &vec_to_dvec(v2))
}

pub fn pt_dis_to_line_seg_dvecs(pt: &DVector<f64>, a: &DVector<f64>, b: &DVector<f64>) -> (f64, DVector<f64>) {
    let tmp = (a-b).norm();
    let mut u = (pt - a).dot( &(b - a) ) / (tmp * tmp).max(0.00000000001);
    u = u.max(0.).min(1.);
    let p = a + u*(b-a);
    let dis = (&p - pt).norm();
    return (dis, p.clone());
}

pub fn pt_dis_to_line_seg(pt: &Vec<f64>, a: &Vec<f64>, b: &Vec<f64>) -> (f64, DVector<f64>) {
    pt_dis_to_line_seg_dvecs(&vec_to_dvec(&pt), &vec_to_dvec(&a), &vec_to_dvec(&b))
}

pub fn pt_dis_to_to_line_dvecs(pt: &DVector<f64>, a: &DVector<f64>, b: &DVector<f64>) -> (f64, DVector<f64>) {
    let tmp = (a-b).norm();
    let mut u = (pt - a).dot( &(b - a) ) / (tmp * tmp).max(0.00000000001);
    let p = a + u*(b-a);
    let dis = (&p - pt).norm();
    return (dis, p.clone());
}

pub fn pt_dis_to_line(pt: &Vec<f64>, a: &Vec<f64>, b: &Vec<f64>) -> (f64, DVector<f64>) {
    pt_dis_to_to_line_dvecs(&vec_to_dvec(&pt), &vec_to_dvec(&a), &vec_to_dvec(&b))
}

pub fn pt_dis_to_line_and_seg_dvecs(pt: &DVector<f64>, a: &DVector<f64>, b: &DVector<f64>) -> (f64, DVector<f64>, f64, f64, DVector<f64>, f64) {
    let tmp = (a-b).norm();
    let mut u1 = (pt - a).dot( &(b - a) ) / (tmp * tmp).max(0.00000000001);
    let mut u2 = u1.max(0.).min(1.);
    let p1 = a + u1*(b-a);
    let p2 = a + u2*(b-a);
    let dis1 = (&p1 - pt).norm();
    let dis2 = (&p2 - pt).norm();
    return (dis1, p1.clone(), u1, dis2, p2.clone(), u2)
}

pub fn pt_dis_to_line_and_seg(pt: &Vec<f64>, a: &Vec<f64>, b: &Vec<f64>) -> (f64, DVector<f64>, f64, f64, DVector<f64>, f64) {
    pt_dis_to_line_and_seg_dvecs(&vec_to_dvec(&pt), &vec_to_dvec(&a), &vec_to_dvec(&b))
}

pub fn area_of_triangle(a: &Vector2<f64>, b: &Vector2<f64>, c: &Vector2<f64>) -> f64 {
    area_of_triangle_from_sidelengths((a-b).norm(), (b-c).norm(), (a-c).norm())
}

pub fn area_of_triangle_from_sidelengths(a_len: f64, b_len: f64, c_len: f64) -> f64 {
    let mut s = (a_len + b_len + c_len) / 2.0;
    (s*(s-a_len)*(s-b_len)*(s-c_len)).sqrt()
}

pub fn quadratic_barycentric_coordinates(pt: &Vector2<f64>, v1: &Vector2<f64>, v2: &Vector2<f64>, v3: &Vector2<f64>, v4: &Vector2<f64>)  -> (f64, f64, f64, f64) {
    let mut a = (v1 - pt);
    let mut b = (v2 - v1);
    let mut c = (v4 - v1);
    let mut d = (v1 - v2 - v4 + v3);

    let a3 = Vector3::new(a[0], a[1], 0.0);
    let b3 = Vector3::new(b[0], b[1], 0.0);
    let c3 = Vector3::new(c[0], c[1], 0.0);
    let d3 = Vector3::new(d[0], d[1], 0.0);

    let a_cross = c3.cross(&d3);
    let b_cross = c3.cross(&b3) + a3.cross(&d3);
    let c_cross = a3.cross(&b3);

    let aa = a_cross[2];
    let bb = b_cross[2];
    let cc = c_cross[2];

    let mut u1 = 0.0;
    let mut u2 = 0.0;

    if aa.abs() < 0.00000000000001 {
        u1 = -cc / bb;
        u2 = u1;
    } else {
        if bb*bb - 4.0 * aa * cc > 0.0 {
            u1 = (-bb + (bb*bb - 4.0*aa*cc).sqrt()) / 2.0*aa;
            u2 = (-bb - (bb*bb - 4.0*aa*cc).sqrt()) / 2.0*aa;
        } else {
            u1 = -1000.0;
            u2 = u1;
        }
    }

    let mut mu = -100000.0;
    if u1 >= 0.0 && u1 <= 1.0 {
        mu = u1;
    }
    if u2 >= 0.0 && u2 <= 1.0 {
        mu = u2;
    }

    let a_cross = b3.cross(&d3);
    let b_cross = b3.cross(&c3) + a3.cross(&d3);
    let c_cross = a3.cross(&c3);

    let aa = a_cross[2];
    let bb = b_cross[2];
    let cc = c_cross[2];

    let mut w1 = 0.0;
    let mut w2 = 0.0;

    if aa.abs() <  0.00000000000001 {
        w1 = -cc / bb;
        w2 = w1;
    } else {
        if bb*bb - 4.0 * aa * cc > 0.0 {
            w1 = (-bb + (bb*bb - 4.0*aa*cc).sqrt()) / 2.0*aa;
            w2 = (-bb - (bb*bb - 4.0*aa*cc).sqrt()) / 2.0*aa;
        } else {
            w1 = -1000.0;
            w2 = w1;
        }
    }

    let mut lambda = -10000.0;
    if w1 >= 0.0 && w1 <= 1.0 {
        lambda = w1;
    }
    if w2 >= 0.0 && w2 <= 1.0 {
        lambda = w2;
    }

    let mut alpha1 = (1.0-mu) * (1.0-lambda);
    let mut alpha2 = lambda * (1.0-mu);
    let mut alpha3 = mu * lambda;
    let mut alpha4 = (1.0-lambda) * mu;

    (alpha1, alpha2, alpha3, alpha4)
}

pub fn signed_volume_of_triangle_ncollide(a: &Point<f64>, b: &Point<f64>, c: &Point<f64>) -> f64 {
    let v321 = c[0]*b[1]*a[2];
    let v231 = b[0]*c[1]*a[2];
    let v312 = c[0]*a[1]*b[2];
    let v132 = a[0]*c[1]*b[2];
    let v213 = b[0]*a[1]*c[2];
    let v123 = a[0]*b[1]*c[2];

    return (1.0/6.0)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

pub fn signed_volume_of_triangle(a: &Vector3<f64>, b: &Vector3<f64>, c: &Vector3<f64>) -> f64 {
    let v321 = c[0]*b[1]*a[2];
    let v231 = b[0]*c[1]*a[2];
    let v312 = c[0]*a[1]*b[2];
    let v132 = a[0]*c[1]*b[2];
    let v213 = b[0]*a[1]*c[2];
    let v123 = a[0]*b[1]*c[2];

    return (1.0/6.0)*(-v321 + v231 + v312 - v132 - v213 + v123);
}