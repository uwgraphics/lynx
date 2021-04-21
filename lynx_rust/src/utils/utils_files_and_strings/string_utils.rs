use nalgebra::{DVector, Vector3};

pub fn usize_to_string(u: usize) -> String {
    u.to_string()
}

pub fn float_to_string(f: f64) -> String {
    f.to_string()
}

pub fn u64_to_string(u: u64) -> String { u.to_string() }

pub fn vec_of_floats_to_string(v: &Vec<f64>) -> String {
    let mut s = format!("{:?}", v);
    s.remove(0);
    s.pop();
    s = s.replace(" ", "");
    s
}

pub fn vec_of_usizes_to_string(v: &Vec<usize>) -> String {
    let mut s = format!("{:?}", v);
    s.remove(0);
    s.pop();
    s = s.replace(" ", "");
    s
}

pub fn dvec_to_string(v: &DVector<f64>) -> String {
    let mut s = format!("{:?}", v.data.as_vec());
    s.remove(0);
    s.pop();
    s = s.replace(" ", "");
    s
}

pub fn vec_of_dvecs_to_string(v: &Vec<DVector<f64>>, separator: &str) -> String {
    let l = v.len();
    if l == 0 { return "".to_string(); }

    let mut out_string = dvec_to_string(&v[0]);
    for i in 1..l {
       out_string = glue_strings(out_string, dvec_to_string(&v[i]), separator);
    }
    out_string
}

pub fn vec3_to_string(v: &Vector3<f64>) -> String {
    let mut s = format!("{:?}", v.data.to_vec());
    s.remove(0);
    s.pop();
    s = s.replace(" ", "");
    s
}

pub fn glue_strings(s1: String, s2: String, separator: &str) -> String {
    s1 + separator + s2.as_str()
}

pub fn str_option_to_string_option(s: Option<&str>) -> Option<String> {
    return if s.is_none() { None } else { Some(s.unwrap().to_string()) }
}

/*
pub fn contains_exact(s: &String, substring: &String) -> bool {
    if s.contains(&substring.as_str()) {

        let s_chars: Vec<char> = s.as_str().chars().collect();
        let substring_chars: Vec<char> = substring.as_str().chars().collect();

        let mut start_idxs = Vec::new();

        let l = s.len();
        for i in 0..l {
            if s_chars[i] == substring_chars[0] {
                start_idxs.push(i);
            }
        }

        let l = start_idxs.len();
        for i in 0..l {
            let mut curr_count = 1;
            let mut curr_s_idx = start_idxs[i].clone() + 1;

            if curr_count == substring_chars.len() { return true; }

            loop {
                if s_chars[curr_s_idx] == substring_chars[curr_count] {
                    curr_count += 1;
                    curr_s_idx += 1;
                    if curr_count == substring_chars.len() { return true; }
                } else {
                    break;
                }

                if curr_count >= s_chars.len() { break; }
            }
        }

        return false;
    } else {
        return false;
    }
}
*/