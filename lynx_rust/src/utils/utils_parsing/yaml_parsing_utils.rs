use yaml_rust::{YamlLoader, Yaml};
use crate::utils::utils_files_and_strings::file_utils::*;
use termion::{color, style};

pub fn get_yaml_obj(fp: String) -> Result<Vec<Yaml>, String> {
    let contents = read_file_contents(fp.clone());

    if contents.is_none() {
        return Ok(Vec::new());
    }

    let docs = YamlLoader::load_from_str(contents.unwrap().as_str());
    if docs.is_err() {
        println!("{}{}Looks like the Yaml formatting is wrong in file {:?}.  Please fix and try again. {}", style::Bold, color::Fg(color::Red), fp, style::Reset);
        return Err(format!("Error in get_yaml_obj.  Looks like yaml with path {:?} was invalid.", fp));
    }

    return Ok(docs.ok().unwrap());
}



pub fn parse_list_of_floats_1(y: &Yaml) -> Vec<f64> {
    let mut ret: Vec<f64> = Vec::new();

    let v1 = y.as_vec().unwrap();
    for i in 0..v1.len() {
        ret.push(v1[i].as_f64().unwrap());
    }
    ret
}

pub fn parse_list_of_floats_2(y: &Yaml) -> Vec<Vec<f64>> {
    let mut ret: Vec<Vec<f64>> = Vec::new();

    let v1 = y.as_vec().unwrap();
    for i in 0..v1.len() {
        ret.push( parse_list_of_floats_1(&v1[i]) );
    }
    ret
}

pub fn parse_list_of_floats_3(y: &Yaml) -> Vec<Vec<Vec<f64>>> {
    let mut ret: Vec<Vec<Vec<f64>>> = Vec::new();

    let v1 = y.as_vec().unwrap();
    for i in 0..v1.len() {
        ret.push( parse_list_of_floats_2(&v1[i]) );
    }
    ret
}

pub fn parse_list_of_floats_4(y: &Yaml) -> Vec<Vec<Vec<Vec<f64>>>> {
    let mut ret: Vec<Vec<Vec<Vec<f64>>>> = Vec::new();

    let v1 = y.as_vec().unwrap();
    for i in 0..v1.len() {
        ret.push( parse_list_of_floats_3(&v1[i]) );
    }
    ret
}