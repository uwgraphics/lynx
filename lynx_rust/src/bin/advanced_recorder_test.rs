extern crate lynx_lib;
use lynx_lib::utils::utils_vars::lynx_vars::LynxVars;
use nalgebra::DVector;
use serde::{Serialize, Deserialize};


fn print_type_of<T>(_: &T) {
    println!("{}", std::any::type_name::<T>())
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Test {
    a: f64
}

macro_rules! get_parsed_json_variable {
    ($serde_string: expr, $type: ident) => {
        {
            // &mut $lynx_vars.$i
            let a : $type = serde_json::from_str($serde_string).unwrap();
            a
        };
    }
}

fn main() {
    /*
    let a = vec![vec![0,1,2]];

    let l = LynxVars::new_empty();

    let t = Test { a: 1.0 };

    let s = stringify!(a);

    // println!("{:?}", s);
    print_type_of(&a);

    let mut out_str = "".to_string();

    let serialized = serde_json::to_string(&a).unwrap();
    println!("{:?}", serialized);

    // let b: alloc::vec::Vec<alloc::vec::Vec<i32>> = a.clone();

    let b : Vec<Vec<u64>> = serde_json::from_str(&serialized).unwrap();

    println!("{:?}", b);
    */
    // let a = Test {a: 1.0};
    let a = DVector::from_element(5, 0.0);
    let b = DVector::from_element(6, 3.0);
    let c : Option<usize> = Some(5);
    let serialized = serde_json::to_string(&(a, b, c)).unwrap();
    // let b : Vec<Vec<u64>> = serde_json::from_str(&serialized).unwrap();

    println!("{:?}", serialized);

    type v = (DVector<f64>, DVector<f64>, Option<usize>);
    let b = get_parsed_json_variable!(&serialized, v);
    println!("{:?}", b);
}