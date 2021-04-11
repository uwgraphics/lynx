use std::time::Instant;
use proc_macro2::{Ident, Span};

#[macro_export]
macro_rules! vec {
    ( $( $x:expr ),* ) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x);
            )*
            temp_vec
        }
    };
}

#[macro_export] #[macro_use]
macro_rules! calculate {
  (eval $e:expr) => {{
    let val: usize = $e;
    println!("{} = {}", stringify!{$e}, val);
    val
  }};
}

macro_rules! test {
    ($x:expr) => {
        {
            println!("{}", $x);
            $x
        }
    };
}

macro_rules! test2 {
    ($x:expr, $y:expr, $i:ident) => {
        $y.$i = $x;
    };
}

macro_rules! test3 {
    ($T:expr, $i:ident) => {
        {
            &mut $T.$i
        };
    }
}

macro_rules! test4 {
    ($i: ident) => {
        {$i}
    };
}

pub struct T {
    pub _hi: f64,
    pub _bye: usize
}


fn main() {
    let mut t = T{_hi: 5.0, _bye: 2};

    // let ident = Ident::new("_hi", Span::call_site());

    // test3!(t, _hi);

    // test4(_hi);

    /*
    let r = test3!(t, _hi);
    println!("{:?}", r);
    *r = 6.0;
    println!("{:?}", t._hi);
    */
}