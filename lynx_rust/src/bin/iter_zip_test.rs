use rayon::prelude::*;


fn main() {

    let mut xs: Vec<_> = (1..4).collect();
    let mut ys: Vec<_> = (-4..-1).collect();
    let mut zs = vec![0; 3];

    let a = (&mut xs, &mut ys, &mut zs).into_par_iter();

    a.for_each(|(b,c,d)| {
        println!("{:?}", b);
    });


}