extern crate lynx_lib;
use lynx_lib::utils::utils_path_planning::utils_paths::linear_spline_path::LinearSplinePath;

fn main() {

    let mut l1 = LinearSplinePath::new_from_vecs( vec![ vec![0.] ] );
    let mut l2 = LinearSplinePath::new_from_vecs( vec![  ] );

    let l3 = l1.combine_with_calculated_order(&l2);

}