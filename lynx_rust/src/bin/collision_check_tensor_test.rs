extern crate lynx_lib;
use lynx_lib::utils::utils_collisions::collision_check_tensor::*;


fn main() -> Result<(), String> {

    let mut c = BoolCollisionCheckTensor::new_manual_inputs(2, 5, 2, 5, SkipCheckForSelfCollisionMode::SameObjectOrSameVector);

    println!("{:?}", c.get_is_skip([1,2], [0,3]));

    Ok(())
}