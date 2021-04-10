use crate::utils::utils_collisions::collision_object::CollisionObject;
use crate::utils::utils_files_and_strings::{file_utils::*, robot_folder_utils::*};
use serde::{Serialize, Deserialize};


#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct BoolCollisionCheckTensor {
    _tensor: Vec<Vec<Vec<Vec<bool>>>>,
    _dim1: usize,
    _dim2: usize,
    _dim3: usize,
    _dim4: usize,
    _m: SkipCheckForSelfCollisionMode
}

impl BoolCollisionCheckTensor {
    pub fn new_manual_inputs(dim1: usize, dim2: usize, dim3: usize, dim4: usize, m: SkipCheckForSelfCollisionMode) -> Self {
        let _tensor = vec![ vec![ vec![ vec![ false; dim4 ] ; dim3 ] ; dim2 ] ; dim1 ];
        let mut out_self =  Self { _tensor, _dim1: dim1, _dim2: dim2, _dim3: dim3, _dim4: dim4, _m: m };

        out_self._set_skips_for_self_collision_mode();

        return out_self;
    }

    pub fn new(collision_objects_group_1: &Vec<Vec<CollisionObject>>, collision_objects_group_2: &Vec<Vec<CollisionObject>>, m: SkipCheckForSelfCollisionMode) -> Self {
        let dim1 = collision_objects_group_1.len();
        let mut dim2 = 0;
        collision_objects_group_1.iter().for_each(|x| if x.len() > dim2 { dim2 = x.len() } );

        let dim3 = collision_objects_group_2.len();
        let mut dim4 = 0;
        collision_objects_group_2.iter().for_each(|x| if x.len() > dim4 { dim4 = x.len() } );

        return Self::new_manual_inputs(dim1, dim2, dim3, dim4, m);
    }

    pub fn new_empty() -> Self {
        return Self::new_manual_inputs(0,0,0,0, SkipCheckForSelfCollisionMode::NoSelfCollisions);
    }

    pub fn new_for_always_in_collision_pairs(count_collision_check_tensor: &FloatCollisionCheckTensor, always_in_collision_ratio_cutoff: Option<f64>, m: SkipCheckForSelfCollisionMode) -> Result<Self, String> {
        let mut always_in_collision_ratio_cutoff_ = 0.99; if always_in_collision_ratio_cutoff.is_some() { always_in_collision_ratio_cutoff_ = always_in_collision_ratio_cutoff.unwrap(); }

        let dim1 = count_collision_check_tensor._dim1;
        let dim2 = count_collision_check_tensor._dim2;
        let dim3 = count_collision_check_tensor._dim3;
        let dim4 = count_collision_check_tensor._dim4;

        let mut skip_collision_check_tensor = BoolCollisionCheckTensor::new_manual_inputs(dim1, dim2, dim3, dim4, m );

        for i in 0..dim1 {
            for j in 0..dim2 {
                for k in 0..dim3 {
                    for l in 0..dim4 {
                        let ratio = count_collision_check_tensor.get_mean_at_given_idxs([i,j], [k,l])?;
                        if ratio >= always_in_collision_ratio_cutoff_ {
                            skip_collision_check_tensor.add_skip([i,j], [k,l]);
                        }
                    }
                }
            }
        }

        Ok(skip_collision_check_tensor)
    }

    pub fn new_for_never_in_collision_pairs(count_collision_check_tensor: &FloatCollisionCheckTensor, never_in_collision_ratio_cutoff: Option<f64>, m: SkipCheckForSelfCollisionMode) -> Result<Self, String> {
        let mut never_in_collision_ratio_cutoff_ = 0.0; if never_in_collision_ratio_cutoff.is_some() { never_in_collision_ratio_cutoff_ = never_in_collision_ratio_cutoff.unwrap(); }

        let dim1 = count_collision_check_tensor._dim1;
        let dim2 = count_collision_check_tensor._dim2;
        let dim3 = count_collision_check_tensor._dim3;
        let dim4 = count_collision_check_tensor._dim4;

        let mut skip_collision_check_tensor = BoolCollisionCheckTensor::new_manual_inputs(dim1, dim2, dim3, dim4, m );

        for i in 0..dim1 {
            for j in 0..dim2 {
                for k in 0..dim3 {
                    for l in 0..dim4 {
                        let ratio = count_collision_check_tensor.get_mean_at_given_idxs([i,j], [k,l])?;
                        if ratio <= never_in_collision_ratio_cutoff_ {
                            skip_collision_check_tensor.add_skip([i,j], [k,l]);
                        }
                    }
                }
            }
        }

        Ok(skip_collision_check_tensor)
    }

    pub fn new_for_always_and_never_in_collision_pairs(count_collision_check_tensor: &FloatCollisionCheckTensor, always_in_collision_ratio: Option<f64>, never_in_collision_ratio: Option<f64>, m: SkipCheckForSelfCollisionMode) -> Result<Self, String> {
        let always = Self::new_for_always_in_collision_pairs(count_collision_check_tensor, always_in_collision_ratio, m.clone())?;
        let never = Self::new_for_never_in_collision_pairs(count_collision_check_tensor, never_in_collision_ratio, m.clone())?;
        return Ok( always.combine(&never)? );
    }

    pub fn load_from_file(fp_to_dir: String, file_name: String) -> Result<Self, String> {
        let json_string = read_file_contents( fp_to_dir.clone() + "/" + file_name.as_str() );
        if json_string.is_none() {
            return Err(format!("no file {:?} found when trying to load a SkipCollisionCheckTensor", fp_to_dir + "/" + file_name.as_str()));
        }

        let out_self = serde_json::from_str(&json_string.unwrap()).unwrap();
        return Ok(out_self);
    }

    pub fn load_from_file_relative_to_robot_directory(robot_name: String, partial_fp_to_dir: String, file_name: String) -> Result<Self, String> {
        let json_string = read_file_contents_relative_to_robot_directory(robot_name, partial_fp_to_dir.clone() + "/" + file_name.as_str());
        if json_string.is_none() {
            return Err(format!("no file {:?} found when trying to load a SkipCollisionCheckTensor", partial_fp_to_dir.clone() + "/" + file_name.as_str()));
        }

        let out_self = serde_json::from_str(&json_string.unwrap()).unwrap();
        return Ok(out_self);
    }

    pub fn combine(&self, other: &BoolCollisionCheckTensor) -> Result<Self, String> {
        if !(self._dim1 == other._dim1) { return Err(format!("SkipCollisionCheck combine failed because _dim1 did not match ({:?} and {:?})", self._dim1, other._dim1)); }
        if !(self._dim2 == other._dim2) { return Err(format!("SkipCollisionCheck combine failed because _dim2 did not match ({:?} and {:?})", self._dim2, other._dim2)); }
        if !(self._dim3 == other._dim3) { return Err(format!("SkipCollisionCheck combine failed because _dim3 did not match ({:?} and {:?})", self._dim3, other._dim3)); }
        if !(self._dim4 == other._dim4) { return Err(format!("SkipCollisionCheck combine failed because _dim4 did not match ({:?} and {:?})", self._dim4, other._dim4)); }

        let mut out_tensor = self._tensor.clone();

        for i in 0..self._dim1 {
            for j in 0..self._dim2 {
                for k in 0..self._dim3 {
                    for l in 0..self._dim4 {
                        out_tensor[i][j][k][l] = self.get_is_skip( [i,j], [k,l] )? || other.get_is_skip([i,j], [k, l])?;
                    }
                }
            }
        }

        return Ok( Self { _tensor: out_tensor, _dim1: self._dim1, _dim2: self._dim2, _dim3: self._dim3, _dim4: self._dim4, _m: self._m.clone() } );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _set_skips_for_self_collision_mode(&mut self) -> Result<(), String> {
        match self._m {
            SkipCheckForSelfCollisionMode::NoSelfCollisions => return Ok(()),
            _ => { }
        }

        let mut visited_already = BoolCollisionCheckTensor::new_manual_inputs(self._dim1, self._dim2, self._dim3, self._dim4, SkipCheckForSelfCollisionMode::NoSelfCollisions);

        for i in 0..self._dim1 {
            for j in 0..self._dim2 {
                for k in 0..self._dim3 {
                    for l in 0..self._dim4 {
                        match self._m {
                            SkipCheckForSelfCollisionMode::SameObjectOnly => {
                                if i == k && l == j { self.add_skip( [i,j], [k,l] ); }
                                visited_already.add_skip( [i,j], [k,l] );
                                if visited_already.get_is_skip( [k,l], [i,j] )? {
                                    self.add_skip( [i,j], [k,l] );
                                }
                            },
                            SkipCheckForSelfCollisionMode::SameObjectOrSameVector => {
                                if i == k { self.add_skip( [i,j], [k,l] ); }
                                visited_already.add_skip( [i,j], [k,l] );
                                visited_already.add_skip( [i,j], [k,l] );
                                if visited_already.get_is_skip( [k,l], [i,j] )? {
                                    self.add_skip( [i,j], [k,l] );
                                }
                            },
                            _ => { }
                        }
                    }
                }
            }
        }

        return Ok(());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_skip(&mut self, collision_objects_group_1_coords: [usize; 2], collision_objects_group_2_coords: [usize; 2]) -> Result<(), String> {
        if collision_objects_group_1_coords[0] >= self._dim1 {
            return Err(format!("invalid dimension 1 inputs in SkipCollisionCheckTensor ({:?} with _dim1 of {:?})", collision_objects_group_1_coords[0], self._dim1));
        }

        if collision_objects_group_1_coords[1] >= self._dim2 {
            return Err(format!("invalid dimension 2 inputs in SkipCollisionCheckTensor ({:?} with _dim2 of {:?})", collision_objects_group_1_coords[1], self._dim2));
        }

        if collision_objects_group_2_coords[0] >= self._dim3 {
            return Err(format!("invalid dimension 3 inputs in SkipCollisionCheckTensor ({:?} with _dim3 of {:?})", collision_objects_group_2_coords[0], self._dim3));
        }

        if collision_objects_group_2_coords[1] >= self._dim4 {
            return Err(format!("invalid dimension 4 inputs in SkipCollisionCheckTensor ({:?} with _dim4 of {:?})", collision_objects_group_2_coords[0], self._dim4));
        }

        self._tensor[collision_objects_group_1_coords[0]][collision_objects_group_1_coords[1]][collision_objects_group_2_coords[0]][collision_objects_group_2_coords[1]] = true;

        Ok(())
    }

    pub fn remove_skip(&mut self, collision_objects_group_1_coords: [usize; 2], collision_objects_group_2_coords: [usize; 2]) -> Result<(), String> {
        if collision_objects_group_1_coords[0] >= self._dim1 {
            return Err(format!("invalid dimension 1 inputs in SkipCollisionCheckTensor ({:?} with _dim1 of {:?})", collision_objects_group_1_coords[0], self._dim1));
        }

        if collision_objects_group_1_coords[1] >= self._dim2 {
            return Err(format!("invalid dimension 2 inputs in SkipCollisionCheckTensor ({:?} with _dim2 of {:?})", collision_objects_group_1_coords[1], self._dim2));
        }

        if collision_objects_group_2_coords[0] >= self._dim3 {
            return Err(format!("invalid dimension 3 inputs in SkipCollisionCheckTensor ({:?} with _dim3 of {:?})", collision_objects_group_2_coords[0], self._dim3));
        }

        if collision_objects_group_2_coords[1] >= self._dim4 {
            return Err(format!("invalid dimension 4 inputs in SkipCollisionCheckTensor ({:?} with _dim4 of {:?})", collision_objects_group_2_coords[0], self._dim4));
        }

        self._tensor[collision_objects_group_1_coords[0]][collision_objects_group_1_coords[1]][collision_objects_group_2_coords[0]][collision_objects_group_2_coords[1]] = false;

        Ok(())
    }

    pub fn add_skip_for_whole_vec(&mut self, collision_objects_group_1_vec_idx: usize, collision_objects_group_2_vec_idx: usize) -> Result<(), String> {
        if collision_objects_group_1_vec_idx >= self._dim1 {
            return Err(format!("invalid dimension 1 inputs in SkipCollisionCheckTensor ({:?} with _dim1 of {:?})", collision_objects_group_1_vec_idx, self._dim1));
        }

        if collision_objects_group_2_vec_idx >= self._dim3 {
            return Err(format!("invalid dimension 3 inputs in SkipCollisionCheckTensor ({:?} with _dim3 of {:?})", collision_objects_group_2_vec_idx, self._dim3));
        }

        for i in 0..self._dim2 {
            for j in 0..self._dim4 {
                self._tensor[collision_objects_group_1_vec_idx][i][collision_objects_group_2_vec_idx][j] = true;
            }
        }

        Ok(())
    }

    pub fn remove_skip_for_whole_vec(&mut self, collision_objects_group_1_vec_idx: usize, collision_objects_group_2_vec_idx: usize) -> Result<(), String> {
        if collision_objects_group_1_vec_idx >= self._dim1 {
            return Err(format!("invalid dimension 1 inputs in SkipCollisionCheckTensor ({:?} with _dim1 of {:?})", collision_objects_group_1_vec_idx, self._dim1));
        }

        if collision_objects_group_2_vec_idx >= self._dim3 {
            return Err(format!("invalid dimension 3 inputs in SkipCollisionCheckTensor ({:?} with _dim3 of {:?})", collision_objects_group_2_vec_idx, self._dim3));
        }

        for i in 0..self._dim2 {
            for j in 0..self._dim4 {
                self._tensor[collision_objects_group_1_vec_idx][i][collision_objects_group_2_vec_idx][j] = false;
            }
        }

        Ok(())
    }

    pub fn get_is_skip(&self, collision_objects_group_1_coords: [usize; 2], collision_objects_group_2_coords: [usize; 2]) -> Result<bool, String> {
        if collision_objects_group_1_coords[0] >= self._dim1 {
            return Err(format!("invalid dimension 1 inputs in SkipCollisionCheckTensor ({:?} with _dim1 of {:?})", collision_objects_group_1_coords[0], self._dim1));
        }

        if collision_objects_group_1_coords[1] >= self._dim2 {
            return Err(format!("invalid dimension 2 inputs in SkipCollisionCheckTensor ({:?} with _dim2 of {:?})", collision_objects_group_1_coords[1], self._dim2));
        }

        if collision_objects_group_2_coords[0] >= self._dim3 {
            return Err(format!("invalid dimension 3 inputs in SkipCollisionCheckTensor ({:?} with _dim3 of {:?})", collision_objects_group_2_coords[0], self._dim3));
        }

        if collision_objects_group_2_coords[1] >= self._dim4 {
            return Err(format!("invalid dimension 4 inputs in SkipCollisionCheckTensor ({:?} with _dim4 of {:?})", collision_objects_group_2_coords[0], self._dim4));
        }

        return Ok(self._tensor[collision_objects_group_1_coords[0]][collision_objects_group_1_coords[1]][collision_objects_group_2_coords[0]][collision_objects_group_2_coords[1]]);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn save_to_file(&self, fp_to_dir: String, file_name: String) {
        let serialized = serde_json::to_string(&self).unwrap();
        write_string_to_file(fp_to_dir, file_name, serialized, true);
    }

    pub fn save_to_file_relative_to_robot_directory(&self, robot_name: String, partial_fp_to_dir: String, file_name: String) {
        let serialized = serde_json::to_string(&self).unwrap();
        write_string_to_file_relative_to_robot_directory(robot_name, partial_fp_to_dir, file_name, serialized, true);
    }
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FloatCollisionCheckTensor {
    _tensor: Vec<Vec<Vec<Vec<f64>>>>,
    _dim1: usize,
    _dim2: usize,
    _dim3: usize,
    _dim4: usize,
    _total_num_collision_checks: f64
}

impl FloatCollisionCheckTensor {
    pub fn new_manual_inputs(dim1: usize, dim2: usize, dim3: usize, dim4: usize) -> Self {
        let _tensor = vec![ vec![ vec![ vec![ 0.0; dim4 ] ; dim3 ] ; dim2 ] ; dim1 ];
        return Self { _tensor, _dim1: dim1, _dim2: dim2, _dim3: dim3, _dim4: dim4, _total_num_collision_checks: 0.0 }
    }

    pub fn new(collision_objects_group_1: &Vec<Vec<CollisionObject>>, collision_objects_group_2: &Vec<Vec<CollisionObject>>) -> Self {
        let dim1 = collision_objects_group_1.len();
        let mut dim2 = 0;
        collision_objects_group_1.iter().for_each(|x| if x.len() > dim2 { dim2 = x.len() } );

        let dim3 = collision_objects_group_2.len();
        let mut dim4 = 0;
        collision_objects_group_2.iter().for_each(|x| if x.len() > dim4 { dim4 = x.len() } );

        return Self::new_manual_inputs(dim1, dim2, dim3, dim4);
    }

    pub fn new_empty() -> Self {
        return Self::new_manual_inputs(0,0,0,0);
    }

    pub fn load_from_file(fp_to_dir: String, file_name: String) -> Result<Self, String> {
        let json_string = read_file_contents( fp_to_dir.clone() + "/" + file_name.as_str() );
        if json_string.is_none() {
            return Err(format!("no file {:?} found when trying to load a FloatCollisionCheckTensor", fp_to_dir + "/" + file_name.as_str()));
        }

        let out_self = serde_json::from_str(&json_string.unwrap()).unwrap();
        return Ok(out_self);
    }

    pub fn load_from_file_relative_to_robot_directory(robot_name: String, partial_fp_to_dir: String, file_name: String) -> Result<Self, String> {
        let json_string = read_file_contents_relative_to_robot_directory(robot_name, partial_fp_to_dir.clone() + "/" + file_name.as_str());
        if json_string.is_none() {
            return Err(format!("no file {:?} found when trying to load a SkipCollisionCheckTensor", partial_fp_to_dir.clone() + "/" + file_name.as_str()));
        }

        let out_self = serde_json::from_str(&json_string.unwrap()).unwrap();
        return Ok(out_self);
    }

    pub fn increment_total_num_collision_checks(&mut self) {
        self._total_num_collision_checks += 1.0;
    }

    pub fn increment_count_at_given_idxs(&mut self, intersection_idxs: &Vec< [ [usize; 2]; 2] >) -> Result<(), String> {
        return self.increment_at_given_idxs_by_custom_value(intersection_idxs, 1.0);
    }

    pub fn increment_at_given_idxs_by_custom_value(&mut self, intersection_idxs: &Vec< [ [usize; 2]; 2] >, value: f64) -> Result<(), String> {
        let l = intersection_idxs.len();
        for i in 0..l {
            if intersection_idxs[i][0][0]  >= self._dim1 {
                return Err(format!("invalid dimension 1 inputs in SkipCollisionCheckTensor ({:?} with _dim1 of {:?})", intersection_idxs[i][0][0], self._dim1));
            }

            if intersection_idxs[i][0][1] >= self._dim2 {
                return Err(format!("invalid dimension 2 inputs in SkipCollisionCheckTensor ({:?} with _dim2 of {:?})",  intersection_idxs[i][0][1], self._dim2));
            }

            if intersection_idxs[i][1][0] >= self._dim3 {
                return Err(format!("invalid dimension 3 inputs in SkipCollisionCheckTensor ({:?} with _dim3 of {:?})", intersection_idxs[i][1][0], self._dim3));
            }

            if intersection_idxs[i][1][1] >= self._dim4 {
                return Err(format!("invalid dimension 4 inputs in SkipCollisionCheckTensor ({:?} with _dim4 of {:?})", intersection_idxs[i][1][1], self._dim4));
            }

            self._tensor[ intersection_idxs[i][0][0] ][ intersection_idxs[i][0][1] ][ intersection_idxs[i][1][0] ][ intersection_idxs[i][1][1] ] += value;
        }

        self._total_num_collision_checks += 1.0;

        return Ok(());
    }

    pub fn increment_at_given_single_idxs_by_custom_value(&mut self, intersection_idxs: &[ [usize; 2]; 2], value: f64) -> Result<(), String> {
            if intersection_idxs[0][0]  >= self._dim1 {
                return Err(format!("invalid dimension 1 inputs in SkipCollisionCheckTensor ({:?} with _dim1 of {:?})", intersection_idxs[0][0], self._dim1));
            }

            if intersection_idxs[0][1] >= self._dim2 {
                return Err(format!("invalid dimension 2 inputs in SkipCollisionCheckTensor ({:?} with _dim2 of {:?})",  intersection_idxs[0][1], self._dim2));
            }

            if intersection_idxs[1][0] >= self._dim3 {
                return Err(format!("invalid dimension 3 inputs in SkipCollisionCheckTensor ({:?} with _dim3 of {:?})", intersection_idxs[1][0], self._dim3));
            }

            if intersection_idxs[1][1] >= self._dim4 {
                return Err(format!("invalid dimension 4 inputs in SkipCollisionCheckTensor ({:?} with _dim4 of {:?})", intersection_idxs[1][1], self._dim4));
            }

            self._tensor[ intersection_idxs[0][0] ][ intersection_idxs[0][1] ][ intersection_idxs[1][0] ][ intersection_idxs[1][1] ] += value;


        return Ok(());
    }

    pub fn get_mean_at_given_idxs(&self, collision_objects_group_1_coords: [usize; 2], collision_objects_group_2_coords: [usize; 2]) -> Result<f64, String> {
        if collision_objects_group_1_coords[0] >= self._dim1 {
            return Err(format!("invalid dimension 1 inputs in SkipCollisionCheckTensor ({:?} with _dim1 of {:?})", collision_objects_group_1_coords[0], self._dim1));
        }

        if collision_objects_group_1_coords[1] >= self._dim2 {
            return Err(format!("invalid dimension 2 inputs in SkipCollisionCheckTensor ({:?} with _dim2 of {:?})", collision_objects_group_1_coords[1], self._dim2));
        }

        if collision_objects_group_2_coords[0] >= self._dim3 {
            return Err(format!("invalid dimension 3 inputs in SkipCollisionCheckTensor ({:?} with _dim3 of {:?})", collision_objects_group_2_coords[0], self._dim3));
        }

        if collision_objects_group_2_coords[1] >= self._dim4 {
            return Err(format!("invalid dimension 4 inputs in SkipCollisionCheckTensor ({:?} with _dim4 of {:?})", collision_objects_group_2_coords[0], self._dim4));
        }

        let val = self._tensor[collision_objects_group_1_coords[0]][collision_objects_group_1_coords[1]][collision_objects_group_2_coords[0]][collision_objects_group_2_coords[1]];
        let ratio = val / self._total_num_collision_checks;

        return Ok(ratio);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn save_to_file(&self, fp_to_dir: String, file_name: String) {
        let serialized = serde_json::to_string(&self).unwrap();
        write_string_to_file(fp_to_dir, file_name, serialized, true);
    }

    pub fn save_to_file_relative_to_robot_directory(&self, robot_name: String, partial_fp_to_dir: String, file_name: String) {
        let serialized = serde_json::to_string(&self).unwrap();
        write_string_to_file_relative_to_robot_directory(robot_name, partial_fp_to_dir, file_name, serialized, true);
    }
}



#[derive(Clone, Serialize, Deserialize, Debug)]
pub enum SkipCheckForSelfCollisionMode {
    SameObjectOnly,
    SameObjectOrSameVector,
    NoSelfCollisions
}