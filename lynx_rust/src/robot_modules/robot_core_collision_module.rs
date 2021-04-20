use crate::utils::utils_collisions::{collision_object::*, collision_check_tensor::*, collision_environment::CollisionEnvironment, collision_object_utils::*};
use crate::robot_modules::{robot_configuration_module::RobotConfigurationModule, robot_dof_module::RobotDOFModule, robot_fk_module::*, robot_bounds_module::RobotBoundsModule, robot_triangle_mesh_collision_module::RobotTriangleMeshCollisionModule};
use crate::utils::utils_preprocessing::mesh_preprocessing_utils::*;
use crate::utils::utils_files_and_strings::{file_utils::*, robot_folder_utils::*};
use crate::utils::utils_collisions::triangle_mesh_engine::TriMeshEngine;
use crate::utils::utils_files_and_strings::string_utils::usize_to_string;
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
use termion::{style, color};

#[derive(Clone)]
pub struct RobotCoreCollisionModule {
    _link_obbs: Vec<Vec<CollisionObject>>,
    _link_convex_shapes: Vec<Vec<CollisionObject>>,
    _link_obb_subcomponents: Vec<Vec<CollisionObject>>,
    _link_convex_shape_subcomponents: Vec<Vec<CollisionObject>>,
    _link_obbs_skip_collision_check_tensor: BoolCollisionCheckTensor,
    _link_convex_shapes_skip_collision_check_tensor: BoolCollisionCheckTensor,
    _link_obb_subcomponents_skip_collision_check_tensor: BoolCollisionCheckTensor,
    _link_convex_shape_subcomponents_skip_collision_check_tensor: BoolCollisionCheckTensor,
    _link_obbs_average_distance_tensor: FloatCollisionCheckTensor,
    _link_convex_shapes_average_distance_tensor: FloatCollisionCheckTensor,
    _link_obb_subcomponents_average_distance_tensor: FloatCollisionCheckTensor,
    _link_convex_shape_subcomponents_average_distance_tensor: FloatCollisionCheckTensor,
    _robot_name_copy: String
}

impl RobotCoreCollisionModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule, robot_fk_module: &RobotFKModule, robot_bounds_module: &RobotBoundsModule) -> Result<Self, String> {
        let _link_obbs = Vec::new();
        let _link_convex_shapes = Vec::new();
        let _link_obb_subcomponents = Vec::new();
        let _link_convex_shape_subcomponents = Vec::new();

        let _link_obbs_skip_collision_check_tensor = BoolCollisionCheckTensor::new_empty();
        let _link_convex_shapes_skip_collision_check_tensor = BoolCollisionCheckTensor::new_empty();
        let _link_obb_subcomponents_skip_collision_check_tensor = BoolCollisionCheckTensor::new_empty();
        let _link_convex_shape_subcomponents_skip_collision_check_tensor = BoolCollisionCheckTensor::new_empty();

        let _link_obbs_average_distance_tensor = FloatCollisionCheckTensor::new_empty();
        let _link_convex_shapes_average_distance_tensor = FloatCollisionCheckTensor::new_empty();
        let _link_obb_subcomponents_average_distance_tensor = FloatCollisionCheckTensor::new_empty();
        let _link_convex_shape_subcomponents_average_distance_tensor = FloatCollisionCheckTensor::new_empty();

        let _robot_name_copy = robot_configuration_module.robot_model_module.robot_name.clone();

        let mut out_self = Self { _link_obbs, _link_convex_shapes, _link_obb_subcomponents, _link_convex_shape_subcomponents,
        _link_obbs_skip_collision_check_tensor, _link_convex_shapes_skip_collision_check_tensor, _link_obb_subcomponents_skip_collision_check_tensor,
            _link_convex_shape_subcomponents_skip_collision_check_tensor, _link_obbs_average_distance_tensor,
            _link_convex_shapes_average_distance_tensor, _link_obb_subcomponents_average_distance_tensor,
            _link_convex_shape_subcomponents_average_distance_tensor, _robot_name_copy };

        Self::_create_link_convex_shapes_if_need_be(robot_configuration_module)?;
        Self::_create_link_convex_subcomponents_if_need_be(robot_configuration_module)?;

        out_self._create_link_obbs(robot_configuration_module)?;
        out_self._create_link_convex_shapes(robot_configuration_module)?;
        out_self._create_link_obb_subcomponents(robot_configuration_module)?;
        out_self._create_link_convex_shape_subcomponents(robot_configuration_module)?;

        let num_samples = 100_000;
        out_self._load_or_create_link_skip_collision_check_tensor(&LinkGeometryType::OBBs, robot_fk_module, robot_bounds_module, num_samples, false);
        out_self._load_or_create_link_skip_collision_check_tensor(&LinkGeometryType::ConvexShapes, robot_fk_module, robot_bounds_module, num_samples, false);
        out_self._load_or_create_link_skip_collision_check_tensor(&LinkGeometryType::OBBSubcomponents, robot_fk_module, robot_bounds_module, num_samples, false);
        out_self._load_or_create_link_skip_collision_check_tensor(&LinkGeometryType::ConvexShapeSubcomponents, robot_fk_module, robot_bounds_module, num_samples, false);

        let num_samples = 1000;
        out_self._load_or_create_link_average_distance_tensor(&LinkGeometryType::OBBs, robot_fk_module, robot_bounds_module, num_samples, false);
        out_self._load_or_create_link_average_distance_tensor(&LinkGeometryType::ConvexShapes, robot_fk_module, robot_bounds_module, num_samples, false);
        out_self._load_or_create_link_average_distance_tensor(&LinkGeometryType::OBBSubcomponents, robot_fk_module, robot_bounds_module, num_samples, false);
        out_self._load_or_create_link_average_distance_tensor(&LinkGeometryType::ConvexShapeSubcomponents, robot_fk_module, robot_bounds_module, num_samples, false);

        out_self._set_dead_link_collision_objects_as_inactive(robot_configuration_module);

        return Ok(out_self);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn self_intersect_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<IntersectCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return intersect_check_between_multiple_collision_objects(self.get_link_geometry_collision_objects_ref(&link_geometry_type), self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected, Some(self._get_skip_collision_check_tensor_ref(&link_geometry_type)));
    }

    pub fn self_distance_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected_intersection: bool) -> Result<DistanceCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return distance_check_between_multiple_collision_objects(self.get_link_geometry_collision_objects_ref(&link_geometry_type), self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected_intersection, Some(self._get_skip_collision_check_tensor_ref(&link_geometry_type)), Some(self._get_average_distance_tensor_ref(&link_geometry_type)));
    }

    pub fn self_contact_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected_intersection: bool, margin: Option<f64>) -> Result<ContactCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return contact_check_between_multiple_collision_objects(self.get_link_geometry_collision_objects_ref(&link_geometry_type), self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected_intersection, margin, Some(self._get_skip_collision_check_tensor_ref(&link_geometry_type)), Some(self._get_average_distance_tensor_ref(&link_geometry_type)));
    }

    pub fn self_intersect_check_subset(&mut self, subset_check_idxs: &Vec<[[usize; 2]; 2]>, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected_intersection: bool) -> Result<IntersectCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return intersect_check_between_multiple_collision_objects_subset(subset_check_idxs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected_intersection, Some(self._get_skip_collision_check_tensor_ref(&link_geometry_type)));
    }

    pub fn self_distance_check_subset(&mut self, subset_check_idxs: &Vec<[[usize; 2]; 2]>, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected_intersection: bool) -> Result<DistanceCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return distance_check_between_multiple_collision_objects_subset(subset_check_idxs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected_intersection, Some(self._get_skip_collision_check_tensor_ref(&link_geometry_type)), Some(self._get_average_distance_tensor_ref(&link_geometry_type)));
    }

    pub fn self_contact_check_subset(&mut self, subset_check_idxs: &Vec<[[usize; 2]; 2]>, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected_intersection: bool, margin: Option<f64>) -> Result<ContactCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return contact_check_between_multiple_collision_objects_subset(subset_check_idxs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected_intersection, margin, Some(self._get_skip_collision_check_tensor_ref(&link_geometry_type)), Some(self._get_average_distance_tensor_ref(&link_geometry_type)));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn environment_intersect_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, collision_environment: &CollisionEnvironment, stop_at_first_detected: bool) -> Result<IntersectCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return intersect_check_between_multiple_collision_objects(&collision_environment.environment_obbs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected, None);
    }

    pub fn environment_distance_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, collision_environment: &CollisionEnvironment, stop_at_first_detected: bool) -> Result<DistanceCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return distance_check_between_multiple_collision_objects(&collision_environment.environment_obbs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected, None, None);
    }

    pub fn environment_contact_check(&mut self, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, collision_environment: &CollisionEnvironment, stop_at_first_detected: bool, margin: Option<f64>) -> Result<ContactCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return contact_check_between_multiple_collision_objects(&collision_environment.environment_obbs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected, margin, None, None);
    }

    pub fn environment_intersect_check_subset(&mut self, subset_check_idxs: &Vec<[[usize; 2]; 2]>, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, collision_environment: &CollisionEnvironment, stop_at_first_detected: bool) -> Result<IntersectCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return intersect_check_between_multiple_collision_objects_subset(subset_check_idxs, &collision_environment.environment_obbs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected, None);
    }

    pub fn environment_distance_check_subset(&mut self, subset_check_idxs: &Vec<[[usize; 2]; 2]>, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, collision_environment: &CollisionEnvironment, stop_at_first_detected: bool) -> Result<DistanceCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return distance_check_between_multiple_collision_objects_subset(subset_check_idxs, &collision_environment.environment_obbs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected, None, None);
    }

    pub fn environment_contact_check_subset(&mut self, subset_check_idxs: &Vec<[[usize; 2]; 2]>, fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, collision_environment: &CollisionEnvironment, stop_at_first_detected: bool, margin: Option<f64>) -> Result<ContactCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        return contact_check_between_multiple_collision_objects_subset(subset_check_idxs, &collision_environment.environment_obbs, self.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected, margin, None, None);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn other_robot_intersect_check(&mut self, other_robot_core_collision_module: &mut RobotCoreCollisionModule, fk_res: &RobotFKResult, other_fk_res: &RobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<IntersectCheckMultipleResult, String> {
        self.set_poses_on_links(fk_res, &link_geometry_type);
        other_robot_core_collision_module.set_poses_on_links(other_fk_res, &link_geometry_type);

        return intersect_check_between_multiple_collision_objects(self.get_link_geometry_collision_objects_ref(&link_geometry_type), other_robot_core_collision_module.get_link_geometry_collision_objects_ref(&link_geometry_type), stop_at_first_detected, None);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn revert_skip_collision_check_tensors(&mut self) -> Result<(), String> {
        let load_result1 = BoolCollisionCheckTensor::load_from_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_obbs_skip_collision_check_tensor_permanent.json".to_string());
        if load_result1.is_err() { return Err(format!("could not revert skip collison check tensors because link_obbs_skip_collision_check_tensor_permanent.json file is missing.  Delete the whole link_skip_collision_check_tensor folder and restart to let tensors recalculate from scratch.")) }

        let load_result2 = BoolCollisionCheckTensor::load_from_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_convex_shapes_skip_collision_check_tensor_permanent.json".to_string());
        if load_result2.is_err() { return Err(format!("could not revert skip collison check tensors because link_convex_shapes_skip_collision_check_tensor_permanent.json file is missing.  Delete the whole link_skip_collision_check_tensor folder and restart to let tensors recalculate from scratch.")) }

        let load_result3 = BoolCollisionCheckTensor::load_from_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_obb_subcomponents_skip_collision_check_tensor_permanent.json".to_string());
        if load_result3.is_err() { return Err(format!("could not revert skip collison check tensors because link_obb_subcomponents_skip_collision_check_tensor_permanent.json file is missing.  Delete the whole link_skip_collision_check_tensor folder and restart to let tensors recalculate from scratch.")) }

        let load_result4 = BoolCollisionCheckTensor::load_from_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_convex_shape_subcomponents_skip_collision_check_tensor_permanent.json".to_string());
        if load_result4.is_err() { return Err(format!("could not revert skip collison check tensors because link_convex_shape_subcomponents_skip_collision_check_tensor_permanent.json file is missing.  Delete the whole link_skip_collision_check_tensor folder and restart to let tensors recalculate from scratch.")) }

        self._link_obbs_skip_collision_check_tensor = load_result1.ok().unwrap();
        self._link_obbs_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_obbs_skip_collision_check_tensor.json".to_string());

        self._link_convex_shapes_skip_collision_check_tensor = load_result2.ok().unwrap();
        self._link_convex_shapes_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_convex_shapes_skip_collision_check_tensor.json".to_string());

        self._link_obb_subcomponents_skip_collision_check_tensor = load_result3.ok().unwrap();
        self._link_obb_subcomponents_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_obb_subcomponents_skip_collision_check_tensor.json".to_string());

        self._link_convex_shape_subcomponents_skip_collision_check_tensor = load_result4.ok().unwrap();
        self._link_convex_shape_subcomponents_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), "autogenerated_metadata/link_skip_collision_check_tensors".to_string(), "link_convex_shape_subcomponents_skip_collision_check_tensor.json".to_string());

        Ok(())
    }

    pub fn add_not_in_collision_example(&mut self, fk_res: &RobotFKResult) -> Result<(), String> {
        let link_geometry_types = vec![ LinkGeometryType::OBBs,  LinkGeometryType::ConvexShapes,  LinkGeometryType::OBBSubcomponents,  LinkGeometryType::ConvexShapeSubcomponents ];

        for lgt in link_geometry_types {
            let res = self.self_intersect_check(fk_res, lgt.clone(), false)?;
            match res {
                IntersectCheckMultipleResult::IntersectionFound(i) => {
                    for idxs in i.get_intersection_idxs() {
                        let name1 = self.get_link_geometry_collision_objects_ref(&lgt)[idxs[0][0]][idxs[0][1]].name.clone();
                        let name2 = self.get_link_geometry_collision_objects_ref(&lgt)[idxs[1][0]][idxs[1][1]].name.clone();
                        println!("{}{}Adding [{:?}, {:?}] as skip pair for {:?} model.  {}", color::Fg(color::Blue), style::Bold, name1, name2, lgt, style::Reset);
                        self._get_skip_collision_check_tensor_mut_ref(&lgt).add_skip(idxs[0], idxs[1]);
                    }
                },
                IntersectCheckMultipleResult::NoIntersectionsFound(i) => {
                    println!("{}{}Everything looked fine for {:?} model given this example.  {}", color::Fg(color::Green), style::Bold, lgt, style::Reset);
                }
            }

            self._save_link_skip_tensor_to_file(&lgt);
        }

        Ok(())
    }

    pub fn add_not_in_collision_example_with_fk(&mut self, not_in_collision_state: &Vec<f64>, robot_fk_module: &RobotFKModule) -> Result<(), String> {
        let fk_res = robot_fk_module.compute_fk_vec(not_in_collision_state)?;
        return self.add_not_in_collision_example(&fk_res);
    }

    pub fn add_all_zeros_config_as_not_in_collision_example(&mut self, robot_fk_module: &RobotFKModule) -> Result<(), String> {
        let fk_res = robot_fk_module.compute_fk_on_all_zeros_config()?;
        return self.add_not_in_collision_example(&fk_res);
    }

    pub fn add_manual_collision_check_skip_between_links(&mut self, robot_configuration_module: &RobotConfigurationModule, link_name_1: String, link_name_2: String) -> Result<(), String> {
        let link_idx_1 = robot_configuration_module.robot_model_module.get_link_idx_from_name(&link_name_1);
        if link_idx_1.is_none() { return Err(format!("{:?} was not a valid link name.  ", link_name_1)); }

        let link_idx_2 = robot_configuration_module.robot_model_module.get_link_idx_from_name(&link_name_2);
        if link_idx_2.is_none() { return Err(format!("{:?} was not a valid link name.  ", link_name_2)); }

        let link_geometry_types = vec![ LinkGeometryType::OBBs,  LinkGeometryType::ConvexShapes,  LinkGeometryType::OBBSubcomponents,  LinkGeometryType::ConvexShapeSubcomponents ];

        for lgt in link_geometry_types {
            let res = self._get_skip_collision_check_tensor_mut_ref(&lgt).add_skip_for_whole_vec(link_idx_1.unwrap(), link_idx_2.unwrap());
            if res.is_err() { return Err( format!("error when adding manual collision check skip between links: {:?}", res.err()) ) }
            self._save_link_skip_tensor_to_file(&lgt);
        }

        Ok(())
    }

    pub fn remove_manual_collision_check_skip_between_links(&mut self, robot_configuration_module: &RobotConfigurationModule, link_name_1: String, link_name_2: String) -> Result<(), String> {
        let link_idx_1 = robot_configuration_module.robot_model_module.get_link_idx_from_name(&link_name_1);
        if link_idx_1.is_none() { return Err(format!("{:?} was not a valid link name.  ", link_name_1)); }

        let link_idx_2 = robot_configuration_module.robot_model_module.get_link_idx_from_name(&link_name_2);
        if link_idx_2.is_none() { return Err(format!("{:?} was not a valid link name.  ", link_name_2)); }

        let link_geometry_types = vec![ LinkGeometryType::OBBs,  LinkGeometryType::ConvexShapes,  LinkGeometryType::OBBSubcomponents,  LinkGeometryType::ConvexShapeSubcomponents ];

        for lgt in link_geometry_types {
            let res = self._get_skip_collision_check_tensor_mut_ref(&lgt).remove_skip_for_whole_vec(link_idx_1.unwrap(), link_idx_2.unwrap());
            if res.is_err() { return Err( format!("error when removing manual collision check skip between links: {:?}", res.err()) ) }
            self._save_link_skip_tensor_to_file(&lgt);
        }

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn accuracy_check(&mut self, robot_fk_module: &RobotFKModule, robot_bounds_module: &RobotBoundsModule, robot_triangle_mesh_collision_module: &mut RobotTriangleMeshCollisionModule, num_samples: usize) -> Result<(), String> {
        let mut total_num = 0.0;

        let mut obbs_true_positive_num = 0.0;
        let mut obbs_true_negative_num = 0.0;
        let mut obbs_false_positive_num = 0.0;
        let mut obbs_false_negative_num = 0.0;

        let mut convex_shapes_true_positive_num = 0.0;
        let mut convex_shapes_true_negative_num = 0.0;
        let mut convex_shapes_false_positive_num = 0.0;
        let mut convex_shapes_false_negative_num = 0.0;

        let mut obb_subcomponents_true_positive_num = 0.0;
        let mut obb_subcomponents_true_negative_num = 0.0;
        let mut obb_subcomponents_false_positive_num = 0.0;
        let mut obb_subcomponents_false_negative_num = 0.0;

        let mut convex_shape_subcomponents_true_positive_num = 0.0;
        let mut convex_shape_subcomponents_true_negative_num = 0.0;
        let mut convex_shape_subcomponents_false_positive_num = 0.0;
        let mut convex_shape_subcomponents_false_negative_num = 0.0;

        println!("Robot core collision module accuracy test");
        for i in 0..num_samples {
            println!("   sample {:?} of {:?}", i, num_samples);
            total_num += 1.0;
            let sample = robot_bounds_module.uniform_sample_from_bounds();
            let fk_res = robot_fk_module.compute_fk(&sample)?;

            let triangle_mesh_res = robot_triangle_mesh_collision_module.self_intersect_check(&fk_res, true)?;

            let obbs_res = self.self_intersect_check(&fk_res, LinkGeometryType::OBBs, true)?;
            let convex_shapes_res = self.self_intersect_check(&fk_res, LinkGeometryType::ConvexShapes, true)?;
            let obb_subcomponents_res = self.self_intersect_check(&fk_res, LinkGeometryType::OBBSubcomponents, true)?;
            let convex_shapes_subcomponents_res = self.self_intersect_check(&fk_res, LinkGeometryType::ConvexShapeSubcomponents, true)?;

            match triangle_mesh_res {
                IntersectCheckMultipleResult::IntersectionFound(i) => {

                    match obbs_res {
                        IntersectCheckMultipleResult::IntersectionFound(_) =>  { obbs_true_positive_num += 1.0 },
                        IntersectCheckMultipleResult::NoIntersectionsFound(_) => { obbs_false_negative_num += 1.0; }
                    }

                    match convex_shapes_res {
                        IntersectCheckMultipleResult::IntersectionFound(_) =>    { convex_shapes_true_positive_num += 1.0 },
                        IntersectCheckMultipleResult::NoIntersectionsFound(_) => { convex_shapes_false_negative_num += 1.0; }
                    }

                    match obb_subcomponents_res {
                        IntersectCheckMultipleResult::IntersectionFound(_) =>    { obb_subcomponents_true_positive_num += 1.0 },
                        IntersectCheckMultipleResult::NoIntersectionsFound(_) => { obb_subcomponents_false_negative_num += 1.0; }
                    }

                    match convex_shapes_subcomponents_res {
                        IntersectCheckMultipleResult::IntersectionFound(_) =>    { convex_shape_subcomponents_true_positive_num += 1.0 },
                        IntersectCheckMultipleResult::NoIntersectionsFound(_) => { convex_shape_subcomponents_false_negative_num += 1.0; }
                    }

                },
                IntersectCheckMultipleResult::NoIntersectionsFound(i) => {

                    match obbs_res {
                        IntersectCheckMultipleResult::IntersectionFound(_) =>  { obbs_false_positive_num += 1.0 },
                        IntersectCheckMultipleResult::NoIntersectionsFound(_) => { obbs_true_negative_num += 1.0; }
                    }

                    match convex_shapes_res {
                        IntersectCheckMultipleResult::IntersectionFound(_) =>    { convex_shapes_false_positive_num += 1.0 },
                        IntersectCheckMultipleResult::NoIntersectionsFound(_) => { convex_shapes_true_negative_num += 1.0; }
                    }

                    match obb_subcomponents_res {
                        IntersectCheckMultipleResult::IntersectionFound(_) =>    { obb_subcomponents_false_positive_num += 1.0 },
                        IntersectCheckMultipleResult::NoIntersectionsFound(_) => { obb_subcomponents_true_negative_num += 1.0; }
                    }

                    match convex_shapes_subcomponents_res {
                        IntersectCheckMultipleResult::IntersectionFound(_) =>    { convex_shape_subcomponents_false_positive_num += 1.0 },
                        IntersectCheckMultipleResult::NoIntersectionsFound(_) => { convex_shape_subcomponents_true_negative_num += 1.0; }
                    }

                }
            }

        }

        println!("OBBs results ---> true positive rate: {}, true negative rate: {}, false positive rate: {}, false negative rate: {}", obbs_true_positive_num/total_num, obbs_true_negative_num/total_num, obbs_false_positive_num/total_num, obbs_false_negative_num/total_num);
        println!("Convex Shapes results ---> true positive rate: {}, true negative rate: {}, false positive rate: {}, false negative rate: {}", convex_shapes_true_positive_num/total_num, convex_shapes_true_negative_num/total_num, convex_shapes_false_positive_num/total_num, convex_shapes_false_negative_num/total_num);
        println!("OBB Subcomponents results ---> true positive rate: {}, true negative rate: {}, false positive rate: {}, false negative rate: {}", obb_subcomponents_true_positive_num/total_num, obb_subcomponents_true_negative_num/total_num, obb_subcomponents_false_positive_num/total_num, obb_subcomponents_false_negative_num/total_num);
        println!("Convex Shape Subcomponents results ---> true positive rate: {}, true negative rate: {}, false positive rate: {}, false negative rate: {}", convex_shape_subcomponents_true_positive_num/total_num, convex_shape_subcomponents_true_negative_num/total_num, convex_shape_subcomponents_false_positive_num/total_num, convex_shape_subcomponents_false_negative_num/total_num);

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _load_or_create_link_skip_collision_check_tensor(&mut self, link_geometry_type: &LinkGeometryType, robot_fk_module: &RobotFKModule, robot_bounds_module: &RobotBoundsModule, num_samples: usize, create_new_no_matter_what: bool) -> Result<(), String> {
        let load_result = BoolCollisionCheckTensor::load_from_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_filename(link_geometry_type));
        if load_result.is_ok() && !create_new_no_matter_what {
            self._set_link_skip_tensor(load_result.ok().unwrap(), link_geometry_type);
            return Ok(());
        }

        let mut count_collision_check_tensor = FloatCollisionCheckTensor::new(&self.get_link_geometry_collision_objects_ref(link_geometry_type), &self.get_link_geometry_collision_objects_ref(link_geometry_type));
        let mut initial_skip_collision_check_tensor = BoolCollisionCheckTensor::new(&self.get_link_geometry_collision_objects_ref(link_geometry_type), &self.get_link_geometry_collision_objects_ref(link_geometry_type), self._get_skip_check_for_self_collision_mode(link_geometry_type));

        for i in 0..num_samples {
            if i % 100 == 0 {
                println!("{}{}Calculating skip collision tensor for {:?}: sample {} of {} {}", color::Fg(color::Blue), style::Bold, link_geometry_type, i, num_samples, style::Reset);
            }

            let sample = robot_bounds_module.uniform_sample_from_bounds();
            let fk_res = robot_fk_module.compute_fk(&sample)?;
            self.set_poses_on_links(&fk_res, link_geometry_type);

            let intersect_check_multiple_result = intersect_check_between_multiple_collision_objects(&self.get_link_geometry_collision_objects_ref(link_geometry_type), &self.get_link_geometry_collision_objects_ref(link_geometry_type), false, Some(&initial_skip_collision_check_tensor))?;
            match intersect_check_multiple_result {
                IntersectCheckMultipleResult::NoIntersectionsFound(i) => { count_collision_check_tensor.increment_count_at_given_idxs( &vec![ ] ); },
                IntersectCheckMultipleResult::IntersectionFound(i) => { count_collision_check_tensor.increment_count_at_given_idxs( i.get_intersection_idxs() );  }
            }
        }

        let new_tensor = BoolCollisionCheckTensor::new_for_always_and_never_in_collision_pairs(&count_collision_check_tensor, None, None, self._get_skip_check_for_self_collision_mode(link_geometry_type))?;
        self._set_link_skip_tensor(new_tensor, link_geometry_type);
        self._save_link_skip_tensor_to_file(link_geometry_type);
        self._save_link_skip_tensor_to_permanent_file(link_geometry_type);

        Ok(())
    }

    fn _load_or_create_link_average_distance_tensor(&mut self, link_geometry_type: &LinkGeometryType, robot_fk_module: &RobotFKModule, robot_bounds_module: &RobotBoundsModule, num_samples: usize, create_new_no_matter_what: bool) -> Result<(), String> {
        let load_result = FloatCollisionCheckTensor::load_from_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_average_distance_tensors(), self._get_link_average_distance_filename(link_geometry_type));
        if load_result.is_ok() && !create_new_no_matter_what {
            self._set_link_average_distance_tensor(load_result.ok().unwrap(), link_geometry_type);
            return Ok(());
        }


        let mut average_distance_tensor = FloatCollisionCheckTensor::new(&self.get_link_geometry_collision_objects_ref(link_geometry_type), &self.get_link_geometry_collision_objects_ref(link_geometry_type));

        for i in 0..num_samples {
            if i % 50 == 0 {
                println!("{}{}Calculating average distance tensor for {:?}: sample {} of {} {}", color::Fg(color::Blue), style::Bold, link_geometry_type, i, num_samples, style::Reset);
            }

            let sample = robot_bounds_module.uniform_sample_from_bounds();
            let fk_res = robot_fk_module.compute_fk(&sample)?;
            self.set_poses_on_links(&fk_res, link_geometry_type);

            let distance_check_multiple_result = distance_check_between_multiple_collision_objects(&self.get_link_geometry_collision_objects_ref(link_geometry_type), &self.get_link_geometry_collision_objects_ref(link_geometry_type), false, None, None)?;
            match distance_check_multiple_result {
                DistanceCheckMultipleResult::NoIntersectionsFound(i) => {
                    let l = i.get_distance_check_idxs().len();
                    for j in 0..l {
                        average_distance_tensor.increment_at_given_single_idxs_by_custom_value( &i.get_distance_check_idxs()[j], i.get_distance_check_distances()[j] );
                    }
                },
                DistanceCheckMultipleResult::IntersectionFound(i) => {
                    let l = i.get_distance_check_idxs().len();
                    for j in 0..l {
                        average_distance_tensor.increment_at_given_single_idxs_by_custom_value( &i.get_distance_check_idxs()[j], i.get_distance_check_distances()[j] );
                    }
                }
            }
            average_distance_tensor.increment_total_num_collision_checks();
        }

        self._set_link_average_distance_tensor(average_distance_tensor, link_geometry_type);
        self._save_link_average_distance_to_file(link_geometry_type);

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _set_dead_link_collision_objects_as_inactive(&mut self, robot_configuration_module: &RobotConfigurationModule) {
        let l = robot_configuration_module.robot_model_module.links.len();
        for i in 0..l {
            if !robot_configuration_module.robot_model_module.links[i].active {

                self._link_obbs[i].iter_mut().for_each(|x| x.active = false );
                self._link_convex_shapes[i].iter_mut().for_each(|x| x.active = false );
                self._link_obb_subcomponents[i].iter_mut().for_each(|x| x.active = false );
                self._link_convex_shape_subcomponents[i].iter_mut().for_each(|x| x.active = false );

            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _create_link_convex_shapes_if_need_be(robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let exists = check_if_path_exists_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_shapes".to_string());
        if !exists {
            convert_all_links_into_convex_shapes_and_save_mesh_files(robot_name.clone())?;
        }
        Ok(())
    }

    fn _create_link_convex_subcomponents_if_need_be(robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let robot_name = robot_configuration_module.robot_model_module.robot_name.clone();
        let exists = check_if_path_exists_relative_to_robot_directory(robot_name.clone(), "autogenerated_metadata/link_convex_subcomponents".to_string());
        if !exists {
            decompose_all_links_into_convex_subcomponents_and_save_mesh_files(robot_name.clone(), 8)?;
        }
        Ok(())
    }

    fn _get_link_convex_shapes_trimesh_engines(robot_configuration_module: &RobotConfigurationModule) -> Result<Vec<Option<TriMeshEngine>>, String> {
        let robot_name= robot_configuration_module.robot_model_module.robot_name.clone();

        let mut out_vec = Vec::new();

        let file_names = get_all_files_in_directory_with_extension_relative_to_robot_directory( robot_name.clone(), "autogenerated_metadata/link_convex_shapes".to_string(), "stl".to_string() );
        let num_files = file_names.len();

        let num_links = robot_configuration_module.robot_model_module.links.len();
        for i in 0..num_links {
            let link_name = robot_configuration_module.robot_model_module.links[i].name.clone();
            let link_name_len = link_name.len();
            let mut found_idx = usize::max_value();
            for j in 0..num_files {
                if file_names[j].contains( &link_name ) && file_names[j].len() == link_name_len + 4 {
                    found_idx = j; break;
                }
            }

            if found_idx == usize::max_value() {
                out_vec.push(None);
            } else {
                let fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_convex_shapes/" + file_names[found_idx].as_str();
                out_vec.push( Some(TriMeshEngine::new_from_path(fp)?) );
            }
        }

        return Ok(out_vec);
    }

    fn _get_link_convex_subcomponents_trimesh_engines(robot_configuration_module: &RobotConfigurationModule) -> Result<Vec<Vec<TriMeshEngine>>, String> {
        let robot_name= robot_configuration_module.robot_model_module.robot_name.clone();

        let mut out_vec = Vec::new();

        let file_names = get_all_files_in_directory_with_extension_relative_to_robot_directory( robot_name.clone(), "autogenerated_metadata/link_convex_subcomponents".to_string(), "stl".to_string() );
        let num_files = file_names.len();

        let num_links = robot_configuration_module.robot_model_module.links.len();
        for i in 0..num_links {
            let mut file_names_for_this_link = Vec::new();
            out_vec.push( Vec::new() );

            let link_name = robot_configuration_module.robot_model_module.links[i].name.clone();
            let link_name_len = link_name.len();
            for j in 0..num_files {
                if file_names[j].contains( &link_name.clone() ) && (file_names[j].len() == link_name_len + 6 || file_names[j].len() == link_name_len + 7 || file_names[j].len() == link_name_len + 8) {
                    file_names_for_this_link.push(file_names[j].clone());
                }
            }

            for f in file_names_for_this_link {
                let fp = get_path_to_particular_robot_directory(robot_name.clone()) + "/autogenerated_metadata/link_convex_subcomponents/" + f.as_str();
                out_vec[i].push( TriMeshEngine::new_from_path(fp)? );
            }
        }

        return Ok(out_vec);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _create_link_obbs(&mut self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let convex_shapes_trimesh_engines = Self::_get_link_convex_shapes_trimesh_engines(robot_configuration_module)?;

        self._link_obbs = Vec::new();

        let l = convex_shapes_trimesh_engines.len();
        for i in 0..l {
            if convex_shapes_trimesh_engines[i].is_none() { self._link_obbs.push( Vec::new() ); }
            else {
                let obb = CollisionObject::new_cuboid_from_trimesh_engine(&convex_shapes_trimesh_engines[i].as_ref().unwrap(), Some( robot_configuration_module.robot_model_module.links[i].name.clone() ));

                self._link_obbs.push( vec![obb] );
            }
        }

        return Ok(());
    }

    fn _create_link_convex_shapes(&mut self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let convex_shapes_trimesh_engines = Self::_get_link_convex_shapes_trimesh_engines(robot_configuration_module)?;

        self._link_convex_shapes = Vec::new();

        let l = convex_shapes_trimesh_engines.len();
        for i in 0..l {
            if convex_shapes_trimesh_engines[i].is_none() { self._link_convex_shapes.push( Vec::new() ); }
            else {
                let convex_shape = CollisionObject::new_convex_hull(&convex_shapes_trimesh_engines[i].as_ref().unwrap(), Some( robot_configuration_module.robot_model_module.links[i].name.clone() ));

                self._link_convex_shapes.push( vec![convex_shape] );
            }
        }

        return Ok(());
    }

    fn _create_link_obb_subcomponents(&mut self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let convex_subcomponents_trimesh_engines = Self::_get_link_convex_subcomponents_trimesh_engines(robot_configuration_module)?;

        self._link_obb_subcomponents = Vec::new();

        let l = convex_subcomponents_trimesh_engines.len();
        for i in 0..l {
            let mut tmp = Vec::new();

            let l2 = convex_subcomponents_trimesh_engines[i].len();
            for j in 0..l2 {
                let collision_obj_name = robot_configuration_module.robot_model_module.links[i].name.clone() + "_" + usize_to_string(j).as_str();
                let obb = CollisionObject::new_cuboid_from_trimesh_engine( &convex_subcomponents_trimesh_engines[i][j], Some(collision_obj_name) );
                tmp.push(obb);
            }

            self._link_obb_subcomponents.push(tmp);
        }

        return Ok(());
    }

    fn _create_link_convex_shape_subcomponents(&mut self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let convex_subcomponents_trimesh_engines = Self::_get_link_convex_subcomponents_trimesh_engines(robot_configuration_module)?;

        self._link_convex_shape_subcomponents = Vec::new();

        let l = convex_subcomponents_trimesh_engines.len();
        for i in 0..l {
            let mut tmp = Vec::new();

            let l2 = convex_subcomponents_trimesh_engines[i].len();
            for j in 0..l2 {
                let collision_obj_name = robot_configuration_module.robot_model_module.links[i].name.clone() + "_" + usize_to_string(j).as_str();
                let convex_shape = CollisionObject::new_convex_hull( &convex_subcomponents_trimesh_engines[i][j], Some(collision_obj_name) );
                tmp.push(convex_shape);
            }

            self._link_convex_shape_subcomponents.push(tmp);
        }

        return Ok(());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_poses_on_links(&mut self, fk_res: &RobotFKResult, link_geometry_type: &LinkGeometryType) {
        match link_geometry_type {
            LinkGeometryType::OBBs => self._set_poses_on_link_obbs(fk_res),
            LinkGeometryType::ConvexShapes => self._set_poses_on_link_convex_shapes(fk_res),
            LinkGeometryType::OBBSubcomponents => self._set_poses_on_link_obb_subcomponents(fk_res),
            LinkGeometryType::ConvexShapeSubcomponents => self._set_poses_on_link_convex_shape_subcomponents(fk_res)
        }
    }

    fn _set_poses_on_link_obbs(&mut self, fk_res: &RobotFKResult) {
        let link_frames = fk_res.get_link_frames_ref();
        let l = link_frames.len();
        for i in 0..l {
            if link_frames[i].is_some() && !self._link_obbs[i].is_empty() {
                self._link_obbs[i][0].set_curr_pose( &link_frames[i].as_ref().unwrap() );
                self._link_obbs[i][0].update_bounding_aabb();
            }
        }
    }

    fn _set_poses_on_link_convex_shapes(&mut self, fk_res: &RobotFKResult) {
        let link_frames = fk_res.get_link_frames_ref();
        let l = link_frames.len();
        for i in 0..l {
            if link_frames[i].is_some() && !self._link_convex_shapes[i].is_empty() {
                self._link_convex_shapes[i][0].set_curr_pose( &link_frames[i].as_ref().unwrap() );
                self._link_convex_shapes[i][0].update_bounding_aabb();
            }
        }
    }

    fn _set_poses_on_link_obb_subcomponents(&mut self, fk_res: &RobotFKResult) {
        let link_frames = fk_res.get_link_frames_ref();
        let l = link_frames.len();
        for i in 0..l {
            if link_frames[i].is_some() && !self._link_obb_subcomponents[i].is_empty() {
                let l2 = self._link_obb_subcomponents[i].len();
                for j in 0..l2 {
                    self._link_obb_subcomponents[i][j].set_curr_pose( &link_frames[i].as_ref().unwrap() );
                    self._link_obb_subcomponents[i][j].update_bounding_aabb();
                }
            }
        }
    }

    fn _set_poses_on_link_convex_shape_subcomponents(&mut self, fk_res: &RobotFKResult) {
        let link_frames = fk_res.get_link_frames_ref();
        let l = link_frames.len();
        for i in 0..l {
            if link_frames[i].is_some() && !self._link_convex_shape_subcomponents[i].is_empty() {
                let l2 = self._link_convex_shape_subcomponents[i].len();
                for j in 0..l2 {
                    self._link_convex_shape_subcomponents[i][j].set_curr_pose( &link_frames[i].as_ref().unwrap() );
                    self._link_convex_shape_subcomponents[i][j].update_bounding_aabb();
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_link_geometry_collision_objects_ref(&self, link_geometry_type: &LinkGeometryType) -> &Vec<Vec<CollisionObject>> {
        match link_geometry_type {
            LinkGeometryType::OBBs => return &self._link_obbs,
            LinkGeometryType::ConvexShapes => return &self._link_convex_shapes,
            LinkGeometryType::OBBSubcomponents => return &self._link_obb_subcomponents,
            LinkGeometryType::ConvexShapeSubcomponents => return &self._link_convex_shape_subcomponents
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _get_skip_collision_check_tensor_ref(&self, link_geometry_type: &LinkGeometryType) -> &BoolCollisionCheckTensor {
        match link_geometry_type {
            LinkGeometryType::OBBs => return &self._link_obbs_skip_collision_check_tensor,
            LinkGeometryType::ConvexShapes => return &self._link_convex_shapes_skip_collision_check_tensor,
            LinkGeometryType::OBBSubcomponents => return &self._link_obb_subcomponents_skip_collision_check_tensor,
            LinkGeometryType::ConvexShapeSubcomponents => return &self._link_convex_shape_subcomponents_skip_collision_check_tensor
        }
    }

    fn _get_skip_collision_check_tensor_mut_ref(&mut self, link_geometry_type: &LinkGeometryType) -> &mut BoolCollisionCheckTensor {
        match link_geometry_type {
            LinkGeometryType::OBBs => return &mut self._link_obbs_skip_collision_check_tensor,
            LinkGeometryType::ConvexShapes => return &mut self._link_convex_shapes_skip_collision_check_tensor,
            LinkGeometryType::OBBSubcomponents => return &mut self._link_obb_subcomponents_skip_collision_check_tensor,
            LinkGeometryType::ConvexShapeSubcomponents => return &mut self._link_convex_shape_subcomponents_skip_collision_check_tensor
        }
    }

    fn _get_average_distance_tensor_ref(&self, link_geometry_type: &LinkGeometryType) -> &FloatCollisionCheckTensor {
        match link_geometry_type {
            LinkGeometryType::OBBs => return &self._link_obbs_average_distance_tensor,
            LinkGeometryType::ConvexShapes => return &self._link_convex_shapes_average_distance_tensor,
            LinkGeometryType::OBBSubcomponents => return &self._link_obb_subcomponents_average_distance_tensor,
            LinkGeometryType::ConvexShapeSubcomponents => return &self._link_convex_shape_subcomponents_average_distance_tensor
        }
    }

    fn _get_average_distance_tensor_mut_ref(&mut self, link_geometry_type: &LinkGeometryType) -> &mut FloatCollisionCheckTensor {
        match link_geometry_type {
            LinkGeometryType::OBBs => return &mut self._link_obbs_average_distance_tensor,
            LinkGeometryType::ConvexShapes => return &mut self._link_convex_shapes_average_distance_tensor,
            LinkGeometryType::OBBSubcomponents => return &mut self._link_obb_subcomponents_average_distance_tensor,
            LinkGeometryType::ConvexShapeSubcomponents => return &mut self._link_convex_shape_subcomponents_average_distance_tensor
        }
    }

    fn _get_partial_fp_to_link_skip_tensors(&self) -> String {
        return "autogenerated_metadata/link_skip_collision_check_tensors".to_string();
    }

    fn _get_link_skip_tensor_filename(&self, link_geometry_type: &LinkGeometryType) -> String {
        match link_geometry_type {
            LinkGeometryType::OBBs => return "link_obbs_skip_collision_check_tensor.json".to_string(),
            LinkGeometryType::ConvexShapes => return "link_convex_shapes_skip_collision_check_tensor.json".to_string(),
            LinkGeometryType::OBBSubcomponents => return "link_obb_subcomponents_skip_collision_check_tensor.json".to_string(),
            LinkGeometryType::ConvexShapeSubcomponents => return "link_convex_shape_subcomponents_skip_collision_check_tensor.json".to_string()
        }
    }

    fn _get_link_skip_tensor_permanent_filename(&self, link_geometry_type: &LinkGeometryType) -> String {
        match link_geometry_type {
            LinkGeometryType::OBBs => return "link_obbs_skip_collision_check_tensor_permanent.json".to_string(),
            LinkGeometryType::ConvexShapes => return "link_convex_shapes_skip_collision_check_tensor_permanent.json".to_string(),
            LinkGeometryType::OBBSubcomponents => return "link_obb_subcomponents_skip_collision_check_tensor_permanent.json".to_string(),
            LinkGeometryType::ConvexShapeSubcomponents => return "link_convex_shape_subcomponents_skip_collision_check_tensor_permanent.json".to_string()
        }
    }

    fn _get_partial_fp_to_average_distance_tensors(&self) -> String {
        return "autogenerated_metadata/link_average_distance_tensors".to_string();
    }

    fn _get_link_average_distance_filename(&self, link_geometry_type: &LinkGeometryType) -> String {
        match link_geometry_type {
            LinkGeometryType::OBBs => return "link_obbs_average_distance_tensor.json".to_string(),
            LinkGeometryType::ConvexShapes => return "link_convex_shapes_average_distance_tensor.json".to_string(),
            LinkGeometryType::OBBSubcomponents => return "link_obb_subcomponents_average_distance_tensor.json".to_string(),
            LinkGeometryType::ConvexShapeSubcomponents => return "link_convex_shape_subcomponents_average_distance_tensor.json".to_string()
        }
    }

    fn _get_skip_check_for_self_collision_mode(&self, link_geometry_type: &LinkGeometryType) -> SkipCheckForSelfCollisionMode {
        match link_geometry_type {
            LinkGeometryType::OBBs => return SkipCheckForSelfCollisionMode::SameObjectOnly,
            LinkGeometryType::ConvexShapes => return SkipCheckForSelfCollisionMode::SameObjectOnly,
            LinkGeometryType::OBBSubcomponents => return SkipCheckForSelfCollisionMode::SameObjectOrSameVector,
            LinkGeometryType::ConvexShapeSubcomponents => return SkipCheckForSelfCollisionMode::SameObjectOrSameVector
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _set_link_skip_tensor(&mut self, tensor: BoolCollisionCheckTensor, link_geometry_type: &LinkGeometryType) {
        match link_geometry_type {
            LinkGeometryType::OBBs => self._link_obbs_skip_collision_check_tensor = tensor,
            LinkGeometryType::ConvexShapes => self._link_convex_shapes_skip_collision_check_tensor = tensor,
            LinkGeometryType::OBBSubcomponents => self._link_obb_subcomponents_skip_collision_check_tensor = tensor,
            LinkGeometryType::ConvexShapeSubcomponents => self._link_convex_shape_subcomponents_skip_collision_check_tensor = tensor
        }
    }

    fn _save_link_skip_tensor_to_file(&self, link_geometry_type: &LinkGeometryType) {
        match link_geometry_type {
            LinkGeometryType::OBBs => self._link_obbs_skip_collision_check_tensor.save_to_file_relative_to_robot_directory( self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_filename(link_geometry_type) ),
            LinkGeometryType::ConvexShapes => self._link_convex_shapes_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_filename(link_geometry_type)),
            LinkGeometryType::OBBSubcomponents => self._link_obb_subcomponents_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_filename(link_geometry_type)),
            LinkGeometryType::ConvexShapeSubcomponents => self._link_convex_shape_subcomponents_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_filename(link_geometry_type))
        }

    }

    fn _save_link_skip_tensor_to_permanent_file(&self, link_geometry_type: &LinkGeometryType) {
        match link_geometry_type {
            LinkGeometryType::OBBs => self._link_obbs_skip_collision_check_tensor.save_to_file_relative_to_robot_directory( self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_permanent_filename(link_geometry_type) ),
            LinkGeometryType::ConvexShapes => self._link_convex_shapes_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_permanent_filename(link_geometry_type)),
            LinkGeometryType::OBBSubcomponents => self._link_obb_subcomponents_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_permanent_filename(link_geometry_type)),
            LinkGeometryType::ConvexShapeSubcomponents => self._link_convex_shape_subcomponents_skip_collision_check_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_link_skip_tensors(), self._get_link_skip_tensor_permanent_filename(link_geometry_type))
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _set_link_average_distance_tensor(&mut self, tensor: FloatCollisionCheckTensor, link_geometry_type: &LinkGeometryType) {
        match link_geometry_type {
            LinkGeometryType::OBBs => self._link_obbs_average_distance_tensor = tensor,
            LinkGeometryType::ConvexShapes => self._link_convex_shapes_average_distance_tensor = tensor,
            LinkGeometryType::OBBSubcomponents => self._link_obb_subcomponents_average_distance_tensor = tensor,
            LinkGeometryType::ConvexShapeSubcomponents => self._link_convex_shape_subcomponents_average_distance_tensor = tensor
        }
    }

    fn _save_link_average_distance_to_file(&self, link_geometry_type: &LinkGeometryType) {
        match link_geometry_type {
            LinkGeometryType::OBBs => self._link_obbs_average_distance_tensor.save_to_file_relative_to_robot_directory( self._robot_name_copy.clone(), self._get_partial_fp_to_average_distance_tensors(), self._get_link_average_distance_filename(link_geometry_type) ),
            LinkGeometryType::ConvexShapes => self._link_convex_shapes_average_distance_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_average_distance_tensors(), self._get_link_average_distance_filename(link_geometry_type)),
            LinkGeometryType::OBBSubcomponents => self._link_obb_subcomponents_average_distance_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_average_distance_tensors(), self._get_link_average_distance_filename(link_geometry_type)),
            LinkGeometryType::ConvexShapeSubcomponents => self._link_convex_shape_subcomponents_average_distance_tensor.save_to_file_relative_to_robot_directory(self._robot_name_copy.clone(), self._get_partial_fp_to_average_distance_tensors(), self._get_link_average_distance_filename(link_geometry_type))
        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_collision_check_skips(&self, link_geometry_type: LinkGeometryType) -> Result<(), String> {
        let collision_check_skip_tensor = self._get_skip_collision_check_tensor_ref(&link_geometry_type);
        let link_geometry_vec = self.get_link_geometry_collision_objects_ref(&link_geometry_type);

        let l = link_geometry_vec.len();
        for i in 0..l {
            let l2 = link_geometry_vec[i].len();
            for j in 0..l2 {
                print!("{}{}{} skips ---> {}", color::Fg(color::Blue), style::Bold, link_geometry_vec[i][j].name, style::Reset);
                let mut skip_string = "".to_string();

                for k in 0..l {
                    let l3 = link_geometry_vec[k].len();
                    for m in 0..l3 {
                        if collision_check_skip_tensor.get_is_skip( [i,j], [k,m] )? && collision_check_skip_tensor.get_is_skip( [k,m], [i,j] )? {
                            skip_string += format!("{}, ", link_geometry_vec[k][m].name).as_str();
                        }
                    }
                }

                print!("{}\n", skip_string);
            }
        }

        Ok(())
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

}

#[derive(Debug, Clone)]
pub enum LinkGeometryType {
    OBBs,
    ConvexShapes,
    OBBSubcomponents,
    ConvexShapeSubcomponents
}