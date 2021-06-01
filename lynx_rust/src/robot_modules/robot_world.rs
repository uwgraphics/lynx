use crate::robot_modules::robot_set::*;
use crate::utils::utils_collisions::prelude::*;
use crate::robot_modules::robot_fk_module::*;
use crate::robot_modules::robot_core_collision_module::LinkGeometryType;
use crate::utils::utils_files_and_strings::prelude::*;
use termion::{style, color};

#[derive(Clone, Debug)]
pub struct RobotWorld {
    _robot_set: RobotSet,
    _collision_environment: Option<CollisionEnvironment>,
    _robot_names: Option<Vec<String>>,
    _configuration_names: Option<Vec<Option<String>>>,
    _robot_set_name: Option<String>,
}

impl RobotWorld {
    pub fn new_from_set_name(robot_set_name: &str, environment_name: Option<&str>) -> Result<Self, String> {
        let _robot_set = RobotSet::new_from_set_name(robot_set_name)?;
        let mut _collision_environment = None;
        if environment_name.is_some() {
            _collision_environment = Some(CollisionEnvironment::new_with_environment_name(environment_name.unwrap())?);
        }

        let mut out_self = Self {
            _robot_set,
            _collision_environment,
            _robot_names: None,
            _configuration_names: None,
            _robot_set_name: Some(robot_set_name.to_string())
        };

        return Ok(out_self);
    }

    pub fn new(robot_names: Vec<&str>, configuration_names: Vec<Option<&str>>, environment_name: Option<&str>) -> Result<Self, String> {
        let mut _robot_names = Vec::new();
        let mut _configuration_names = Vec::new();

        let l = robot_names.len();
        for i in 0..l {
            _robot_names.push(robot_names[i].to_string());
            _configuration_names.push(if configuration_names[i].is_some() { Some(configuration_names[i].as_ref().unwrap().to_string()) } else { None })
        }

        let _robot_set = RobotSet::new(robot_names, configuration_names)?;
        let mut _collision_environment = None;
        if environment_name.is_some() {
            _collision_environment = Some(CollisionEnvironment::new_with_environment_name(environment_name.unwrap())?);
        }

        let mut out_self = Self {
            _robot_set,
            _collision_environment,
            _robot_names: Some(_robot_names),
            _configuration_names: Some(_configuration_names),
            _robot_set_name: None,
        };

        return Ok(out_self);
    }

    pub fn new_from_json_string(json_string: String) -> Result<Self, String> {
        let t: (String, String, String, String) = serde_json::from_str(&json_string).unwrap();
        let robot_names_string: &String = &t.0;
        let configuration_names_string: &String = &t.1;
        let robot_set_name_string: &String = &t.2;
        let collision_environment_string: &String = &t.3;

        type t1 = Option<Vec<String>>;
        let robot_names = load_from_json_string!(robot_names_string, t1);
        type t2 = Option<Vec<Option<String>>>;
        let configuration_names = load_from_json_string!(configuration_names_string, t2);
        type t3 = Option<String>;
        let robot_set_name = load_from_json_string!(robot_set_name_string, t3);

        let collision_environment = if collision_environment_string == "" { None } else { Some(CollisionEnvironment::new_with_json_string(collision_environment_string.to_string()).expect("error")) };

        return if robot_names.is_some() {
            let robot_names_ = robot_names.as_ref().unwrap();
            let configuration_names_ = configuration_names.as_ref().unwrap();

            let mut robot_names_str = Vec::new();
            let mut configuration_names_str = Vec::new();

            for r in robot_names_ {
                robot_names_str.push(r.as_str());
            }

            for c in configuration_names_ {
                configuration_names_str.push(if c.is_none() { None } else { Some(c.as_ref().unwrap().as_str()) })
            }

            let robot_set = RobotSet::new(robot_names_str, configuration_names_str).expect("error");

            Ok(Self {
                _robot_set: robot_set,
                _collision_environment: collision_environment,
                _robot_names: Some(robot_names_.clone()),
                _configuration_names: Some(configuration_names_.clone()),
                _robot_set_name: None
            })
        } else {
            let robot_set_name_ = robot_set_name.as_ref().unwrap();
            let robot_set = RobotSet::new_from_set_name(robot_set_name_).expect("error");

            Ok(Self {
                _robot_set: robot_set,
                _collision_environment: collision_environment,
                _robot_names: None,
                _configuration_names: None,
                _robot_set_name: Some(robot_set_name_.clone())
            })
        };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn environment_intersect_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VecOfIntersectCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_intersect_check(&fk_res.get_robot_fk_results_ref()[i],
                                             link_geometry_type.clone(),
                                             &self._collision_environment.as_ref().unwrap(),
                                             stop_at_first_detected)?;

            let label = format!("environment intersect check on robot {} ({:?})", i, self._robot_set.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_distance_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfDistanceCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VecOfDistanceCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_distance_check(&fk_res.get_robot_fk_results_ref()[i],
                                            link_geometry_type.clone(),
                                            &self._collision_environment.as_ref().unwrap(),
                                            stop_at_first_detected)?;

            let label = format!("environment distance check on robot {} ({:?})", i, self._robot_set.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_contact_check(&mut self, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VecOfContactCheckMultipleResult, String> {
        if self._collision_environment.is_none() {
            return Ok(VecOfContactCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected, margin));
        }
        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_contact_check(&fk_res.get_robot_fk_results_ref()[i],
                                           link_geometry_type.clone(),
                                           &self._collision_environment.as_ref().unwrap(),
                                           stop_at_first_detected, margin)?;

            let label = format!("environment contact check on robot {} ({:?})", i, self._robot_set.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_intersect_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfIntersectCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._robot_set.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._robot_set.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfIntersectCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfIntersectCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_intersect_check_subset(&subset_check_idxs[i],
                                                    &fk_res.get_robot_fk_results_ref()[i],
                                                    link_geometry_type.clone(),
                                                    &self._collision_environment.as_ref().unwrap(),
                                                    stop_at_first_detected)?;

            let label = format!("environment intersect subset check on robot {} ({:?})", i, self._robot_set.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_distance_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool) -> Result<VecOfDistanceCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._robot_set.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._robot_set.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfDistanceCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected));
        }
        let mut out_vec = VecOfDistanceCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_distance_check_subset(&subset_check_idxs[i],
                                                   &fk_res.get_robot_fk_results_ref()[i],
                                                   link_geometry_type.clone(),
                                                   &self._collision_environment.as_ref().unwrap(),
                                                   stop_at_first_detected)?;

            let label = format!("environment distance check subset on robot {} ({:?})", i, self._robot_set.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    pub fn environment_contact_check_subset(&mut self, subset_check_idxs: &Vec<Vec<[[usize; 2]; 2]>>, fk_res: &VecOfRobotFKResult, link_geometry_type: LinkGeometryType, stop_at_first_detected: bool, margin: Option<f64>) -> Result<VecOfContactCheckMultipleResult, String> {
        if subset_check_idxs.len() != self._robot_set.get_num_robots() {
            return Err(format!("number of subset_check_idxs {:?} must equal number of robots {:?}", subset_check_idxs.len(), self._robot_set.get_num_robots()));
        }

        if self._collision_environment.is_none() {
            return Ok(VecOfContactCheckMultipleResult::new_no_intersections_found(self._robot_set.get_num_robots(), stop_at_first_detected, margin));
        }
        let mut out_vec = VecOfContactCheckMultipleResult::new_empty();

        let l = self._robot_set.get_num_robots();
        for i in 0..l {
            let res = self._robot_set
                .get_robots_mut_ref()[i]
                .get_core_collision_module_mut_ref()
                .environment_contact_check_subset(&subset_check_idxs[i],
                                                  &fk_res.get_robot_fk_results_ref()[i],
                                                  link_geometry_type.clone(),
                                                  &self._collision_environment.as_ref().unwrap(),
                                                  stop_at_first_detected, margin)?;

            let label = format!("environment contact check subset on robot {} ({:?})", i, self._robot_set.get_robots_ref()[i].get_configuration_module_ref().robot_model_module.robot_name.clone());
            if stop_at_first_detected && res.is_in_collision() {
                out_vec.add_new_result(res, label);
                return Ok(out_vec);
            } else {
                out_vec.add_new_result(res, label);
            }
        }

        return Ok(out_vec);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn update_robot_set(&mut self, robot_names: Vec<&str>, configuration_names: Vec<Option<&str>>) -> Result<(), String> {
        let robot_set = RobotSet::new(robot_names.clone(), configuration_names.clone())?;
        self._robot_set = robot_set;
        self._robot_names = Some(str_vec_to_string_vec(&robot_names));
        self._configuration_names = Some( str_option_vec_to_string_option_vec(&configuration_names) );
        self._robot_set_name = None;
        Ok(())
    }

    pub fn update_robot_set_from_set_name(&mut self, robot_set_name: &str) -> Result<(), String> {
        let robot_set = RobotSet::new_from_set_name(robot_set_name)?;
        self._robot_set = robot_set;
        self._robot_names = None;
        self._configuration_names = None;
        self._robot_set_name = Some(robot_set_name.to_string());
        Ok(())
    }

    pub fn update_robot_set_with_given_set(&mut self, robot_set: RobotSet) {
        self._robot_names = robot_set.robot_names.clone();
        self._configuration_names = robot_set.configuration_names.clone();
        self._robot_set_name = robot_set.robot_set_name.clone();
        self._robot_set = robot_set;
    }

    pub fn update_collision_environment(&mut self, environment_name: Option<&str>) -> Result<(), String> {
        self._collision_environment = None;
        if environment_name.is_some() {
            self._collision_environment = Some(CollisionEnvironment::new_with_environment_name(environment_name.unwrap())?);
            self._collision_environment.as_mut().unwrap().update_bounding_volumes_on_all_environment_obbs();
        }
        Ok(())
    }

    pub fn update_collision_environment_with_given_collision_environment(&mut self, collision_environment: CollisionEnvironment) {
        self._collision_environment = Some(collision_environment);
        self._collision_environment.as_mut().unwrap().update_bounding_volumes_on_all_environment_obbs();
    }

    pub fn set_robot_set(&mut self, robot_set: RobotSet) {
        self._robot_set = robot_set;
    }

    pub fn set_collision_environment(&mut self, collision_environment: CollisionEnvironment) {
        self._collision_environment = Some(collision_environment);
    }

    pub fn absorb_collision_environment(&mut self, collision_environment: CollisionEnvironment) {
        if self._collision_environment.is_some() {
            self._collision_environment.as_mut().unwrap().absorb(&collision_environment);
        } else {
            self.set_collision_environment(collision_environment);
        }
    }

    pub fn remove_collision_environment(&mut self) {
        self._collision_environment = None;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_json_string(&self) -> String {
        let c1 = convert_to_json_string!(&self._robot_names);
        let c2 = convert_to_json_string!(&self._configuration_names);
        let c3 = convert_to_json_string!(&self._robot_set_name);
        let c4 = if self._collision_environment.is_none() { "".to_string() } else { self._collision_environment.as_ref().unwrap().get_json_string() };

        let out_string = convert_to_json_string!(&(c1,c2,c3,c4));

        return out_string;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_robot_set_mut_ref(&mut self) -> &mut RobotSet {
        return &mut self._robot_set;
    }

    pub fn get_robot_set_ref(&self) -> &RobotSet {
        return &self._robot_set;
    }

    pub fn get_collision_environment_option_mut_ref(&mut self) -> &mut Option<CollisionEnvironment> {
        return &mut self._collision_environment;
    }

    pub fn get_collision_environment_option_ref(&self) -> &Option<CollisionEnvironment> {
        return &self._collision_environment;
    }
}
