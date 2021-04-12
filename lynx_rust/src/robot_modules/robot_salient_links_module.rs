use crate::utils::utils_robot_objective_specification::link_info::*;
use crate::utils::utils_parsing::yaml_parsing_utils::get_yaml_obj;
use crate::utils::utils_files_and_strings::robot_folder_utils::*;
use crate::robot_modules::robot_configuration_module::RobotConfigurationModule;
use termion::{color, style};
use yaml_rust::yaml::Yaml::{BadValue, Null};

#[derive(Clone)]
pub struct RobotSalientLinksModule {
    _salient_links: Vec<LinkInfo>,
    _robot_name: String
}

impl RobotSalientLinksModule {
    pub fn new(robot_configuration_module: &RobotConfigurationModule) -> Self {
        let robot_name = robot_configuration_module.robot_model_module.robot_name.as_str();
        let partial_fp0 = "robot_salient_links".to_string();

        let res = check_if_path_exists_relative_to_robot_directory(robot_name.to_string(), partial_fp0.clone());
        if !res {
            Self::_create_robot_salient_links_directory_with_example_if_need_be(robot_name);
        }

        let mut out_self = Self { _salient_links: Vec::new(), _robot_name: robot_name.to_string() };

        out_self._load_robot_salient_links_from_file(robot_configuration_module);

        return out_self;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_all_salient_links_ref(&self) -> &Vec<LinkInfo> {
        return &self._salient_links;
    }

    pub fn get_salient_link_ref(&self, salient_link_idx: usize) -> Result<&LinkInfo, String> {
        if salient_link_idx >= self._salient_links.len() {
            return Err(format!("idx {:?} is too big for number of salient links ({:?})", salient_link_idx, self._salient_links.len()));
        }
        return Ok( &self._salient_links[salient_link_idx] );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let l = self._salient_links.len();
        for i in 0..l {
            print!("{}{}salient link {:?} ---> {}", style::Bold, color::Fg(color::Blue), i, style::Reset);
            print!("link_name: {:?}, ", self._salient_links[i].link_name);
            print!("link_idx: {:?}, ", self._salient_links[i].link_idx);
            print!("salient_link_type: {:?}, ", self._salient_links[i].salient_link_type);
            print!("local_forward_axis: {:?}, ", self._salient_links[i].link_local_forward_axis);
            print!("local_left_axis: {:?}, ", self._salient_links[i].link_local_left_axis);
            print!("local_up_axis: {:?}", self._salient_links[i].link_local_up_axis);
            print!("\n");
        }
        println!();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _load_robot_salient_links_from_file(&mut self, robot_configuration_module: &RobotConfigurationModule) -> Result<(), String> {
        let mut fp = get_path_to_particular_robot_directory(self._robot_name.clone()) + "/robot_salient_links/robot_salient_links.yaml";
        let docs = get_yaml_obj(fp.clone())?;
        if docs.len() == 0 { return Ok(()); }

        let doc_option = docs[0].as_vec().clone();
        if doc_option.as_ref().is_none() {
            println!("{}{}Looks like the Yaml formatting is wrong in file {:?}.  Please fix and try again. {}", style::Bold, color::Fg(color::Yellow), fp, style::Reset);
            return Err(format!("looks like the Yaml formatting is wrong in file {:?}.", fp));
        }
        let arr = doc_option.as_ref().unwrap();
        let num_salient_links = arr.len();

        for i in 0..num_salient_links {
            let mut link_name = "".to_string();
            let res = arr[i]["link_name"].clone();
            if res == BadValue {
                println!("{}{}ERROR: The field link_name is required on all array entries in the robot_salient_links Yaml.  Could not initialize robot_salient_links_module.{}", style::Bold, color::Fg(color::Red), style::Reset);
                return Err("The field link_name is required on all array entries in the robot_salient_links Yaml".to_string());
            }
            link_name = res.as_str().unwrap().to_string();



            let mut link_idx = 0 as usize;
            let res = robot_configuration_module.robot_model_module.get_link_idx_from_name(&link_name);
            if res.is_none() {
                println!("{}{}ERROR: link_name {:?} was not found for robot {:?}.  Could not initialize robot_salient_links_module. {}", style::Bold, color::Fg(color::Red), link_name, self._robot_name, style::Reset);
                return Err(format!("link_name {:?} was not found for robot {:?}.", link_name, self._robot_name));
            }
            link_idx = res.unwrap();



            let mut salient_link_type_string = "".to_string();
            let res = arr[i]["salient_link_type"].clone();
            if res == BadValue {
                println!("{}{}ERROR: The field salient_link_type is required on all array entries in the robot_salient_links Yaml.  Could not initialize robot_salient_links_module.{}", style::Bold, color::Fg(color::Red), style::Reset);
                return Err("The field salient_link_type is required on all array entries in the robot_salient_links Yaml".to_string());
            }
            salient_link_type_string = res.as_str().unwrap().to_string();
            let mut salient_link_type_res = SalientLinkType::from_string(&salient_link_type_string);
            if salient_link_type_res.is_err() {
                println!("{}{}ERROR: {:?} is not a valid salient link type.  Could not initialize robot_salient_links_module.{}", style::Bold, color::Fg(color::Red), salient_link_type_string, style::Reset);
                return Err(format!("{:?} is not a valid salient link type.  Could not initialize robot_salient_links_module.", salient_link_type_string));
            }


            let mut link_local_forward_axis_string = "".to_string();
            let mut link_local_forward_axis : Option<LinkAxis> = None;
            let res = arr[i]["link_local_forward_axis"].clone();
            if !(res == BadValue) && !(res == Null) {
                link_local_forward_axis_string = res.as_str().unwrap().to_string();
                let link_axis_res = LinkAxis::from_string(&link_local_forward_axis_string);
                if link_axis_res.is_ok() {
                    link_local_forward_axis = Some(link_axis_res.ok().unwrap().clone());
                } else {
                    println!("{}{}WARNING: Note that {:?} is not a valid link axis.  link_local_forward_axis is defaulting to None.{}", style::Bold, color::Fg(color::Yellow), link_local_forward_axis_string, style::Reset);
                }
            }


            let mut link_local_left_axis_string = "".to_string();
            let mut link_local_left_axis : Option<LinkAxis> = None;
            let res = arr[i]["link_local_left_axis"].clone();
            if !(res == BadValue) && !(res == Null) {
                link_local_left_axis_string = res.as_str().unwrap().to_string();
                let link_axis_res = LinkAxis::from_string(&link_local_left_axis_string);
                if link_axis_res.is_ok() {
                    link_local_left_axis = Some(link_axis_res.ok().unwrap().clone());
                } else {
                    println!("{}{}WARNING: Note that {:?} is not a valid link axis.  link_local_left_axis is defaulting to None.{}", style::Bold, color::Fg(color::Yellow), link_local_left_axis_string, style::Reset);
                }
            }


            let mut link_local_up_axis_string = "".to_string();
            let mut link_local_up_axis : Option<LinkAxis> = None;
            let res = arr[i]["link_local_up_axis"].clone();
            if !(res == BadValue) && !(res == Null) {
                link_local_up_axis_string = res.as_str().unwrap().to_string();
                let link_axis_res = LinkAxis::from_string(&link_local_up_axis_string);
                if link_axis_res.is_ok() {
                    link_local_up_axis = Some(link_axis_res.ok().unwrap().clone());
                } else {
                    println!("{}{}WARNING: Note that {:?} is not a valid link axis.  link_local_up_axis is defaulting to None.{}", style::Bold, color::Fg(color::Yellow), link_local_up_axis_string, style::Reset);
                }
            }

            self._salient_links.push(  LinkInfo::new(link_name, link_idx, link_local_forward_axis, link_local_left_axis, link_local_up_axis, salient_link_type_res.ok().unwrap()) );
        }

        Ok(())
    }

    fn _create_robot_salient_links_directory_with_example_if_need_be(robot_name: &str) {
        let mut out_string = "".to_string();

        out_string += "# this file is used to specify salient links for a particular robot.\n";
        out_string += "# here is the format for two example salient links on the Hubo robot (can support any number): \n";
        out_string += "\n";
        out_string += "# /////////////////////////////////////////////////////////////////////////////\n";
        out_string += "# - link_name: \"BodyRHAND\" \n";
        out_string += "#   salient_link_type: \"EndEffector\" \n";
        out_string += "#   link_local_forward_axis: \"NegZ\" \n";
        out_string += "#   link_local_left_axis: \"Y\" \n";
        out_string += "#   link_local_up_axis: \"X\" \n";
        out_string += "# \n";
        out_string += "# - link_name: \"BodyLHAND\" \n";
        out_string += "#   salient_link_type: \"EndEffector\" \n";
        out_string += "#   link_local_forward_axis: \"NegZ\" \n";
        out_string += "#   link_local_left_axis: \"Y\" \n";
        out_string += "#   link_local_up_axis: \"X\" \n";
        out_string += "# /////////////////////////////////////////////////////////////////////////////\n";
        out_string += "\n";
        out_string += "# Here, link_name is a string of the link's name (as specified in the urdf), e.g., \"panda_hand\".  \n";
        out_string += "# salient_link_type has the options { \"EndEffector\", \"Foot\", \"Elbow\", \"Head\", \"Knee\", \"Base\"}. \n";
        out_string += "# link_local_*_axis, all have options of { \"X\", \"Y\", \"Z\", \"NegX\", \"NegY\", \"NegZ\"}. \n";
        out_string += "#     All link_local_*_axis variables are optional.  However, certain features in the Lynx library will require them, at which point, the API will inform you that they must be specified. \n";
        out_string += "# \n";
        out_string += "# feel free to uncomment the following lines to get started.  If you do not want to specify any robot_salient_links for now, just leave this commented. \n";
        out_string += "\n";
        out_string += "# - link_name: \n";
        out_string += "#   salient_link_type: \n";
        out_string += "#   link_local_forward_axis: \n";
        out_string += "#   link_local_left_axis: \n";
        out_string += "#   link_local_up_axis: \n";

        let partial_fp = "robot_salient_links".to_string();
        write_string_to_file_relative_to_robot_directory(robot_name.to_string(), partial_fp.clone(), "robot_salient_links.yaml".to_string(), out_string, true);
    }
}

