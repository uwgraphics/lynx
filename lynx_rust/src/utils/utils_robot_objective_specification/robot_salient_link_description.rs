
// PRE-DEPRECATION, MAKE SURE ENTRIES ARE COPIED BEFORE DELETING

/*
use crate::utils::utils_robot_objective_specification::link_info::{*};
use termion::{color, style};

/*
#[derive(Clone, Debug)]
pub struct RobotInfo {
    pub ee_link_idxs: Vec<usize>,
    pub ee_link_names: Vec<String>,
    pub ee_link_forward_axes: Vec<LinkAxis>,
    pub ee_link_left_axes: Vec<LinkAxis>,
    pub ee_link_up_axes: Vec<LinkAxis>,
    pub elbow_link_idxs: Vec<usize>,
    pub elbow_link_names: Vec<String>,
    pub base_link_idx: usize,
    pub base_link_name: String
}

impl RobotInfo {
    pub fn new(robot_name: &'static str) -> Result<Self, String> {
        match robot_name {
            "ur5" => return Ok(Self::ur5()),
            "sawyer" => return Ok(Self::sawyer()),
            "panda" => return Ok(Self::panda()),
            _ => return Err(format!("robot_name {} currently not handled by RobotInfo", robot_name))
        }
    }

    pub fn ur5() -> Self {
        let ee_link_idxs = vec![ 7 ];
        let ee_link_names = vec!["ee_link".to_string()];
        let ee_link_forward_axes = vec![ LinkAxis::X ];
        let ee_link_left_axes = vec![ LinkAxis::Y ];
        let ee_link_up_axes = vec![ LinkAxis::Z ];
        let elbow_link_idxs = vec![ 3 ];
        let elbow_link_names = vec![ "forearm_link".to_string() ];
        let base_link_idx = 10 as usize;
        let base_link_name = "world".to_string();
        return Self { ee_link_idxs, ee_link_names, ee_link_forward_axes, ee_link_left_axes, ee_link_up_axes, elbow_link_idxs, elbow_link_names, base_link_idx, base_link_name };
    }

    pub fn sawyer() -> Self {
        let ee_link_idxs = vec![ 18 ];
        let ee_link_names = vec!["right_hand".to_string()];
        let ee_link_forward_axes = vec![ LinkAxis::Z ];
        let ee_link_left_axes = vec![ LinkAxis::NegY ];
        let ee_link_up_axes = vec![ LinkAxis::X ];
        let elbow_link_idxs = vec![ 11 ];
        let elbow_link_names = vec![ "right_l3".to_string() ];
        let base_link_idx = 0 as usize;
        let base_link_name = "base".to_string();
        return Self { ee_link_idxs, ee_link_names, ee_link_forward_axes, ee_link_left_axes, ee_link_up_axes, elbow_link_idxs, elbow_link_names, base_link_idx, base_link_name };
    }

    pub fn panda() -> Self {
        let ee_link_idxs = vec![ 10 ];
        let ee_link_names = vec!["panda_hand".to_string()];
        let ee_link_forward_axes = vec![ LinkAxis::Z ];
        let ee_link_left_axes = vec![ LinkAxis::NegY ];
        let ee_link_up_axes = vec![ LinkAxis::X ];
        let elbow_link_idxs = vec![ 5 ];
        let elbow_link_names = vec![ "panda_link4".to_string() ];
        let base_link_idx = 1 as usize;
        let base_link_name = "panda_link0".to_string();
        return Self { ee_link_idxs, ee_link_names, ee_link_forward_axes, ee_link_left_axes, ee_link_up_axes, elbow_link_idxs, elbow_link_names, base_link_idx, base_link_name };
    }

    pub fn hubo() -> Self {
        let ee_link_idxs = vec![ 28, 10, 55, 45];
        let ee_link_names = vec!["Body_RHAND".to_string(), "Body_LHAND".to_string(), "Body_RAFT".to_string(), "Body_LAFT".to_string()];
        let ee_link_forward_axes = vec![ LinkAxis::NegZ, LinkAxis::NegZ, LinkAxis::X, LinkAxis::X];
        let ee_link_left_axes = vec![ LinkAxis::Y, LinkAxis::Y, LinkAxis::Y, LinkAxis::Y];
        let ee_link_up_axes = vec![ LinkAxis::X, LinkAxis::X, LinkAxis::Z, LinkAxis::Z ];
        let elbow_link_idxs = vec![ 5 ];
        let elbow_link_names = vec![ "panda_link4".to_string() ];
        let base_link_idx = 1 as usize;
        let base_link_name = "panda_link0".to_string();
        return Self { ee_link_idxs, ee_link_names, ee_link_forward_axes, ee_link_left_axes, ee_link_up_axes, elbow_link_idxs, elbow_link_names, base_link_idx, base_link_name };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        println!("{:?}", self);
    }
}
*/

#[derive(Clone, Debug)]
pub struct RobotSalientLinkDescription {
    _salient_links: Vec<LinkInfo>
}

impl RobotSalientLinkDescription {
    pub fn new(robot_name: &str) -> Result<Self, String> {
        match robot_name {
            "ur5" => return Ok(Self::ur5()),
            "sawyer" => return Ok(Self::sawyer()),
            "panda" => return Ok(Self::panda()),
            "hubo" => return Ok(Self::hubo()),
            "jaco7" => return Ok(Self::jaco7()),
            "iiwa7" => return Ok(Self::iiwa7()),
            _ => return Err(format!("robot_name {} currently not handled by RobotInfo", robot_name))
        }
    }

    pub fn ur5() -> Self {
        let mut salient_links = Vec::new();

        salient_links.push( LinkInfo::new( "ee_link".to_string(), 7, Some(LinkAxis::X), Some(LinkAxis::Y), Some(LinkAxis::Z), SalientLinkType::EndEffector ) );

        Self { _salient_links: salient_links }
    }

    pub fn sawyer() -> Self {
        let mut salient_links = Vec::new();

        salient_links.push( LinkInfo::new( "right_hand".to_string(), 18, Some(LinkAxis::Z), Some(LinkAxis::NegY), Some(LinkAxis::X), SalientLinkType::EndEffector ) );

        Self { _salient_links: salient_links }
    }

    pub fn panda() -> Self {
        let mut salient_links = Vec::new();

        salient_links.push( LinkInfo::new( "panda_hand".to_string(), 10, Some(LinkAxis::Z), Some(LinkAxis::NegY), Some(LinkAxis::X), SalientLinkType::EndEffector ) );

        Self { _salient_links: salient_links }
    }

    pub fn hubo() -> Self {
        let mut salient_links = Vec::new();

        salient_links.push( LinkInfo::new( "Body_RHAND".to_string(), 28, Some(LinkAxis::NegZ), Some(LinkAxis::Y), Some(LinkAxis::X), SalientLinkType::EndEffector ) );
        salient_links.push( LinkInfo::new( "Body_LHAND".to_string(), 10, Some(LinkAxis::NegZ), Some(LinkAxis::Y), Some(LinkAxis::X), SalientLinkType::EndEffector ) );
        salient_links.push( LinkInfo::new( "Body_RAFT".to_string(), 55, Some(LinkAxis::X), Some(LinkAxis::Y), Some(LinkAxis::Z), SalientLinkType::Foot ) );
        salient_links.push( LinkInfo::new( "Body_LAFT".to_string(), 45, Some(LinkAxis::X), Some(LinkAxis::Y), Some(LinkAxis::Z), SalientLinkType::Foot ) );
        salient_links.push( LinkInfo::new( "Body_WST".to_string(), 38, Some(LinkAxis::X), Some(LinkAxis::Y), Some(LinkAxis::Z), SalientLinkType::Base ) );

        Self { _salient_links: salient_links }
    }

    pub fn jaco7() -> Self {
        let mut salient_links = Vec::new();

        salient_links.push( LinkInfo::new( "j2s7s300_end_effector".to_string(), 10, Some(LinkAxis::Z), Some(LinkAxis::NegY), Some(LinkAxis::X), SalientLinkType::EndEffector ) );

        Self { _salient_links: salient_links }
    }

    pub fn iiwa7() -> Self {
        let mut salient_links = Vec::new();

        salient_links.push( LinkInfo::new( "iiwa_link_ee".to_string(), 9, Some(LinkAxis::Z), Some(LinkAxis::Y), Some(LinkAxis::NegX), SalientLinkType::EndEffector ) );

        Self { _salient_links: salient_links }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_salient_links(&self) -> &Vec<LinkInfo> {
        return &self._salient_links;
    }

    pub fn print_summary(&self) {
        let l = self._salient_links.len();
        for i in 0..l {
            println!("{}{}Salient link {:?} ---> {}", color::Fg(color::Blue), style::Bold, i, style::Reset);
            println!("{}     link name: {:?} {}", color::Fg(color::White), self._salient_links[i].link_name, style::Reset);
            println!("{}     link idx: {:?} {}", color::Fg(color::White), self._salient_links[i].link_idx, style::Reset);
            println!("{}     link local forward axis: {:?} {}", color::Fg(color::White), self._salient_links[i].link_local_forward_axis, style::Reset);
            println!("{}     link local left axis: {:?} {}", color::Fg(color::White), self._salient_links[i].link_local_left_axis, style::Reset);
            println!("{}     link local up axis: {:?} {}", color::Fg(color::White), self._salient_links[i].link_local_up_axis, style::Reset);
            println!("{}     salient link type: {:?} {}", color::Fg(color::White), self._salient_links[i].salient_link_type, style::Reset);
        }
    }
}


 */