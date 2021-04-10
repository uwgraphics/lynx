// NEEDS REFACTOR

/*
use crate::utils::utils_se3::implicit_dual_quaternion::ImplicitDualQuaternion;
// use crate::utils::utils_robot_objective_specification::robot_salient_link_description::RobotSalientLinkDescription;
use crate::robot_modules::robot_fk_module::RobotFKResult;
use nalgebra::{Vector3, Point3, DVector, UnitQuaternion};

#[derive(Clone, Debug)]
pub struct LinkPoseMatchingSpecification {
    pub salient_link_idx: usize,
    pub goal_pose: ImplicitDualQuaternion
}

impl LinkPoseMatchingSpecification {
    pub fn new(salient_link_idx: usize, goal_pose: ImplicitDualQuaternion) -> Self {
        Self { salient_link_idx, goal_pose }
    }

    pub fn new_relative_to_fk_res( robot_salient_link_description: &RobotSalientLinkDescription, salient_link_idx: usize, fk_res: &RobotFKResult, offset: Option<ImplicitDualQuaternion>) -> Result<Self, String> {
        let salient_links = robot_salient_link_description.get_salient_links();
        if salient_link_idx >= salient_links.len() {
            return Err(format!("salient_link_idx is too large given the number of salient links ({:?} vs. {:?})", salient_link_idx, salient_links.len()));
        }

        let link_idx = salient_links[salient_link_idx].link_idx;

        let link_frames_ref = fk_res.get_link_frames_ref();

        if link_frames_ref[link_idx].is_none() {
            return Err(format!("link idx {:?} is none in fk_res.  Cannot make LinkPoseMatchingSpecification.", link_idx));
        }

        let mut goal_pose = link_frames_ref[link_idx].as_ref().unwrap().clone();

        if offset.is_some() {
            // goal_pose = offset.unwrap().multiply(&goal_pose);
            goal_pose.quat = offset.as_ref().unwrap().quat * &goal_pose.quat;
            goal_pose.translation = offset.as_ref().unwrap().translation + &goal_pose.translation;
        }

        return Ok(Self::new(salient_link_idx, goal_pose));

    }
}

#[derive(Clone, Debug)]
pub struct LinkPositionMatchingSpecification {
    pub salient_link_idx: usize,
    pub goal_position: Vector3<f64>
}

impl LinkPositionMatchingSpecification {
    pub fn new(salient_link_idx: usize, goal_position: Vector3<f64>) -> Self {
        Self {salient_link_idx, goal_position}
    }

    pub fn new_relative_to_fk_res( robot_salient_link_description: &RobotSalientLinkDescription, salient_link_idx: usize, fk_res: &RobotFKResult, offset: Option<Vector3<f64>>) -> Result<Self, String> {
        let salient_links = robot_salient_link_description.get_salient_links();
        if salient_link_idx >= salient_links.len() {
            return Err(format!("salient_link_idx is too large given the number of salient links ({:?} vs. {:?})", salient_link_idx, salient_links.len()));
        }

        let link_idx = salient_links[salient_link_idx].link_idx;

        let link_frames_ref = fk_res.get_link_frames_ref();

        if link_frames_ref[link_idx].is_none() {
            return Err(format!("link idx {:?} is none in fk_res.  Cannot make LinkPoseMatchingSpecification.", link_idx));
        }

        let mut goal_position = link_frames_ref[link_idx].as_ref().unwrap().translation.clone();

        if offset.is_some() {
            goal_position = offset.unwrap() + &goal_position;
        }

        return Ok(Self::new(salient_link_idx, goal_position));

    }
}

#[derive(Clone, Debug)]
pub struct LinkOrientationMatchingSpecification {
    pub salient_link_idx: usize,
    pub goal_orientation: UnitQuaternion<f64>
}

impl LinkOrientationMatchingSpecification {
    pub fn new(salient_link_idx: usize, goal_orientation: UnitQuaternion<f64>) -> Self {
        Self {salient_link_idx, goal_orientation}
    }

    pub fn new_relative_to_fk_res( robot_salient_link_description: &RobotSalientLinkDescription, salient_link_idx: usize, fk_res: &RobotFKResult, offset: Option<UnitQuaternion<f64>>) -> Result<Self, String> {
        let salient_links = robot_salient_link_description.get_salient_links();
        if salient_link_idx >= salient_links.len() {
            return Err(format!("salient_link_idx is too large given the number of salient links ({:?} vs. {:?})", salient_link_idx, salient_links.len()));
        }

        let link_idx = salient_links[salient_link_idx].link_idx;

        let link_frames_ref = fk_res.get_link_frames_ref();

        if link_frames_ref[link_idx].is_none() {
            return Err(format!("link idx {:?} is none in fk_res.  Cannot make LinkPoseMatchingSpecification.", link_idx));
        }

        let mut goal_orientation = link_frames_ref[link_idx].as_ref().unwrap().quat.clone();

        if offset.is_some() {
            goal_orientation = &offset.unwrap() * &goal_orientation;
        }

        return Ok(Self::new(salient_link_idx, goal_orientation));

    }
}

#[derive(Clone, Debug)]
pub struct LinkLookAtSpecification {
    pub salient_link_idx: usize,
    pub lookat_target: Vector3<f64>
}

impl LinkLookAtSpecification {
    pub fn new(salient_link_idx: usize, lookat_target: Vector3<f64>) -> Self {
        Self {salient_link_idx, lookat_target}
    }

    pub fn new_relative_to_fk_res( robot_salient_link_description: &RobotSalientLinkDescription, salient_link_idx: usize, fk_res: &RobotFKResult, offset: Option<Vector3<f64>>) -> Result<Self, String> {
        let salient_links = robot_salient_link_description.get_salient_links();
        if salient_link_idx >= salient_links.len() {
            return Err(format!("salient_link_idx is too large given the number of salient links ({:?} vs. {:?})", salient_link_idx, salient_links.len()));
        }

        let link_idx = salient_links[salient_link_idx].link_idx;

        let link_frames_ref = fk_res.get_link_frames_ref();

        if link_frames_ref[link_idx].is_none() {
            return Err(format!("link idx {:?} is none in fk_res.  Cannot make LinkPoseMatchingSpecification.", link_idx));
        }

        let mut lookat_target = link_frames_ref[link_idx].as_ref().unwrap().translation.clone();

        if offset.is_some() {
            lookat_target = offset.unwrap() + &lookat_target;
        }

        return Ok(Self::new(salient_link_idx, lookat_target));
    }
}

#[derive(Clone, Debug)]
pub struct LinkLookAtAnotherLinkSpecification {
    pub salient_link_idx: usize,
    pub lookat_salient_link_idx: usize
}

impl LinkLookAtAnotherLinkSpecification {
    pub fn new(salient_link_idx: usize, lookat_salient_link_idx: usize) -> Self {
        Self {salient_link_idx, lookat_salient_link_idx}
    }
}

#[derive(Clone, Debug)]
pub struct LinkUprightSpecification {
    pub salient_link_idx: usize,
}

impl LinkUprightSpecification {
    pub fn new(salient_link_idx: usize) -> Self {
        Self {salient_link_idx}
    }

}


#[derive(Clone, Debug)]
pub struct TwoLinkMatchGivenDistanceSpecification {
    pub salient_link_idx1: usize,
    pub salient_link_idx2: usize,
    pub distance: f64
}

impl TwoLinkMatchGivenDistanceSpecification {
    pub fn new(salient_link_idx1: usize, salient_link_idx2: usize, distance: f64) -> Self {
        Self {salient_link_idx1, salient_link_idx2, distance}
    }

}


#[derive(Clone, Debug)]
pub struct TwoLinkMatchOrientationSpecification {
    pub salient_link_idx1: usize,
    pub salient_link_idx2: usize,
}

impl TwoLinkMatchOrientationSpecification {
    pub fn new(salient_link_idx1: usize, salient_link_idx2: usize) -> Self {
        Self {salient_link_idx1, salient_link_idx2}
    }

}

*/