use nalgebra::{Vector3, Rotation3};

#[derive(Clone, Debug)]
pub struct LinkInfo {
    pub link_name: String,
    pub link_idx: usize,
    pub link_local_forward_axis: Option<LinkAxis>,
    pub link_local_left_axis: Option<LinkAxis>,
    pub link_local_up_axis: Option<LinkAxis>,
    pub salient_link_type: SalientLinkType
}
impl LinkInfo {
    pub fn new( link_name: String, link_idx: usize, link_local_forward_axis: Option<LinkAxis>, link_local_left_axis: Option<LinkAxis>, link_local_up_axis: Option<LinkAxis>, salient_link_type: SalientLinkType ) -> Self {
        Self { link_name, link_idx, link_local_forward_axis, link_local_left_axis, link_local_up_axis, salient_link_type }
    }
}

#[derive(Clone, Debug)]
pub enum SalientLinkType {
    EndEffector, Foot, Elbow, Head, Knee, Base
}

impl SalientLinkType {
    pub fn from_string(s: &String) -> Result<SalientLinkType, String> {
        if s == "EndEffector" { return Ok(SalientLinkType::EndEffector); }
        else if s == "Foot" { return Ok(SalientLinkType::Foot); }
        else if s == "Elbow" { return Ok(SalientLinkType::Elbow); }
        else if s == "Head" { return Ok(SalientLinkType::Head); }
        else if s == "Knee" { return Ok(SalientLinkType::Knee); }
        else if s == "Base" { return Ok(SalientLinkType::Base); }
        else { return Err(format!("{:?} is not a valid salient link type", s)) }
    }
}

#[derive(Clone, Debug)]
pub enum LinkAxis {
    X, Y, Z, NegX, NegY, NegZ
}

impl LinkAxis {
    pub fn from_string(s: &String) -> Result<LinkAxis, String> {
        if s == "X" { return Ok(LinkAxis::X); }
        else if s == "Y" { return Ok(LinkAxis::Y); }
        else if s == "Z" { return Ok(LinkAxis::Z); }
        else if s == "NegX" { return Ok(LinkAxis::NegX); }
        else if s == "NegY" { return Ok(LinkAxis::NegY); }
        else if s == "NegZ" { return Ok(LinkAxis::NegZ); }
        else { return Err(format!("{:?} is not a valid link axis", s)) }
    }

    pub fn to_vector_from_rotation_matrix(&self, rotation_matrix: &Rotation3<f64>) -> Vector3<f64> {
        match self {
            LinkAxis::X => return Vector3::new( rotation_matrix[(0,0)], rotation_matrix[(1,0)], rotation_matrix[(2,0)] ),
            LinkAxis::Y => return Vector3::new( rotation_matrix[(0,1)], rotation_matrix[(1,1)], rotation_matrix[(2,1)] ),
            LinkAxis::Z => return Vector3::new( rotation_matrix[(0,2)], rotation_matrix[(1,2)], rotation_matrix[(2,2)] ),
            LinkAxis::NegX => return Vector3::new( -rotation_matrix[(0,0)], -rotation_matrix[(1,0)], -rotation_matrix[(2,0)] ),
            LinkAxis::NegY => return Vector3::new( -rotation_matrix[(0,1)], -rotation_matrix[(1,1)], -rotation_matrix[(2,1)] ),
            LinkAxis::NegZ => return Vector3::new( -rotation_matrix[(0,2)], -rotation_matrix[(1,2)], -rotation_matrix[(2,2)] )
        }
    }
}