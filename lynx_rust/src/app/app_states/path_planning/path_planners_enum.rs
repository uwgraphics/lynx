

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd)]
pub enum PathPlannerSelection {
    SPRINT,
    RRTConnect,
    RRT
}

impl PathPlannerSelection {
    pub fn to_string(&self) -> String {
        return match self {
            PathPlannerSelection::SPRINT => { "SPRINT".to_string() }
            PathPlannerSelection::RRTConnect => { "RRTConnect".to_string() }
            PathPlannerSelection::RRT => { "RRT".to_string() }
        }
    }
}

