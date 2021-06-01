use crate::robot_modules::prelude::RobotWorld;
use nalgebra::DVector;

#[derive(Clone, Debug)]
pub struct PathPlanningQuery {
    pub robot_world: RobotWorld,
    pub start_state: DVector<f64>,
    pub goal_state: DVector<f64>
}

impl PathPlanningQuery {
    pub fn new(robot_world: RobotWorld, start_state: DVector<f64>, goal_state: DVector<f64>) -> Self {
        Self {
            robot_world,
            start_state,
            goal_state
        }
    }

    pub fn new_from_json_string(json_string: String) -> Result<Self, String> {
        let t: (String, String, String) = serde_json::from_str(json_string.as_str()).unwrap();
        let robot_world_string = &t.0;
        let start_state_string = &t.1;
        let goal_state_string = &t.2;

        let robot_world = RobotWorld::new_from_json_string(robot_world_string.clone())?;
        type D = DVector<f64>;
        let start_state = load_from_json_string!(start_state_string, D);
        let goal_state = load_from_json_string!(goal_state_string, D);

        Ok( Self {
            robot_world,
            start_state,
            goal_state
        })
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_json_string(&self) -> String {
        let s1 = self.robot_world.get_json_string();
        let s2 = convert_to_json_string!(&self.start_state);
        let s3 = convert_to_json_string!(&self.goal_state);

        let out_json_string = convert_to_json_string!(&(s1,s2,s3));
        return out_json_string;
    }
}

