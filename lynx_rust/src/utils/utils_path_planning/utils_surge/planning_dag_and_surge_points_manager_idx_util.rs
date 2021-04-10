

/*
explanation for _surge_points_manager_to_planning_dag_overflow --->
// because surge_points_manager_to_planning_dag may be a one to many mapping,
// this vector at least allows for a second planning_dag idx alias per every surge point.
// This allows for solutions to be glued together at a certain node.
*/
#[derive(Clone, Debug)]
pub struct PlanningDAGAndSurgePointsManagerIdxUtil {
    _surge_points_manager_to_planning_dag: Vec<Option<usize>>,
    _surge_points_manager_to_planning_dag_overflow: Vec<Option<usize>>,
    _planning_dag_to_surge_points_manager: Vec<Option<usize>>
}

impl PlanningDAGAndSurgePointsManagerIdxUtil {
    pub fn new_empty() -> Self {
        let _surge_points_manager_to_planning_dag = Vec::new();
        let _surge_points_manager_to_planning_dag_overflow = Vec::new();
        let _planning_dag_to_surge_points_manager = Vec::new();

        return Self { _surge_points_manager_to_planning_dag, _surge_points_manager_to_planning_dag_overflow, _planning_dag_to_surge_points_manager };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_idx_pairing(&mut self, surge_points_manager_idx: usize, planning_dag_idx: usize) -> Result<(), String> {
        while self._surge_points_manager_to_planning_dag.len() <= surge_points_manager_idx {
            self._surge_points_manager_to_planning_dag.push(None);
            self._surge_points_manager_to_planning_dag_overflow.push(None);
        }

        if self._surge_points_manager_to_planning_dag[surge_points_manager_idx].is_some() {
            if self._surge_points_manager_to_planning_dag_overflow[surge_points_manager_idx].is_some() &&
                self._surge_points_manager_to_planning_dag_overflow[surge_points_manager_idx].unwrap() == planning_dag_idx {
                return Ok(());
            } else {
                self._surge_points_manager_to_planning_dag_overflow[surge_points_manager_idx] = Some(planning_dag_idx);
                return Ok(());
            }
        } else {
            self._surge_points_manager_to_planning_dag[surge_points_manager_idx] = Some(planning_dag_idx);
        }

        while self._planning_dag_to_surge_points_manager.len() <= planning_dag_idx {
            self._planning_dag_to_surge_points_manager.push(None);
        }

        if self._planning_dag_to_surge_points_manager[planning_dag_idx].is_some() {
            return Err(format!("planning_dag_idx {:?} was already assigned a surge points manager idx.", planning_dag_idx));
        } else {
            self._planning_dag_to_surge_points_manager[planning_dag_idx] = Some(surge_points_manager_idx);
        }

        Ok(())
    }

    pub fn get_planning_dag_idx_from_surge_points_manager_idx(&self, surge_points_manager_idx: usize) -> Result<usize, String> {
        if surge_points_manager_idx >= self._surge_points_manager_to_planning_dag.len() {
            return Err(format!("surge_points_manager_idx {:?} is too high for the given vector ({:?})", surge_points_manager_idx, self._surge_points_manager_to_planning_dag.len()));
        }

        let o = self._surge_points_manager_to_planning_dag[surge_points_manager_idx];
        if o.is_none() {
            return Err(format!("no mapping exists yet for surge_points_manager_idx {:?}.", surge_points_manager_idx));
        }

        return Ok(o.unwrap());
    }

    pub fn get_planning_dag_overflow_idx_from_surge_points_manager_idx(&self, surge_points_manager_idx: usize) -> Result<usize, String> {
        if surge_points_manager_idx >= self._surge_points_manager_to_planning_dag_overflow.len() {
            return Err(format!("surge_points_manager_idx {:?} is too high for the given vector ({:?})", surge_points_manager_idx, self._surge_points_manager_to_planning_dag_overflow.len()));
        }

        let o = self._surge_points_manager_to_planning_dag_overflow[surge_points_manager_idx];
        if o.is_none() {
            return Err(format!("no overflow mapping exists yet for surge_points_manager_idx {:?}.", surge_points_manager_idx));
        }

        return Ok(o.unwrap());
    }

    pub fn get_surge_points_manager_idx_from_planning_dag_idx(&self, planning_dag_idx: usize) -> Result<usize, String> {
        if planning_dag_idx >= self._planning_dag_to_surge_points_manager.len() {
            return Err(format!("planning_dag_idx {:?} is too high for the given vector ({:?})", planning_dag_idx, self._planning_dag_to_surge_points_manager.len()));
        }

        let o = self._planning_dag_to_surge_points_manager[planning_dag_idx];
        if o.is_none() {
            return Err(format!("no mapping exists yet for planning_dag_idx {:?}.", planning_dag_idx));
        }

        return Ok(o.unwrap());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print(&self) { println!("{:?}", self) }
}