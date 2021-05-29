

pub struct NumberOfRobotSetsSpawnedInScene(pub usize);
impl Default for NumberOfRobotSetsSpawnedInScene {
    fn default() -> Self {
        Self(0)
    }
}
impl NumberOfRobotSetsSpawnedInScene {
    pub fn increment(&mut self) {
        self.0 += 1;
    }
}