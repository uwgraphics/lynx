

#[derive(Debug, Clone)]
pub enum CollisionCheckResult {
    NotInCollision,
    InCollision(String),
    Error(String)
}

impl CollisionCheckResult {
    pub fn is_in_collision(&self) -> bool {
        return match self {
            CollisionCheckResult::NotInCollision => { false }
            CollisionCheckResult::InCollision(_) => { true }
            CollisionCheckResult::Error(_) => { false }
        }
    }
}