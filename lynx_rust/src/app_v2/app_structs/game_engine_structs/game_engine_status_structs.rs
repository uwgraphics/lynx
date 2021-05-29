
pub struct FrameCount(pub usize);
impl Default for FrameCount {
    fn default() -> Self {
        Self(0)
    }
}