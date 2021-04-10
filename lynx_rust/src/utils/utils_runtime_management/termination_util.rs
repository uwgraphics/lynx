use std::sync::{Arc, RwLock};
use std::{time, thread};
use std::time::Instant;


#[derive(Clone, Debug)]
pub struct TerminationUtil {
    _terminate: Arc<RwLock<bool>>,
    _pause_duration: time::Duration
}
impl TerminationUtil {
    pub fn new() -> Self {
        let pause_duration = time::Duration::from_nanos(100);
        return Self { _terminate: Arc::new( RwLock::new(false) ), _pause_duration: pause_duration };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_terminate(&self) -> bool {
        // thread::sleep(self._pause_duration);
        return *self._terminate.read().unwrap();
    }

    pub fn set_to_terminate(&mut self) {
        *self._terminate.write().unwrap() = true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print(&self) {
        println!("{:?}", self);
    }
}

#[derive(Clone, Debug)]
pub struct TerminationUtilOption(Option<TerminationUtil>);
impl TerminationUtilOption {
    pub fn new() -> Self {
        return TerminationUtilOption( Some(TerminationUtil::new()) );
    }

    pub fn new_none() -> Self {
        return TerminationUtilOption( None );
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_terminate(&self) -> bool {
        if self.0.is_none() {
            return false;
        } else {
            return self.0.as_ref().unwrap().get_terminate();
        }
    }

    pub fn set_to_terminate(&mut self) {
        if self.0.is_none() {
            return;
        } else {
            self.0.as_mut().unwrap().set_to_terminate();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print(&self) {
        println!("{:?}", self);
    }
}