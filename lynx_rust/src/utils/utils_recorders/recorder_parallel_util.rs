use crate::utils::utils_recorders::recorder::*;
use std::sync::RwLock;

pub struct RecorderParallelUtil {
    _none: bool,
    _recorder_collection: RwLock<Vec<RecorderArcMutexOption>>
}

impl RecorderParallelUtil {
    pub fn new_from_recorder_type(recorder: &RecorderArcMutexOption) -> Self {
        return if recorder.is_none() { Self::new_none() } else { Self::new() }
    }

    pub fn new_none() -> Self {
        Self { _none: true, _recorder_collection: RwLock::new(Vec::new()) }
    }

    pub fn new() -> Self {
        Self { _none: false, _recorder_collection: RwLock::new(Vec::new()) }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn spawn(&self) -> RecorderArcMutexOption {
        return if self._none { RecorderArcMutexOption::new_none() } else { RecorderArcMutexOption::new() }
    }

    pub fn collect_recorder_from_thread(&self, recorder: RecorderArcMutexOption) {
        if self._none { return; }

        self._recorder_collection.write().unwrap().push(recorder);
    }

    pub fn absorb_collected_recorders_into_central_recorder(&mut self, central_recorder: &RecorderArcMutexOption) {
        if self._none { return; }

        let mut recorder_collection_unwrap = self._recorder_collection.read().unwrap();
        let l = recorder_collection_unwrap.len();
        for i in 0..l {
            central_recorder.absorb(&recorder_collection_unwrap[i]);
        }
    }
}

