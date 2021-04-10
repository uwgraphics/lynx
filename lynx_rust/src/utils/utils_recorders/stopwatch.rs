use std::time::{Instant, Duration};

#[derive(Clone, Debug)]
pub struct Stopwatch {
    _time_markers: Vec<Duration>,
    _time_marker_labels: Vec<Option<String>>,
    _start_instant: Instant
}

impl Stopwatch {
    pub fn new() -> Self {
        return Self { _time_markers: Vec::new(), _time_marker_labels: Vec::new(), _start_instant: Instant::now() };
    }

    pub fn add_time_marker(&mut self, label: Option<String>) {
        self._time_marker_labels.push(label);
        self._time_markers.push( self._start_instant.elapsed() );
    }

    pub fn print_summary(&self) {
        let l = self._time_markers.len();
        for i in 0..l {
            if self._time_marker_labels[i].is_none() {
                println!("{:?}: {:?}", i, self._time_markers[i]);
            } else {
                println!("{:?}: {:?} --- {:?}", i, self._time_markers[i], self._time_marker_labels[i]);
            }
        }
    }
}