// PRE-DEPRECATION

/*
use nalgebra::DVector;
use termion::{color, style};

/* fulcrum point is the point that predecessors and successors are all centered around.  If optimizing
over a PathSegmentWindow, the fulcrum point should likely be the "variable" that is able to move.

predecessor points and successor points should be organized such that idx 0's are closest to
fulcrum point, then idx 1's, idx 2's, etc. */

pub struct PathSegmentWindow<'a> {
    _fulcrum_point: Option<&'a DVector<f64>>,
    _predecessor_points: Option<Vec<&'a DVector<f64>>>,
    _successor_points: Option<Vec<&'a DVector<f64>>>
}

impl <'a> PathSegmentWindow<'a> {
    pub fn new(fulcrum_point: Option<&'a DVector<f64>>) -> Self {
        return Self { _fulcrum_point: fulcrum_point, _predecessor_points: None, _successor_points: None };
    }

    pub fn set_fulcrum_point(&mut self, point: &'a DVector<f64>) {
        match &mut self._fulcrum_point {
            Some(p) => *p = point,
            None => { self._fulcrum_point = Some(point); }
        }
    }

    pub fn set_predecessor_points(&mut self, points: Vec<&'a DVector<f64>>) {
        self._predecessor_points = Some(points);
    }

    pub fn set_successor_points(&mut self, points: Vec<&'a DVector<f64>>) {
        self._successor_points = Some(points);
    }

    pub fn add_predecessor_point(&mut self, point: &'a DVector<f64>) {
        match &mut self._predecessor_points {
            Some(p) => p.push(point),
            None => { self._predecessor_points = Some( vec![point] ); }
        }
    }

    pub fn add_successor_point(&mut self, point: &'a DVector<f64>) {
        match &mut self._successor_points {
            Some(p) => p.push(point),
            None => { self._successor_points = Some( vec![point] ); }
        }
    }

    pub fn print_summary(&self) {
        println!("{}{}fulcrum point ---> {}", color::Fg(color::Blue), style::Bold, style::Reset);
        println!("{}      {:?} {}", color::Fg(color::White), self._fulcrum_point, style::Reset);
        println!("{}{}predecessor_points ---> {}", color::Fg(color::Blue), style::Bold, style::Reset);
        println!("{}      {:?} {}", color::Fg(color::White), self._predecessor_points, style::Reset);
        println!("{}{}successor points ---> {}", color::Fg(color::Blue), style::Bold, style::Reset);
        println!("{}      {:?} {}", color::Fg(color::White), self._successor_points, style::Reset);
    }
}

 */