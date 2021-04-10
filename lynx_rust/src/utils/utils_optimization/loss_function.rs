use core::fmt::Debug;

pub trait LossFunction: Debug + LossFunctionClone {
    fn name(&self) -> String {
        return "".to_string();
    }
    fn loss(&self, x: f64) -> f64;
    fn derivative(&self, x: f64) -> f64;
}

pub trait LossFunctionClone {
    fn clone_box(&self) -> Box<dyn LossFunction>;
}
impl<T> LossFunctionClone for T where T: 'static + LossFunction + Clone {
    fn clone_box(&self) -> Box<dyn LossFunction> {
        Box::new(self.clone())
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug)]
pub struct ZeroLoss;
impl LossFunction for ZeroLoss {
    fn loss(&self, x: f64) -> f64 {
        return 0.0;
    }

    fn derivative(&self, x: f64) -> f64 {
        return 0.0;
    }
}

#[derive(Clone, Debug)]
pub struct IdentityLoss;
impl LossFunction for IdentityLoss {
    fn loss(&self, x: f64) -> f64 {
        return x;
    }

    fn derivative(&self, x: f64) -> f64 {
        return 1.0;
    }
}

#[derive(Clone, Debug)]
pub struct LinearLoss {
    coefficient: f64
}
impl LossFunction for LinearLoss {
    fn loss(&self, x: f64) -> f64 {
        return self.coefficient * x;
    }

    fn derivative(&self, x: f64) -> f64 {
        return self.coefficient;
    }
}

#[derive(Clone, Debug)]
pub struct QuadraticLoss {
    coefficient: f64
}
impl LossFunction for QuadraticLoss {
    fn loss(&self, x: f64) -> f64 {
        return self.coefficient * x.powi(2);
    }

    fn derivative(&self, x: f64) -> f64 {
        return self.coefficient * 2.0 * x;
    }
}

#[derive(Clone, Debug)]
pub struct PolynomialLoss {
    coefficient: f64,
    exponent: f64
}
impl LossFunction for PolynomialLoss {
    fn loss(&self, x: f64) -> f64 {
        return self.coefficient * x.powf(self.exponent);
    }

    fn derivative(&self, x: f64) -> f64 {
        return self.coefficient * self.exponent * x.powf(self.exponent - 1.0);
    }
}

#[derive(Clone, Debug)]
pub struct GrooveLoss {
    t: f64,
    d: i32,
    c: f64,
    f: f64,
    g: i32
}
impl GrooveLoss {
    pub fn new(t: f64, d: i32, c: f64, f: f64, g: i32) -> Self {
        Self { t,d,c,f,g }
    }
}
impl LossFunction for GrooveLoss {
    fn loss(&self, x: f64) -> f64 {
        (-( (-(x - self.t).powi(self.d)) / (2.0 * self.c.powi(2) ) ).exp() + self.f * (x - self.t).powi(self.g)) + 1.0
    }

    fn derivative(&self, x: f64) -> f64 {
        -( (-(x - self.t).powi(self.d)) / (2.0 * self.c.powi(2) ) ).exp() *  ((-self.d as f64 * (x - self.t)) /  (2.0 * self.c.powi(2))) + self.g as f64 * self.f * (x - self.t)
    }
}