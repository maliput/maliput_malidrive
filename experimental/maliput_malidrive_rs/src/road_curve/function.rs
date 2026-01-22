//! Function trait for scalar functions of a single parameter.
//!
//! Functions are used for elevation profiles, superelevation, lane widths,
//! and lane offsets.

use crate::common::MalidriveResult;

/// Trait for G¹ scalar functions of a single parameter.
///
/// Functions must be G¹ continuous (continuous with continuous first derivative)
/// in their domain [p0, p1].
pub trait Function: Send + Sync {
    /// Evaluates the function at the given parameter.
    ///
    /// # Arguments
    /// * `p` - The parameter value. Must be in [p0(), p1()].
    ///
    /// # Returns
    /// The function value f(p).
    fn f(&self, p: f64) -> MalidriveResult<f64>;

    /// Evaluates the first derivative at the given parameter.
    ///
    /// # Arguments
    /// * `p` - The parameter value. Must be in [p0(), p1()].
    ///
    /// # Returns
    /// The derivative f'(p).
    fn f_dot(&self, p: f64) -> MalidriveResult<f64>;

    /// Evaluates the second derivative at the given parameter.
    ///
    /// # Arguments
    /// * `p` - The parameter value. Must be in [p0(), p1()].
    ///
    /// # Returns
    /// The second derivative f''(p).
    fn f_dot_dot(&self, p: f64) -> MalidriveResult<f64>;

    /// Returns the lower bound of the parameter range.
    fn p0(&self) -> f64;

    /// Returns the upper bound of the parameter range.
    fn p1(&self) -> f64;

    /// Returns true if the function is G¹ continuous in [p0, p1].
    fn is_g1_contiguous(&self) -> bool;
}

/// A constant function that returns the same value for all parameters.
#[derive(Debug, Clone, Copy)]
pub struct ConstantFunction {
    value: f64,
    p0: f64,
    p1: f64,
}

impl ConstantFunction {
    /// Creates a new constant function.
    pub fn new(value: f64, p0: f64, p1: f64) -> Self {
        Self { value, p0, p1 }
    }

    /// Creates a zero constant function.
    pub fn zero(p0: f64, p1: f64) -> Self {
        Self::new(0.0, p0, p1)
    }
}

impl Function for ConstantFunction {
    fn f(&self, _p: f64) -> MalidriveResult<f64> {
        Ok(self.value)
    }

    fn f_dot(&self, _p: f64) -> MalidriveResult<f64> {
        Ok(0.0)
    }

    fn f_dot_dot(&self, _p: f64) -> MalidriveResult<f64> {
        Ok(0.0)
    }

    fn p0(&self) -> f64 {
        self.p0
    }

    fn p1(&self) -> f64 {
        self.p1
    }

    fn is_g1_contiguous(&self) -> bool {
        true
    }
}

/// A linear function f(p) = a + b*(p - p0).
#[derive(Debug, Clone, Copy)]
pub struct LinearFunction {
    a: f64,
    b: f64,
    p0: f64,
    p1: f64,
}

impl LinearFunction {
    /// Creates a new linear function.
    ///
    /// # Arguments
    /// * `a` - The y-intercept (value at p0).
    /// * `b` - The slope.
    /// * `p0` - The lower bound of the parameter range.
    /// * `p1` - The upper bound of the parameter range.
    pub fn new(a: f64, b: f64, p0: f64, p1: f64) -> Self {
        Self { a, b, p0, p1 }
    }

    /// Creates a function with value `start` at `p0` and `end` at `p1`.
    pub fn from_endpoints(start: f64, end: f64, p0: f64, p1: f64) -> Self {
        let b = if (p1 - p0).abs() > 1e-15 {
            (end - start) / (p1 - p0)
        } else {
            0.0
        };
        Self::new(start, b, p0, p1)
    }
}

impl Function for LinearFunction {
    fn f(&self, p: f64) -> MalidriveResult<f64> {
        Ok(self.a + self.b * (p - self.p0))
    }

    fn f_dot(&self, _p: f64) -> MalidriveResult<f64> {
        Ok(self.b)
    }

    fn f_dot_dot(&self, _p: f64) -> MalidriveResult<f64> {
        Ok(0.0)
    }

    fn p0(&self) -> f64 {
        self.p0
    }

    fn p1(&self) -> f64 {
        self.p1
    }

    fn is_g1_contiguous(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_constant_function() {
        let f = ConstantFunction::new(5.0, 0.0, 100.0);
        assert_relative_eq!(f.f(0.0).unwrap(), 5.0);
        assert_relative_eq!(f.f(50.0).unwrap(), 5.0);
        assert_relative_eq!(f.f(100.0).unwrap(), 5.0);
        assert_relative_eq!(f.f_dot(50.0).unwrap(), 0.0);
        assert_relative_eq!(f.f_dot_dot(50.0).unwrap(), 0.0);
        assert!(f.is_g1_contiguous());
    }

    #[test]
    fn test_linear_function() {
        let f = LinearFunction::new(10.0, 0.5, 0.0, 100.0);
        assert_relative_eq!(f.f(0.0).unwrap(), 10.0);
        assert_relative_eq!(f.f(50.0).unwrap(), 35.0); // 10 + 0.5*50
        assert_relative_eq!(f.f(100.0).unwrap(), 60.0); // 10 + 0.5*100
        assert_relative_eq!(f.f_dot(50.0).unwrap(), 0.5);
        assert_relative_eq!(f.f_dot_dot(50.0).unwrap(), 0.0);
        assert!(f.is_g1_contiguous());
    }

    #[test]
    fn test_linear_function_from_endpoints() {
        let f = LinearFunction::from_endpoints(0.0, 100.0, 0.0, 100.0);
        assert_relative_eq!(f.f(0.0).unwrap(), 0.0);
        assert_relative_eq!(f.f(50.0).unwrap(), 50.0);
        assert_relative_eq!(f.f(100.0).unwrap(), 100.0);
        assert_relative_eq!(f.f_dot(50.0).unwrap(), 1.0);
    }
}
