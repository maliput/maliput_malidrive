//! Cubic polynomial function implementation.
//!
//! Cubic polynomials are used for elevation, superelevation, lane width,
//! and lane offset descriptions in OpenDRIVE.

use crate::common::MalidriveResult;
use crate::road_curve::Function;

/// A cubic polynomial function f(p) = a + b*(p-p0) + c*(p-p0)² + d*(p-p0)³
#[derive(Debug, Clone, Copy)]
pub struct CubicPolynomial {
    /// Constant coefficient (value at p0).
    pub a: f64,
    /// Linear coefficient.
    pub b: f64,
    /// Quadratic coefficient.
    pub c: f64,
    /// Cubic coefficient.
    pub d: f64,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
}

impl CubicPolynomial {
    /// Creates a new cubic polynomial.
    ///
    /// # Arguments
    /// * `a` - Constant coefficient.
    /// * `b` - Linear coefficient.
    /// * `c` - Quadratic coefficient.
    /// * `d` - Cubic coefficient.
    /// * `p0` - Start parameter.
    /// * `p1` - End parameter.
    pub fn new(a: f64, b: f64, c: f64, d: f64, p0: f64, p1: f64) -> Self {
        Self { a, b, c, d, p0, p1 }
    }

    /// Creates a constant polynomial.
    pub fn constant(value: f64, p0: f64, p1: f64) -> Self {
        Self::new(value, 0.0, 0.0, 0.0, p0, p1)
    }

    /// Creates a linear polynomial.
    pub fn linear(a: f64, b: f64, p0: f64, p1: f64) -> Self {
        Self::new(a, b, 0.0, 0.0, p0, p1)
    }

    /// Creates a quadratic polynomial.
    pub fn quadratic(a: f64, b: f64, c: f64, p0: f64, p1: f64) -> Self {
        Self::new(a, b, c, 0.0, p0, p1)
    }

    /// Returns true if this is a constant polynomial.
    pub fn is_constant(&self) -> bool {
        self.b == 0.0 && self.c == 0.0 && self.d == 0.0
    }

    /// Returns true if this is a linear polynomial (c = d = 0).
    pub fn is_linear(&self) -> bool {
        self.c == 0.0 && self.d == 0.0
    }

    /// Returns true if this is a quadratic polynomial (d = 0).
    pub fn is_quadratic(&self) -> bool {
        self.d == 0.0
    }

    /// Evaluates the polynomial at p without range checking.
    pub fn evaluate(&self, p: f64) -> f64 {
        let dp = p - self.p0;
        self.a + self.b * dp + self.c * dp * dp + self.d * dp * dp * dp
    }

    /// Evaluates the first derivative at p without range checking.
    pub fn evaluate_dot(&self, p: f64) -> f64 {
        let dp = p - self.p0;
        self.b + 2.0 * self.c * dp + 3.0 * self.d * dp * dp
    }

    /// Evaluates the second derivative at p without range checking.
    pub fn evaluate_dot_dot(&self, p: f64) -> f64 {
        let dp = p - self.p0;
        2.0 * self.c + 6.0 * self.d * dp
    }

    /// Returns the value at the start (p = p0).
    pub fn value_at_start(&self) -> f64 {
        self.a
    }

    /// Returns the value at the end (p = p1).
    pub fn value_at_end(&self) -> f64 {
        self.evaluate(self.p1)
    }

    /// Returns the derivative at the start (p = p0).
    pub fn slope_at_start(&self) -> f64 {
        self.b
    }

    /// Returns the derivative at the end (p = p1).
    pub fn slope_at_end(&self) -> f64 {
        self.evaluate_dot(self.p1)
    }
}

impl Function for CubicPolynomial {
    fn f(&self, p: f64) -> MalidriveResult<f64> {
        Ok(self.evaluate(p))
    }

    fn f_dot(&self, p: f64) -> MalidriveResult<f64> {
        Ok(self.evaluate_dot(p))
    }

    fn f_dot_dot(&self, p: f64) -> MalidriveResult<f64> {
        Ok(self.evaluate_dot_dot(p))
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

/// A piecewise cubic polynomial function composed of multiple segments.
#[derive(Debug, Clone)]
pub struct PiecewiseCubicPolynomial {
    /// The polynomial segments, sorted by p0.
    segments: Vec<CubicPolynomial>,
}

impl PiecewiseCubicPolynomial {
    /// Creates a new piecewise polynomial from segments.
    pub fn new(segments: Vec<CubicPolynomial>) -> Self {
        let mut sorted = segments;
        sorted.sort_by(|a, b| a.p0.partial_cmp(&b.p0).unwrap());
        Self { segments: sorted }
    }

    /// Creates a piecewise polynomial with a single segment.
    pub fn single(segment: CubicPolynomial) -> Self {
        Self::new(vec![segment])
    }

    /// Adds a segment to the polynomial.
    pub fn add_segment(&mut self, segment: CubicPolynomial) {
        self.segments.push(segment);
        self.segments.sort_by(|a, b| a.p0.partial_cmp(&b.p0).unwrap());
    }

    /// Returns the segment that contains the given parameter.
    fn find_segment(&self, p: f64) -> Option<&CubicPolynomial> {
        // Find the last segment where p0 <= p
        self.segments.iter().rev().find(|s| s.p0 <= p)
    }

    /// Returns the number of segments.
    pub fn num_segments(&self) -> usize {
        self.segments.len()
    }

    /// Returns true if the polynomial is empty.
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }
}

impl Function for PiecewiseCubicPolynomial {
    fn f(&self, p: f64) -> MalidriveResult<f64> {
        self.find_segment(p)
            .map(|s| s.evaluate(p))
            .ok_or_else(|| {
                crate::common::MalidriveError::ParameterOutOfRange {
                    parameter: "p".to_string(),
                    value: p,
                    min: self.p0(),
                    max: self.p1(),
                }
            })
    }

    fn f_dot(&self, p: f64) -> MalidriveResult<f64> {
        self.find_segment(p)
            .map(|s| s.evaluate_dot(p))
            .ok_or_else(|| {
                crate::common::MalidriveError::ParameterOutOfRange {
                    parameter: "p".to_string(),
                    value: p,
                    min: self.p0(),
                    max: self.p1(),
                }
            })
    }

    fn f_dot_dot(&self, p: f64) -> MalidriveResult<f64> {
        self.find_segment(p)
            .map(|s| s.evaluate_dot_dot(p))
            .ok_or_else(|| {
                crate::common::MalidriveError::ParameterOutOfRange {
                    parameter: "p".to_string(),
                    value: p,
                    min: self.p0(),
                    max: self.p1(),
                }
            })
    }

    fn p0(&self) -> f64 {
        self.segments.first().map(|s| s.p0).unwrap_or(0.0)
    }

    fn p1(&self) -> f64 {
        self.segments.last().map(|s| s.p1).unwrap_or(0.0)
    }

    fn is_g1_contiguous(&self) -> bool {
        if self.segments.len() <= 1 {
            return true;
        }

        // Check that adjacent segments connect with matching values and derivatives
        let tolerance = 1e-9;
        for i in 0..self.segments.len() - 1 {
            let current = &self.segments[i];
            let next = &self.segments[i + 1];

            // Check value continuity
            let end_value = current.value_at_end();
            let start_value = next.value_at_start();
            if (end_value - start_value).abs() > tolerance {
                return false;
            }

            // Check derivative continuity (G1)
            let end_slope = current.slope_at_end();
            let start_slope = next.slope_at_start();
            if (end_slope - start_slope).abs() > tolerance {
                return false;
            }
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_cubic_polynomial_creation() {
        let poly = CubicPolynomial::new(1.0, 2.0, 3.0, 4.0, 0.0, 10.0);
        assert_relative_eq!(poly.a, 1.0);
        assert_relative_eq!(poly.b, 2.0);
        assert_relative_eq!(poly.c, 3.0);
        assert_relative_eq!(poly.d, 4.0);
    }

    #[test]
    fn test_cubic_polynomial_constant() {
        let poly = CubicPolynomial::constant(5.0, 0.0, 100.0);
        assert!(poly.is_constant());
        assert_relative_eq!(poly.evaluate(0.0), 5.0);
        assert_relative_eq!(poly.evaluate(50.0), 5.0);
        assert_relative_eq!(poly.evaluate(100.0), 5.0);
    }

    #[test]
    fn test_cubic_polynomial_linear() {
        let poly = CubicPolynomial::linear(0.0, 1.0, 0.0, 100.0);
        assert!(poly.is_linear());
        assert_relative_eq!(poly.evaluate(0.0), 0.0);
        assert_relative_eq!(poly.evaluate(50.0), 50.0);
        assert_relative_eq!(poly.evaluate(100.0), 100.0);
        assert_relative_eq!(poly.evaluate_dot(50.0), 1.0);
    }

    #[test]
    fn test_cubic_polynomial_quadratic() {
        // f(p) = p²
        let poly = CubicPolynomial::quadratic(0.0, 0.0, 1.0, 0.0, 10.0);
        assert!(poly.is_quadratic());
        assert_relative_eq!(poly.evaluate(0.0), 0.0);
        assert_relative_eq!(poly.evaluate(5.0), 25.0);
        assert_relative_eq!(poly.evaluate(10.0), 100.0);
        assert_relative_eq!(poly.evaluate_dot(5.0), 10.0); // 2*c*p = 2*1*5
        assert_relative_eq!(poly.evaluate_dot_dot(5.0), 2.0); // 2*c = 2
    }

    #[test]
    fn test_cubic_polynomial_full() {
        // f(p) = 1 + 2p + 3p² + 4p³
        let poly = CubicPolynomial::new(1.0, 2.0, 3.0, 4.0, 0.0, 10.0);
        assert_relative_eq!(poly.evaluate(0.0), 1.0);
        // f(1) = 1 + 2 + 3 + 4 = 10
        assert_relative_eq!(poly.evaluate(1.0), 10.0);
        // f'(p) = 2 + 6p + 12p²
        // f'(1) = 2 + 6 + 12 = 20
        assert_relative_eq!(poly.evaluate_dot(1.0), 20.0);
        // f''(p) = 6 + 24p
        // f''(1) = 6 + 24 = 30
        assert_relative_eq!(poly.evaluate_dot_dot(1.0), 30.0);
    }

    #[test]
    fn test_cubic_polynomial_with_offset() {
        // f(p) = 10 + 0.5*(p - 5) starting at p0 = 5
        let poly = CubicPolynomial::linear(10.0, 0.5, 5.0, 15.0);
        assert_relative_eq!(poly.evaluate(5.0), 10.0);
        assert_relative_eq!(poly.evaluate(10.0), 12.5); // 10 + 0.5*5
        assert_relative_eq!(poly.evaluate(15.0), 15.0); // 10 + 0.5*10
    }

    #[test]
    fn test_cubic_polynomial_function_trait() {
        let poly = CubicPolynomial::linear(0.0, 1.0, 0.0, 100.0);
        assert_relative_eq!(poly.f(50.0).unwrap(), 50.0);
        assert_relative_eq!(poly.f_dot(50.0).unwrap(), 1.0);
        assert_relative_eq!(poly.f_dot_dot(50.0).unwrap(), 0.0);
        assert!(poly.is_g1_contiguous());
    }

    #[test]
    fn test_piecewise_polynomial() {
        let seg1 = CubicPolynomial::constant(0.0, 0.0, 50.0);
        let seg2 = CubicPolynomial::constant(10.0, 50.0, 100.0);

        let piecewise = PiecewiseCubicPolynomial::new(vec![seg1, seg2]);

        assert_eq!(piecewise.num_segments(), 2);
        assert_relative_eq!(piecewise.f(25.0).unwrap(), 0.0);
        assert_relative_eq!(piecewise.f(75.0).unwrap(), 10.0);
    }

    #[test]
    fn test_piecewise_polynomial_continuity() {
        // Continuous function: f(p) = p for [0, 50], f(p) = p for [50, 100]
        let seg1 = CubicPolynomial::linear(0.0, 1.0, 0.0, 50.0);
        let seg2 = CubicPolynomial::linear(50.0, 1.0, 50.0, 100.0);

        let piecewise = PiecewiseCubicPolynomial::new(vec![seg1, seg2]);
        assert!(piecewise.is_g1_contiguous());
    }

    #[test]
    fn test_piecewise_polynomial_discontinuous() {
        // Discontinuous: jump at p=50
        let seg1 = CubicPolynomial::constant(0.0, 0.0, 50.0);
        let seg2 = CubicPolynomial::constant(10.0, 50.0, 100.0);

        let piecewise = PiecewiseCubicPolynomial::new(vec![seg1, seg2]);
        assert!(!piecewise.is_g1_contiguous());
    }
}
