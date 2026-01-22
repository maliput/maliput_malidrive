//! Ground curve trait for 2D reference curves.
//!
//! Ground curves represent the reference line of a road in the XY plane.
//! They are parameterized by a parameter p in [p0, p1].

use crate::common::MalidriveResult;
use nalgebra::Vector2;

/// Small epsilon value for numerical comparisons in ground curves.
pub const GROUND_CURVE_EPSILON: f64 = 1e-13;

/// Trait for parametric 2D ground curves.
///
/// Ground curves describe the reference line of a road projected onto the XY plane.
/// They are G¹ continuous (continuous with continuous first derivative) functions
/// that map a parameter p to 2D coordinates.
///
/// In mathematical terms:
/// - G(p) maps p ∈ [p0, p1] to (x, y) ∈ ℝ²
/// - G'(p) is the tangent vector
/// - θ(p) is the heading angle of the tangent
/// - G⁻¹(x, y) returns the parameter p closest to the given point
pub trait GroundCurve: Send + Sync {
    /// Converts from XODR s-coordinate to the internal p parameter.
    ///
    /// # Arguments
    /// * `xodr_p` - The XODR s-coordinate.
    ///
    /// # Returns
    /// The internal p parameter.
    fn p_from_xodr_p(&self, xodr_p: f64) -> MalidriveResult<f64>;

    /// Evaluates the curve at the given parameter.
    ///
    /// # Arguments
    /// * `p` - The parameter value. Must be in [p0(), p1()].
    ///
    /// # Returns
    /// The 2D point G(p) = (x, y).
    fn g(&self, p: f64) -> MalidriveResult<Vector2<f64>>;

    /// Evaluates the tangent vector at the given parameter.
    ///
    /// # Arguments
    /// * `p` - The parameter value. Must be in [p0(), p1()].
    ///
    /// # Returns
    /// The tangent vector G'(p).
    fn g_dot(&self, p: f64) -> MalidriveResult<Vector2<f64>>;

    /// Evaluates the heading angle at the given parameter.
    ///
    /// # Arguments
    /// * `p` - The parameter value. Must be in [p0(), p1()].
    ///
    /// # Returns
    /// The heading angle θ(p) in radians, in the range [-π, π].
    fn heading(&self, p: f64) -> MalidriveResult<f64>;

    /// Evaluates the derivative of the heading at the given parameter.
    ///
    /// # Arguments
    /// * `p` - The parameter value. Must be in [p0(), p1()].
    ///
    /// # Returns
    /// The heading derivative θ'(p) (related to curvature).
    fn heading_dot(&self, p: f64) -> MalidriveResult<f64>;

    /// Computes the inverse of the curve: finds p for the closest point to (x, y).
    ///
    /// # Arguments
    /// * `xy` - A 2D point.
    ///
    /// # Returns
    /// The parameter p that minimizes the distance to the given point.
    fn g_inverse(&self, xy: &Vector2<f64>) -> MalidriveResult<f64>;

    /// Returns the arc length of the curve.
    fn arc_length(&self) -> f64;

    /// Returns the linear tolerance used by this curve.
    fn linear_tolerance(&self) -> f64;

    /// Returns the lower bound of the parameter range.
    fn p0(&self) -> f64;

    /// Returns the upper bound of the parameter range.
    fn p1(&self) -> f64;

    /// Returns true if the curve is G¹ continuous.
    fn is_g1_contiguous(&self) -> bool;
}

/// Validates that a parameter is within the valid range with tolerance.
pub fn validate_p(p: f64, p0: f64, p1: f64, tolerance: f64) -> MalidriveResult<f64> {
    if p < p0 - tolerance - GROUND_CURVE_EPSILON {
        return Err(crate::common::MalidriveError::ParameterOutOfRange {
            parameter: "p".to_string(),
            value: p,
            min: p0,
            max: p1,
        });
    }
    if p > p1 + tolerance + GROUND_CURVE_EPSILON {
        return Err(crate::common::MalidriveError::ParameterOutOfRange {
            parameter: "p".to_string(),
            value: p,
            min: p0,
            max: p1,
        });
    }
    // Clamp to valid range
    Ok(p.clamp(p0, p1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_p_in_range() {
        let result = validate_p(50.0, 0.0, 100.0, 0.01);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 50.0);
    }

    #[test]
    fn test_validate_p_at_bounds() {
        let result = validate_p(0.0, 0.0, 100.0, 0.01);
        assert!(result.is_ok());

        let result = validate_p(100.0, 0.0, 100.0, 0.01);
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_p_with_tolerance() {
        // Slightly outside but within tolerance
        let result = validate_p(-0.005, 0.0, 100.0, 0.01);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 0.0); // Clamped

        let result = validate_p(100.005, 0.0, 100.0, 0.01);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 100.0); // Clamped
    }

    #[test]
    fn test_validate_p_out_of_range() {
        let result = validate_p(-1.0, 0.0, 100.0, 0.01);
        assert!(result.is_err());

        let result = validate_p(101.0, 0.0, 100.0, 0.01);
        assert!(result.is_err());
    }
}
