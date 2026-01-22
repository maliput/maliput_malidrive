//! Line ground curve implementation.
//!
//! A line ground curve represents a straight line segment in the XY plane.

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{validate_p, GroundCurve, GROUND_CURVE_EPSILON};
use nalgebra::Vector2;

/// A ground curve representing a straight line segment.
///
/// The line is defined by a start point and a direction vector.
/// The curve is parameterized by p ∈ [p0, p1].
#[derive(Debug, Clone)]
pub struct LineGroundCurve {
    /// Linear tolerance for parameter validation.
    linear_tolerance: f64,
    /// Start point of the line.
    xy0: Vector2<f64>,
    /// Direction vector (end - start).
    dxy: Vector2<f64>,
    /// Arc length (norm of dxy).
    arc_length: f64,
    /// Heading angle.
    heading: f64,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
}

impl LineGroundCurve {
    /// Creates a new line ground curve.
    ///
    /// # Arguments
    /// * `linear_tolerance` - Linear tolerance for parameter validation. Must be positive.
    /// * `xy0` - Start point of the line.
    /// * `dxy` - Direction vector (difference from start to end point).
    /// * `p0` - Start parameter. Must be non-negative.
    /// * `p1` - End parameter. Must be greater than p0 by at least GROUND_CURVE_EPSILON.
    ///
    /// # Errors
    /// Returns an error if:
    /// - `linear_tolerance` is not positive
    /// - `dxy` has norm less than GROUND_CURVE_EPSILON
    /// - `p0` is negative
    /// - `p1 - p0` is less than GROUND_CURVE_EPSILON
    pub fn new(
        linear_tolerance: f64,
        xy0: Vector2<f64>,
        dxy: Vector2<f64>,
        p0: f64,
        p1: f64,
    ) -> MalidriveResult<Self> {
        if linear_tolerance <= 0.0 {
            return Err(MalidriveError::RoadGeometryConstructionError(
                "linear_tolerance must be positive".to_string(),
            ));
        }

        let arc_length = dxy.norm();
        if arc_length < GROUND_CURVE_EPSILON {
            return Err(MalidriveError::RoadGeometryConstructionError(
                "dxy norm must be at least GROUND_CURVE_EPSILON".to_string(),
            ));
        }

        if p0 < 0.0 {
            return Err(MalidriveError::RoadGeometryConstructionError(
                "p0 must be non-negative".to_string(),
            ));
        }

        if p1 - p0 < GROUND_CURVE_EPSILON {
            return Err(MalidriveError::RoadGeometryConstructionError(
                "p1 - p0 must be at least GROUND_CURVE_EPSILON".to_string(),
            ));
        }

        let heading = dxy.y.atan2(dxy.x);

        Ok(Self {
            linear_tolerance,
            xy0,
            dxy,
            arc_length,
            heading,
            p0,
            p1,
        })
    }

    /// Creates a line from a start point, heading angle, and length.
    ///
    /// # Arguments
    /// * `linear_tolerance` - Linear tolerance for geometric comparisons
    /// * `xy0` - Start point in 2D
    /// * `heading` - Heading angle in radians (0 = east, PI/2 = north)
    /// * `length` - Arc length of the line
    /// * `p0` - Start parameter (p coordinate)
    /// * `p1` - End parameter (p coordinate)
    ///
    /// # Errors
    /// Returns an error if validation fails.
    pub fn from_heading(
        linear_tolerance: f64,
        xy0: Vector2<f64>,
        heading: f64,
        length: f64,
        p0: f64,
        p1: f64,
    ) -> MalidriveResult<Self> {
        let dxy = Vector2::new(heading.cos() * length, heading.sin() * length);
        Self::new(linear_tolerance, xy0, dxy, p0, p1)
    }

    /// Creates a line from start and end points.
    pub fn from_endpoints(
        linear_tolerance: f64,
        start: Vector2<f64>,
        end: Vector2<f64>,
        p0: f64,
        p1: f64,
    ) -> MalidriveResult<Self> {
        let dxy = end - start;
        Self::new(linear_tolerance, start, dxy, p0, p1)
    }
}

impl GroundCurve for LineGroundCurve {
    fn p_from_xodr_p(&self, xodr_p: f64) -> MalidriveResult<f64> {
        validate_p(xodr_p, self.p0, self.p1, self.linear_tolerance)
    }

    fn g(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        let p = validate_p(p, self.p0, self.p1, self.linear_tolerance)?;
        let t = (p - self.p0) / (self.p1 - self.p0);
        Ok(self.xy0 + t * self.dxy)
    }

    fn g_dot(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        validate_p(p, self.p0, self.p1, self.linear_tolerance)?;
        // Derivative with respect to p: dxy * (1 / (p1 - p0))
        Ok(self.dxy / (self.p1 - self.p0))
    }

    fn heading(&self, p: f64) -> MalidriveResult<f64> {
        validate_p(p, self.p0, self.p1, self.linear_tolerance)?;
        Ok(self.heading)
    }

    fn heading_dot(&self, p: f64) -> MalidriveResult<f64> {
        validate_p(p, self.p0, self.p1, self.linear_tolerance)?;
        Ok(0.0) // Line has constant heading, so derivative is zero
    }

    fn g_inverse(&self, xy: &Vector2<f64>) -> MalidriveResult<f64> {
        // Project xy onto the line and find the parameter
        let v = xy - self.xy0;
        let unit_dxy = self.dxy / self.arc_length;
        let projection = v.dot(&unit_dxy);

        // Convert projection distance to parameter p
        let t = projection / self.arc_length;
        let p = self.p0 + t * (self.p1 - self.p0);

        // Clamp to valid range
        Ok(p.clamp(self.p0, self.p1))
    }

    fn arc_length(&self) -> f64 {
        self.arc_length
    }

    fn linear_tolerance(&self) -> f64 {
        self.linear_tolerance
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
    use std::f64::consts::PI;

    #[test]
    fn test_line_ground_curve_creation() {
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        )
        .unwrap();

        assert_relative_eq!(curve.arc_length(), 100.0);
        assert_relative_eq!(curve.p0(), 0.0);
        assert_relative_eq!(curve.p1(), 100.0);
        assert_relative_eq!(curve.heading, 0.0);
    }

    #[test]
    fn test_line_ground_curve_from_endpoints() {
        let curve = LineGroundCurve::from_endpoints(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 100.0),
            0.0,
            100.0,
        )
        .unwrap();

        assert_relative_eq!(curve.arc_length(), (2.0_f64).sqrt() * 100.0);
        assert_relative_eq!(curve.heading, PI / 4.0);
    }

    #[test]
    fn test_line_ground_curve_from_heading() {
        let curve =
            LineGroundCurve::from_heading(0.01, Vector2::new(0.0, 0.0), PI / 4.0, 100.0, 0.0, 100.0)
                .unwrap();

        assert_relative_eq!(curve.arc_length(), 100.0);
        assert_relative_eq!(curve.heading, PI / 4.0);
    }

    #[test]
    fn test_line_ground_curve_g() {
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        )
        .unwrap();

        let p0 = curve.g(0.0).unwrap();
        assert_relative_eq!(p0.x, 0.0);
        assert_relative_eq!(p0.y, 0.0);

        let p50 = curve.g(50.0).unwrap();
        assert_relative_eq!(p50.x, 50.0);
        assert_relative_eq!(p50.y, 0.0);

        let p100 = curve.g(100.0).unwrap();
        assert_relative_eq!(p100.x, 100.0);
        assert_relative_eq!(p100.y, 0.0);
    }

    #[test]
    fn test_line_ground_curve_g_dot() {
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        )
        .unwrap();

        let tangent = curve.g_dot(50.0).unwrap();
        assert_relative_eq!(tangent.x, 1.0); // dxy / (p1 - p0) = 100 / 100 = 1
        assert_relative_eq!(tangent.y, 0.0);
    }

    #[test]
    fn test_line_ground_curve_heading() {
        // Horizontal line (heading = 0)
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        )
        .unwrap();
        assert_relative_eq!(curve.heading(50.0).unwrap(), 0.0);

        // 45-degree line
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 100.0),
            0.0,
            100.0,
        )
        .unwrap();
        assert_relative_eq!(curve.heading(50.0).unwrap(), PI / 4.0);

        // Vertical line (heading = π/2)
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, 100.0),
            0.0,
            100.0,
        )
        .unwrap();
        assert_relative_eq!(curve.heading(50.0).unwrap(), PI / 2.0);
    }

    #[test]
    fn test_line_ground_curve_heading_dot() {
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        )
        .unwrap();

        // Line has constant heading
        assert_relative_eq!(curve.heading_dot(50.0).unwrap(), 0.0);
    }

    #[test]
    fn test_line_ground_curve_g_inverse() {
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        )
        .unwrap();

        // Point on the line
        let p = curve.g_inverse(&Vector2::new(50.0, 0.0)).unwrap();
        assert_relative_eq!(p, 50.0);

        // Point off the line (perpendicular)
        let p = curve.g_inverse(&Vector2::new(50.0, 10.0)).unwrap();
        assert_relative_eq!(p, 50.0);

        // Point before the line (clamped to p0)
        let p = curve.g_inverse(&Vector2::new(-10.0, 0.0)).unwrap();
        assert_relative_eq!(p, 0.0);

        // Point after the line (clamped to p1)
        let p = curve.g_inverse(&Vector2::new(110.0, 0.0)).unwrap();
        assert_relative_eq!(p, 100.0);
    }

    #[test]
    fn test_line_ground_curve_validation_errors() {
        // Negative linear tolerance
        let result = LineGroundCurve::new(
            -0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        );
        assert!(result.is_err());

        // Zero-length line
        let result = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(0.0, 0.0),
            0.0,
            100.0,
        );
        assert!(result.is_err());

        // Negative p0
        let result = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            -1.0,
            100.0,
        );
        assert!(result.is_err());

        // p1 <= p0
        let result = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            100.0,
            100.0,
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_line_ground_curve_is_g1_contiguous() {
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        )
        .unwrap();

        assert!(curve.is_g1_contiguous());
    }

    // Additional tests based on C++ test patterns
    #[test]
    fn test_line_ground_curve_with_nonzero_p0() {
        // Test with p0=10, p1=20 as in C++ tests
        let linear_tolerance = 0.01;
        let p0 = 10.0;
        let p1 = 20.0;
        let xy0 = Vector2::new(0.0, 0.0);
        let dxy = Vector2::new(1.0, 0.0);

        let curve = LineGroundCurve::new(linear_tolerance, xy0, dxy, p0, p1).unwrap();

        assert_relative_eq!(curve.p0(), p0);
        assert_relative_eq!(curve.p1(), p1);
        assert_relative_eq!(curve.arc_length(), 1.0);

        // G at p0 should be xy0
        let g_p0 = curve.g(p0).unwrap();
        assert_relative_eq!(g_p0.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(g_p0.y, 0.0, epsilon = 1e-12);

        // G at p1 should be xy0 + dxy
        let g_p1 = curve.g(p1).unwrap();
        assert_relative_eq!(g_p1.x, 1.0, epsilon = 1e-12);
        assert_relative_eq!(g_p1.y, 0.0, epsilon = 1e-12);

        // G at midpoint
        let p_mid = (p0 + p1) / 2.0; // 15.0
        let g_mid = curve.g(p_mid).unwrap();
        assert_relative_eq!(g_mid.x, 0.5, epsilon = 1e-12);
        assert_relative_eq!(g_mid.y, 0.0, epsilon = 1e-12);

        // GDot = dxy / (p1 - p0) = (1, 0) / 10 = (0.1, 0)
        let g_dot = curve.g_dot(p0).unwrap();
        assert_relative_eq!(g_dot.x, 0.1, epsilon = 1e-12);
        assert_relative_eq!(g_dot.y, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn test_line_ground_curve_quadrant1() {
        // Line in quadrant 1 direction: (3, 4) from (1, 0)
        let linear_tolerance = 0.01;
        let p0 = 10.0;
        let p1 = 20.0;
        let xy0 = Vector2::new(1.0, 0.0);
        let dxy = Vector2::new(3.0, 4.0);

        let curve = LineGroundCurve::new(linear_tolerance, xy0, dxy, p0, p1).unwrap();

        assert_relative_eq!(curve.arc_length(), 5.0, epsilon = 1e-12);

        // G at p0
        let g_p0 = curve.g(p0).unwrap();
        assert_relative_eq!(g_p0.x, 1.0, epsilon = 1e-12);
        assert_relative_eq!(g_p0.y, 0.0, epsilon = 1e-12);

        // G at p1
        let g_p1 = curve.g(p1).unwrap();
        assert_relative_eq!(g_p1.x, 4.0, epsilon = 1e-12);
        assert_relative_eq!(g_p1.y, 4.0, epsilon = 1e-12);

        // G at midpoint
        let g_mid = curve.g(15.0).unwrap();
        assert_relative_eq!(g_mid.x, 2.5, epsilon = 1e-12);
        assert_relative_eq!(g_mid.y, 2.0, epsilon = 1e-12);

        // GDot = dxy / (p1 - p0) = (3, 4) / 10 = (0.3, 0.4)
        let g_dot = curve.g_dot(p0).unwrap();
        assert_relative_eq!(g_dot.x, 0.3, epsilon = 1e-12);
        assert_relative_eq!(g_dot.y, 0.4, epsilon = 1e-12);

        // Heading: atan2(4, 3) ≈ 0.9272952180016122
        let heading = curve.heading(p0).unwrap();
        assert_relative_eq!(heading, 0.9272952180016122, epsilon = 1e-12);
    }

    #[test]
    fn test_line_ground_curve_quadrant3() {
        // Line in quadrant 3 direction: (-4, -3) from (10, 9)
        let linear_tolerance = 0.01;
        let p0 = 10.0;
        let p1 = 20.0;
        let xy0 = Vector2::new(10.0, 9.0);
        let dxy = Vector2::new(-4.0, -3.0);

        let curve = LineGroundCurve::new(linear_tolerance, xy0, dxy, p0, p1).unwrap();

        assert_relative_eq!(curve.arc_length(), 5.0, epsilon = 1e-12);

        // G at p0
        let g_p0 = curve.g(p0).unwrap();
        assert_relative_eq!(g_p0.x, 10.0, epsilon = 1e-12);
        assert_relative_eq!(g_p0.y, 9.0, epsilon = 1e-12);

        // G at p1
        let g_p1 = curve.g(p1).unwrap();
        assert_relative_eq!(g_p1.x, 6.0, epsilon = 1e-12);
        assert_relative_eq!(g_p1.y, 6.0, epsilon = 1e-12);

        // G at midpoint
        let g_mid = curve.g(15.0).unwrap();
        assert_relative_eq!(g_mid.x, 8.0, epsilon = 1e-12);
        assert_relative_eq!(g_mid.y, 7.5, epsilon = 1e-12);

        // GDot = dxy / (p1 - p0) = (-4, -3) / 10 = (-0.4, -0.3)
        let g_dot = curve.g_dot(p0).unwrap();
        assert_relative_eq!(g_dot.x, -0.4, epsilon = 1e-12);
        assert_relative_eq!(g_dot.y, -0.3, epsilon = 1e-12);

        // Heading: atan2(-3, -4) ≈ -2.498091544796509
        let heading = curve.heading(p0).unwrap();
        assert_relative_eq!(heading, -2.498091544796509, epsilon = 1e-12);
    }

    #[test]
    fn test_line_ground_curve_parameter_tolerance() {
        // Test that parameters slightly outside range but within tolerance are accepted
        let linear_tolerance = 0.01;
        let p0 = 10.0;
        let p1 = 20.0;
        let xy0 = Vector2::new(0.0, 0.0);
        let dxy = Vector2::new(1.0, 0.0);

        let curve = LineGroundCurve::new(linear_tolerance, xy0, dxy, p0, p1).unwrap();

        // Within tolerance
        assert!(curve.g(9.995).is_ok());
        assert!(curve.g(20.005).is_ok());

        // Outside tolerance - exceeds linear_tolerance + GROUND_CURVE_EPSILON
        assert!(curve.g(9.98).is_err());
        assert!(curve.g(20.02).is_err());
    }

    #[test]
    fn test_line_ground_curve_p_from_xodr_p() {
        let curve = LineGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            Vector2::new(100.0, 0.0),
            0.0,
            100.0,
        )
        .unwrap();

        // p_from_xodr_p should validate and return the same p
        assert_relative_eq!(curve.p_from_xodr_p(50.0).unwrap(), 50.0);

        // Should fail for out of range values
        assert!(curve.p_from_xodr_p(-1.0).is_err());
        assert!(curve.p_from_xodr_p(101.0).is_err());
    }
}
