//! Arc ground curve implementation.
//!
//! An arc ground curve represents a circular arc segment in the XY plane.

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{validate_p, GroundCurve, GROUND_CURVE_EPSILON};
use nalgebra::Vector2;
use std::f64::consts::PI;

/// A ground curve representing a circular arc.
///
/// The arc is defined by a start point, start heading, curvature, and arc length.
/// Positive curvature turns counterclockwise (left), negative turns clockwise (right).
#[derive(Debug, Clone)]
pub struct ArcGroundCurve {
    /// Linear tolerance for parameter validation.
    linear_tolerance: f64,
    /// Start point of the arc.
    xy0: Vector2<f64>,
    /// Arc length.
    arc_length: f64,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
    /// Radius of the arc (1 / curvature).
    radius: f64,
    /// Total angle change along the arc.
    d_theta: f64,
    /// Angle from center to start point.
    theta0: f64,
    /// Center of the arc.
    center: Vector2<f64>,
}

impl ArcGroundCurve {
    /// Creates a new arc ground curve.
    ///
    /// # Arguments
    /// * `linear_tolerance` - Linear tolerance for parameter validation. Must be positive.
    /// * `xy0` - Start point of the arc.
    /// * `start_heading` - Heading at the start of the arc (tangent direction).
    /// * `curvature` - Curvature of the arc (1/radius). Positive = counterclockwise.
    /// * `arc_length` - Length of the arc. Must be at least GROUND_CURVE_EPSILON.
    /// * `p0` - Start parameter. Must be non-negative.
    /// * `p1` - End parameter. Must be greater than p0 by at least GROUND_CURVE_EPSILON.
    ///
    /// # Errors
    /// Returns an error if:
    /// - `linear_tolerance` is not positive
    /// - `arc_length` is less than GROUND_CURVE_EPSILON
    /// - `curvature` magnitude is less than GROUND_CURVE_EPSILON
    /// - `p0` is negative
    /// - `p1 - p0` is less than GROUND_CURVE_EPSILON
    pub fn new(
        linear_tolerance: f64,
        xy0: Vector2<f64>,
        start_heading: f64,
        curvature: f64,
        arc_length: f64,
        p0: f64,
        p1: f64,
    ) -> MalidriveResult<Self> {
        if linear_tolerance <= 0.0 {
            return Err(MalidriveError::RoadGeometryConstructionError(
                "linear_tolerance must be positive".to_string(),
            ));
        }

        if arc_length < GROUND_CURVE_EPSILON {
            return Err(MalidriveError::RoadGeometryConstructionError(
                "arc_length must be at least GROUND_CURVE_EPSILON".to_string(),
            ));
        }

        if curvature.abs() < GROUND_CURVE_EPSILON {
            return Err(MalidriveError::RoadGeometryConstructionError(
                "curvature magnitude must be at least GROUND_CURVE_EPSILON".to_string(),
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

        let radius = 1.0 / curvature;
        let d_theta = arc_length * curvature;

        // theta0 is the angle from the center to the start point
        // The tangent at the start is perpendicular to the radius
        // For positive curvature, center is to the left of the tangent
        let theta0 = start_heading - d_theta.signum() * PI / 2.0;

        // Center is radius away from start point, perpendicular to the tangent
        let center = xy0 - radius.abs() * Vector2::new(theta0.cos(), theta0.sin());

        Ok(Self {
            linear_tolerance,
            xy0,
            arc_length,
            p0,
            p1,
            radius,
            d_theta,
            theta0,
            center,
        })
    }

    /// Returns the curvature of the arc.
    pub fn curvature(&self) -> f64 {
        1.0 / self.radius
    }

    /// Computes the angle theta at parameter p.
    fn theta(&self, p: f64) -> f64 {
        self.theta0 + (p - self.p0) * self.heading_dot_internal()
    }

    /// Internal heading derivative computation (constant for arc).
    fn heading_dot_internal(&self) -> f64 {
        self.d_theta / (self.p1 - self.p0)
    }
}

impl GroundCurve for ArcGroundCurve {
    fn p_from_xodr_p(&self, xodr_p: f64) -> MalidriveResult<f64> {
        validate_p(xodr_p, self.p0, self.p1, self.linear_tolerance)
    }

    fn g(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        let p = validate_p(p, self.p0, self.p1, self.linear_tolerance)?;
        let theta = self.theta(p);
        Ok(self.center + self.radius.abs() * Vector2::new(theta.cos(), theta.sin()))
    }

    fn g_dot(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        let p = validate_p(p, self.p0, self.p1, self.linear_tolerance)?;
        let theta = self.theta(p);
        let theta_dot = self.heading_dot_internal();

        // Derivative of the arc position with respect to p
        Ok(self.radius.abs() * theta_dot * Vector2::new(-theta.sin(), theta.cos()))
    }

    fn heading(&self, p: f64) -> MalidriveResult<f64> {
        let p = validate_p(p, self.p0, self.p1, self.linear_tolerance)?;
        let theta = self.theta(p);
        // Heading is perpendicular to the radius, in the direction of travel
        let heading = theta + self.d_theta.signum() * PI / 2.0;
        // Normalize to [-π, π]
        Ok(normalize_angle(heading))
    }

    fn heading_dot(&self, p: f64) -> MalidriveResult<f64> {
        validate_p(p, self.p0, self.p1, self.linear_tolerance)?;
        Ok(self.heading_dot_internal())
    }

    fn g_inverse(&self, xy: &Vector2<f64>) -> MalidriveResult<f64> {
        // Find the angle from center to the query point
        let v = xy - self.center;
        let theta_xy = v.y.atan2(v.x);

        // Convert angle to parameter
        // theta = theta0 + (p - p0) * heading_dot
        // p = p0 + (theta - theta0) / heading_dot
        let theta_dot = self.heading_dot_internal();
        
        // Handle angle wrapping
        let mut delta_theta = theta_xy - self.theta0;
        
        // Normalize delta_theta based on the direction of travel
        if self.d_theta > 0.0 {
            while delta_theta < 0.0 {
                delta_theta += 2.0 * PI;
            }
            while delta_theta > 2.0 * PI {
                delta_theta -= 2.0 * PI;
            }
        } else {
            while delta_theta > 0.0 {
                delta_theta -= 2.0 * PI;
            }
            while delta_theta < -2.0 * PI {
                delta_theta += 2.0 * PI;
            }
        }

        let p = self.p0 + delta_theta / theta_dot;

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

/// Normalizes an angle to the range [-π, π].
fn normalize_angle(angle: f64) -> f64 {
    let mut result = angle;
    while result > PI {
        result -= 2.0 * PI;
    }
    while result < -PI {
        result += 2.0 * PI;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_arc_ground_curve_creation() {
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0, // heading = 0 (going east)
            0.1, // curvature = 0.1 (radius = 10, turning left)
            5.0, // arc length = 5 (about 28.6 degrees)
            0.0,
            5.0,
        )
        .unwrap();

        assert_relative_eq!(curve.arc_length(), 5.0);
        assert_relative_eq!(curve.curvature(), 0.1);
        assert_relative_eq!(curve.p0(), 0.0);
        assert_relative_eq!(curve.p1(), 5.0);
    }

    #[test]
    fn test_arc_ground_curve_start_point() {
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0,
            0.1,
            5.0,
            0.0,
            5.0,
        )
        .unwrap();

        let start = curve.g(0.0).unwrap();
        assert_relative_eq!(start.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(start.y, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_arc_ground_curve_start_heading() {
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0, // heading = 0
            0.1,
            5.0,
            0.0,
            5.0,
        )
        .unwrap();

        let heading = curve.heading(0.0).unwrap();
        assert_relative_eq!(heading, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_arc_ground_curve_heading_increases_ccw() {
        // Counterclockwise arc (positive curvature)
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0,
            0.1, // positive = ccw
            5.0,
            0.0,
            5.0,
        )
        .unwrap();

        let heading_start = curve.heading(0.0).unwrap();
        let heading_end = curve.heading(5.0).unwrap();

        // Heading should increase (turning left)
        assert!(heading_end > heading_start);
    }

    #[test]
    fn test_arc_ground_curve_heading_decreases_cw() {
        // Clockwise arc (negative curvature)
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0,
            -0.1, // negative = cw
            5.0,
            0.0,
            5.0,
        )
        .unwrap();

        let heading_start = curve.heading(0.0).unwrap();
        let heading_end = curve.heading(5.0).unwrap();

        // Heading should decrease (turning right)
        assert!(heading_end < heading_start);
    }

    #[test]
    fn test_arc_ground_curve_quarter_circle() {
        // Quarter circle: radius = 10, arc length = π * 10 / 2 = 5π
        let arc_length = PI * 10.0 / 2.0;
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0, // start heading east
            0.1, // radius = 10, turning left
            arc_length,
            0.0,
            arc_length,
        )
        .unwrap();

        // End point should be at approximately (10, 10)
        let end = curve.g(arc_length).unwrap();
        assert_relative_eq!(end.x, 10.0, epsilon = 1e-6);
        assert_relative_eq!(end.y, 10.0, epsilon = 1e-6);

        // End heading should be π/2 (north)
        let end_heading = curve.heading(arc_length).unwrap();
        assert_relative_eq!(end_heading, PI / 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_arc_ground_curve_heading_dot() {
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0,
            0.1,
            5.0,
            0.0,
            5.0,
        )
        .unwrap();

        // Heading derivative should be constant (curvature * arc_length / parameter_range)
        let hdot = curve.heading_dot(2.5).unwrap();
        let expected = 0.1 * 5.0 / 5.0; // curvature * arc_length / (p1 - p0) = 0.1
        assert_relative_eq!(hdot, expected, epsilon = 1e-10);
    }

    #[test]
    fn test_arc_ground_curve_g_inverse() {
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0,
            0.1,
            5.0,
            0.0,
            5.0,
        )
        .unwrap();

        // Test inverse at start
        let p = curve.g_inverse(&Vector2::new(0.0, 0.0)).unwrap();
        assert_relative_eq!(p, 0.0, epsilon = 0.1);

        // Test inverse at a midpoint
        let mid_point = curve.g(2.5).unwrap();
        let p_mid = curve.g_inverse(&mid_point).unwrap();
        assert_relative_eq!(p_mid, 2.5, epsilon = 0.1);
    }

    #[test]
    fn test_arc_ground_curve_validation_errors() {
        // Zero curvature
        let result = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0,
            0.0, // zero curvature
            5.0,
            0.0,
            5.0,
        );
        assert!(result.is_err());

        // Negative arc length
        let result = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0,
            0.1,
            -5.0, // negative
            0.0,
            5.0,
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_arc_ground_curve_is_g1_contiguous() {
        let curve = ArcGroundCurve::new(
            0.01,
            Vector2::new(0.0, 0.0),
            0.0,
            0.1,
            5.0,
            0.0,
            5.0,
        )
        .unwrap();

        assert!(curve.is_g1_contiguous());
    }

    #[test]
    fn test_normalize_angle() {
        assert_relative_eq!(normalize_angle(0.0), 0.0);
        assert_relative_eq!(normalize_angle(PI), PI, epsilon = 1e-10);
        assert_relative_eq!(normalize_angle(-PI), -PI, epsilon = 1e-10);
        assert_relative_eq!(normalize_angle(2.0 * PI), 0.0, epsilon = 1e-10);
        assert_relative_eq!(normalize_angle(-2.0 * PI), 0.0, epsilon = 1e-10);
        assert_relative_eq!(normalize_angle(3.0 * PI), PI, epsilon = 1e-10);
    }
}
