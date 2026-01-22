//! Spiral (Euler spiral / clothoid) ground curve implementation.
//!
//! A spiral curve is defined by a linearly varying curvature along its length.
//! The curvature changes from curvature_start at the beginning to curvature_end
//! at the end of the curve.
//!
//! Euler spirals are commonly used for road transitions between straight sections
//! and circular arcs, as they provide smooth curvature transitions.

use nalgebra::Vector2;

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::GroundCurve;

/// A spiral (Euler spiral / clothoid) ground curve.
///
/// The curvature κ varies linearly with arc length s:
/// κ(s) = κ_start + (κ_end - κ_start) * s / L
///
/// where L is the total arc length.
///
/// The curve is computed using Fresnel integrals:
/// x(s) = ∫cos(θ(s))ds
/// y(s) = ∫sin(θ(s))ds
/// θ(s) = θ₀ + κ_start * s + (κ_end - κ_start) * s² / (2L)
#[derive(Debug, Clone, Copy)]
pub struct SpiralGroundCurve {
    /// Linear tolerance for numerical computations.
    linear_tolerance: f64,
    /// Starting position (x, y).
    xy0: Vector2<f64>,
    /// Arc length of the spiral.
    arc_length: f64,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
    /// Curvature at the start of the spiral.
    curvature_start: f64,
    /// Curvature at the end of the spiral.
    curvature_end: f64,
    /// Heading angle at the start (radians).
    heading_start: f64,
}

impl SpiralGroundCurve {
    /// Number of integration steps for Fresnel integral approximation.
    const INTEGRATION_STEPS: usize = 100;

    /// Creates a new spiral ground curve.
    ///
    /// # Arguments
    /// * `linear_tolerance` - Linear tolerance for computations.
    /// * `xy0` - Starting position.
    /// * `heading_start` - Heading angle at the start (radians).
    /// * `curvature_start` - Curvature at the start.
    /// * `curvature_end` - Curvature at the end.
    /// * `arc_length` - Total arc length.
    /// * `p0` - Start parameter.
    /// * `p1` - End parameter.
    pub fn new(
        linear_tolerance: f64,
        xy0: Vector2<f64>,
        heading_start: f64,
        curvature_start: f64,
        curvature_end: f64,
        arc_length: f64,
        p0: f64,
        p1: f64,
    ) -> Self {
        Self {
            linear_tolerance,
            xy0,
            arc_length,
            p0,
            p1,
            curvature_start,
            curvature_end,
            heading_start,
        }
    }

    /// Creates from OpenDRIVE spiral parameters.
    ///
    /// OpenDRIVE specifies spirals with start position, heading, and curvature at both ends.
    pub fn from_xodr(
        linear_tolerance: f64,
        x: f64,
        y: f64,
        hdg: f64,
        length: f64,
        curv_start: f64,
        curv_end: f64,
        s: f64,
    ) -> Self {
        Self::new(
            linear_tolerance,
            Vector2::new(x, y),
            hdg,
            curv_start,
            curv_end,
            length,
            s,
            s + length,
        )
    }

    /// Returns the curvature rate (dκ/ds).
    pub fn curvature_rate(&self) -> f64 {
        if self.arc_length.abs() < f64::EPSILON {
            0.0
        } else {
            (self.curvature_end - self.curvature_start) / self.arc_length
        }
    }

    /// Returns the curvature at parameter p.
    pub fn curvature_at(&self, p: f64) -> f64 {
        let s = p - self.p0;
        self.curvature_start + self.curvature_rate() * s
    }

    /// Returns the heading at parameter p.
    pub fn heading_at(&self, p: f64) -> f64 {
        let s = p - self.p0;
        // θ(s) = θ₀ + κ_start * s + rate * s² / 2
        self.heading_start + self.curvature_start * s + self.curvature_rate() * s * s / 2.0
    }

    /// Computes position using numerical integration of the Fresnel integrals.
    ///
    /// This uses Simpson's rule for numerical integration.
    fn compute_position(&self, p: f64) -> Vector2<f64> {
        let s_target = p - self.p0;

        if s_target.abs() < f64::EPSILON {
            return self.xy0;
        }

        let n = Self::INTEGRATION_STEPS;
        let h = s_target / (n as f64);

        // Simpson's rule: ∫f(x)dx ≈ h/3 * (f(x₀) + 4f(x₁) + 2f(x₂) + 4f(x₃) + ... + f(xₙ))
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;

        for i in 0..=n {
            let s = h * (i as f64);
            let theta = self.heading_start + self.curvature_start * s + self.curvature_rate() * s * s / 2.0;

            let coeff = if i == 0 || i == n {
                1.0
            } else if i % 2 == 1 {
                4.0
            } else {
                2.0
            };

            sum_x += coeff * theta.cos();
            sum_y += coeff * theta.sin();
        }

        let integral_x = h / 3.0 * sum_x;
        let integral_y = h / 3.0 * sum_y;

        self.xy0 + Vector2::new(integral_x, integral_y)
    }

    /// Validates that p is within range.
    fn validate_p(&self, p: f64) -> MalidriveResult<()> {
        let tolerance = self.linear_tolerance;
        if p < self.p0 - tolerance || p > self.p1 + tolerance {
            return Err(MalidriveError::ParameterOutOfRange {
                parameter: "p".to_string(),
                value: p,
                min: self.p0,
                max: self.p1,
            });
        }
        Ok(())
    }
}

impl GroundCurve for SpiralGroundCurve {
    fn p_from_xodr_p(&self, xodr_p: f64) -> MalidriveResult<f64> {
        // For spiral, xodr_p is the same as internal p
        Ok(xodr_p)
    }

    fn g(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        self.validate_p(p)?;
        Ok(self.compute_position(p))
    }

    fn g_dot(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        self.validate_p(p)?;
        let theta = self.heading_at(p);
        Ok(Vector2::new(theta.cos(), theta.sin()))
    }

    fn heading(&self, p: f64) -> MalidriveResult<f64> {
        self.validate_p(p)?;
        Ok(self.heading_at(p))
    }

    fn heading_dot(&self, p: f64) -> MalidriveResult<f64> {
        self.validate_p(p)?;
        // dθ/ds = κ(s)
        Ok(self.curvature_at(p))
    }

    fn g_inverse(&self, xy: &Vector2<f64>) -> MalidriveResult<f64> {
        // Use iterative search to find p
        let tolerance = self.linear_tolerance;
        let num_samples = 100;
        let dp = (self.p1 - self.p0) / (num_samples as f64);

        let mut best_p = self.p0;
        let mut best_dist_sq = f64::MAX;

        // Sample the curve to find approximate location
        for i in 0..=num_samples {
            let p = self.p0 + dp * (i as f64);
            let pos = self.compute_position(p);
            let dist_sq = (pos - xy).norm_squared();
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_p = p;
            }
        }

        // Refine using Newton-Raphson
        let mut p = best_p;
        for _ in 0..20 {
            let pos = self.compute_position(p);
            let diff = xy - pos;
            let theta = self.heading_at(p);
            let tangent = Vector2::new(theta.cos(), theta.sin());

            // Project error onto tangent
            let dp = diff.dot(&tangent);
            if dp.abs() < tolerance {
                break;
            }
            p += dp.clamp(-self.arc_length / 10.0, self.arc_length / 10.0);
            p = p.clamp(self.p0, self.p1);
        }

        // Verify the result
        let final_pos = self.compute_position(p);
        let dist = (final_pos - xy).norm();
        if dist > tolerance * 10.0 {
            return Err(MalidriveError::GeometryError(format!(
                "Point ({}, {}) not on spiral curve (distance: {})",
                xy.x, xy.y, dist
            )));
        }

        Ok(p)
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

    const TOLERANCE: f64 = 1e-6;

    #[test]
    fn test_spiral_creation() {
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            0.0,
            0.1,
            100.0,
            0.0,
            100.0,
        );

        assert_relative_eq!(spiral.arc_length(), 100.0);
        assert_relative_eq!(spiral.p0(), 0.0);
        assert_relative_eq!(spiral.p1(), 100.0);
    }

    #[test]
    fn test_spiral_curvature_rate() {
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            0.0,
            0.1,
            100.0,
            0.0,
            100.0,
        );

        // Curvature rate = (0.1 - 0) / 100 = 0.001
        assert_relative_eq!(spiral.curvature_rate(), 0.001);

        // Curvature at start
        assert_relative_eq!(spiral.curvature_at(0.0), 0.0);

        // Curvature at middle
        assert_relative_eq!(spiral.curvature_at(50.0), 0.05);

        // Curvature at end
        assert_relative_eq!(spiral.curvature_at(100.0), 0.1);
    }

    #[test]
    fn test_spiral_heading() {
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            0.0,
            0.1,
            100.0,
            0.0,
            100.0,
        );

        // Heading at start
        assert_relative_eq!(spiral.heading(0.0).unwrap(), 0.0);

        // Heading at end: θ = 0 + 0*100 + 0.001*100²/2 = 5 radians
        assert_relative_eq!(spiral.heading(100.0).unwrap(), 5.0, epsilon = 1e-6);
    }

    #[test]
    fn test_spiral_start_position() {
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(10.0, 20.0),
            PI / 4.0,
            0.0,
            0.01,
            50.0,
            0.0,
            50.0,
        );

        let start = spiral.g(0.0).unwrap();
        assert_relative_eq!(start.x, 10.0, epsilon = 1e-9);
        assert_relative_eq!(start.y, 20.0, epsilon = 1e-9);
    }

    #[test]
    fn test_spiral_tangent_at_start() {
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            PI / 4.0, // 45 degrees
            0.0,
            0.1,
            100.0,
            0.0,
            100.0,
        );

        let g_dot = spiral.g_dot(0.0).unwrap();
        let expected_x = (PI / 4.0).cos();
        let expected_y = (PI / 4.0).sin();

        assert_relative_eq!(g_dot.x, expected_x, epsilon = 1e-9);
        assert_relative_eq!(g_dot.y, expected_y, epsilon = 1e-9);
    }

    #[test]
    fn test_spiral_unit_tangent() {
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            0.0,
            0.1,
            100.0,
            0.0,
            100.0,
        );

        // Tangent should be unit length at all points
        for s in [0.0, 25.0, 50.0, 75.0, 100.0] {
            let g_dot = spiral.g_dot(s).unwrap();
            assert_relative_eq!(g_dot.norm(), 1.0, epsilon = 1e-9);
        }
    }

    #[test]
    fn test_spiral_heading_dot() {
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            0.0,
            0.1,
            100.0,
            0.0,
            100.0,
        );

        // heading_dot = curvature
        assert_relative_eq!(spiral.heading_dot(0.0).unwrap(), 0.0);
        assert_relative_eq!(spiral.heading_dot(50.0).unwrap(), 0.05);
        assert_relative_eq!(spiral.heading_dot(100.0).unwrap(), 0.1);
    }

    #[test]
    fn test_straight_spiral() {
        // A spiral with zero curvature at both ends is a straight line
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0, // heading = 0 (east)
            0.0, // start curvature
            0.0, // end curvature
            100.0,
            0.0,
            100.0,
        );

        // At s=50, should be at (50, 0)
        let pos = spiral.g(50.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-6);

        // At s=100, should be at (100, 0)
        let pos = spiral.g(100.0).unwrap();
        assert_relative_eq!(pos.x, 100.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_spiral_from_xodr() {
        let spiral = SpiralGroundCurve::from_xodr(
            TOLERANCE,
            10.0, // x
            20.0, // y
            0.5,  // hdg
            50.0, // length
            0.0,  // curv_start
            0.02, // curv_end
            100.0, // s offset
        );

        assert_relative_eq!(spiral.p0(), 100.0);
        assert_relative_eq!(spiral.p1(), 150.0);
        assert_relative_eq!(spiral.arc_length(), 50.0);
    }

    #[test]
    fn test_spiral_parameter_out_of_range() {
        let spiral = SpiralGroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            0.0,
            0.1,
            100.0,
            0.0,
            100.0,
        );

        assert!(spiral.g(-1.0).is_err());
        assert!(spiral.g(101.0).is_err());
    }
}
