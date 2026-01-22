//! Parametric cubic polynomial (ParamPoly3) ground curve implementation.
//!
//! ParamPoly3 geometry is defined in OpenDRIVE and represents a curve
//! where both x and y are parametric cubic polynomials of a normalized
//! parameter t ∈ [0, 1].
//!
//! The curve is defined in the local coordinate system of the geometry element,
//! then rotated by the heading angle and translated to the start position.

use nalgebra::Vector2;

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::GroundCurve;

/// The parameter range for ParamPoly3 curves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamPoly3Range {
    /// Parameter range is [0, 1] (arcLength normalization).
    Normalized,
    /// Parameter range is [0, length] (explicit arcLength).
    ArcLength,
}

/// A parametric cubic polynomial ground curve.
///
/// The curve is defined by:
/// - u(t) = aU + bU*t + cU*t² + dU*t³  (local x)
/// - v(t) = aV + bV*t + cV*t² + dV*t³  (local y)
///
/// Where t is the parameter in range [0, p_range] (either 1 or arc_length).
///
/// The global position is computed by rotating (u,v) by the heading and
/// translating by the start position.
#[derive(Debug, Clone, Copy)]
pub struct ParamPoly3GroundCurve {
    /// Linear tolerance for numerical computations.
    linear_tolerance: f64,
    /// Starting position in global coordinates.
    xy0: Vector2<f64>,
    /// Heading angle at the start (radians).
    heading0: f64,
    /// Cosine of heading (cached).
    cos_heading: f64,
    /// Sine of heading (cached).
    sin_heading: f64,
    /// Arc length of the curve.
    arc_length: f64,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
    /// Polynomial coefficients for u (local x): a, b, c, d.
    u_coeffs: [f64; 4],
    /// Polynomial coefficients for v (local y): a, b, c, d.
    v_coeffs: [f64; 4],
    /// Parameter range type.
    param_range: ParamPoly3Range,
    /// Maximum parameter value (1.0 for normalized, arc_length for arc_length range).
    t_max: f64,
}

impl ParamPoly3GroundCurve {
    /// Creates a new ParamPoly3 ground curve.
    ///
    /// # Arguments
    /// * `linear_tolerance` - Linear tolerance for computations.
    /// * `xy0` - Starting position in global coordinates.
    /// * `heading0` - Heading angle at the start (radians).
    /// * `arc_length` - Arc length of the curve.
    /// * `p0` - Start parameter (usually the s-coordinate offset).
    /// * `p1` - End parameter (usually p0 + arc_length).
    /// * `u_coeffs` - Coefficients [aU, bU, cU, dU] for local x polynomial.
    /// * `v_coeffs` - Coefficients [aV, bV, cV, dV] for local y polynomial.
    /// * `param_range` - Whether parameters are normalized [0,1] or arc length [0,L].
    pub fn new(
        linear_tolerance: f64,
        xy0: Vector2<f64>,
        heading0: f64,
        arc_length: f64,
        p0: f64,
        p1: f64,
        u_coeffs: [f64; 4],
        v_coeffs: [f64; 4],
        param_range: ParamPoly3Range,
    ) -> Self {
        let t_max = match param_range {
            ParamPoly3Range::Normalized => 1.0,
            ParamPoly3Range::ArcLength => arc_length,
        };

        Self {
            linear_tolerance,
            xy0,
            heading0,
            cos_heading: heading0.cos(),
            sin_heading: heading0.sin(),
            arc_length,
            p0,
            p1,
            u_coeffs,
            v_coeffs,
            param_range,
            t_max,
        }
    }

    /// Creates from OpenDRIVE paramPoly3 element.
    pub fn from_xodr(
        linear_tolerance: f64,
        x: f64,
        y: f64,
        hdg: f64,
        length: f64,
        s: f64,
        a_u: f64,
        b_u: f64,
        c_u: f64,
        d_u: f64,
        a_v: f64,
        b_v: f64,
        c_v: f64,
        d_v: f64,
        p_range: Option<&str>,
    ) -> Self {
        let param_range = match p_range {
            Some("arcLength") => ParamPoly3Range::ArcLength,
            _ => ParamPoly3Range::Normalized,
        };

        Self::new(
            linear_tolerance,
            Vector2::new(x, y),
            hdg,
            length,
            s,
            s + length,
            [a_u, b_u, c_u, d_u],
            [a_v, b_v, c_v, d_v],
            param_range,
        )
    }

    /// Converts road parameter p to curve parameter t.
    fn p_to_t(&self, p: f64) -> f64 {
        let s = p - self.p0;
        match self.param_range {
            ParamPoly3Range::Normalized => {
                if self.arc_length.abs() < f64::EPSILON {
                    0.0
                } else {
                    s / self.arc_length
                }
            }
            ParamPoly3Range::ArcLength => s,
        }
    }

    /// Computes the scale factor dt/dp.
    fn dt_dp(&self) -> f64 {
        match self.param_range {
            ParamPoly3Range::Normalized => {
                if self.arc_length.abs() < f64::EPSILON {
                    0.0
                } else {
                    1.0 / self.arc_length
                }
            }
            ParamPoly3Range::ArcLength => 1.0,
        }
    }

    /// Evaluates u(t) = aU + bU*t + cU*t² + dU*t³.
    fn u(&self, t: f64) -> f64 {
        self.u_coeffs[0]
            + self.u_coeffs[1] * t
            + self.u_coeffs[2] * t * t
            + self.u_coeffs[3] * t * t * t
    }

    /// Evaluates v(t) = aV + bV*t + cV*t² + dV*t³.
    fn v(&self, t: f64) -> f64 {
        self.v_coeffs[0]
            + self.v_coeffs[1] * t
            + self.v_coeffs[2] * t * t
            + self.v_coeffs[3] * t * t * t
    }

    /// Evaluates du/dt = bU + 2*cU*t + 3*dU*t².
    fn u_dot(&self, t: f64) -> f64 {
        self.u_coeffs[1] + 2.0 * self.u_coeffs[2] * t + 3.0 * self.u_coeffs[3] * t * t
    }

    /// Evaluates dv/dt = bV + 2*cV*t + 3*dV*t².
    fn v_dot(&self, t: f64) -> f64 {
        self.v_coeffs[1] + 2.0 * self.v_coeffs[2] * t + 3.0 * self.v_coeffs[3] * t * t
    }

    /// Evaluates d²u/dt² = 2*cU + 6*dU*t.
    fn u_dot_dot(&self, t: f64) -> f64 {
        2.0 * self.u_coeffs[2] + 6.0 * self.u_coeffs[3] * t
    }

    /// Evaluates d²v/dt² = 2*cV + 6*dV*t.
    fn v_dot_dot(&self, t: f64) -> f64 {
        2.0 * self.v_coeffs[2] + 6.0 * self.v_coeffs[3] * t
    }

    /// Transforms local coordinates (u, v) to global coordinates (x, y).
    fn local_to_global(&self, u: f64, v: f64) -> Vector2<f64> {
        // Rotate and translate
        let x = self.xy0.x + u * self.cos_heading - v * self.sin_heading;
        let y = self.xy0.y + u * self.sin_heading + v * self.cos_heading;
        Vector2::new(x, y)
    }

    /// Transforms local velocity (du/dt, dv/dt) to global velocity (dx/dt, dy/dt).
    fn local_velocity_to_global(&self, u_dot: f64, v_dot: f64) -> Vector2<f64> {
        // Just rotate (translation doesn't affect velocity)
        let dx = u_dot * self.cos_heading - v_dot * self.sin_heading;
        let dy = u_dot * self.sin_heading + v_dot * self.cos_heading;
        Vector2::new(dx, dy)
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

impl GroundCurve for ParamPoly3GroundCurve {
    fn p_from_xodr_p(&self, xodr_p: f64) -> MalidriveResult<f64> {
        Ok(xodr_p)
    }

    fn g(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        self.validate_p(p)?;
        let t = self.p_to_t(p);
        let u = self.u(t);
        let v = self.v(t);
        Ok(self.local_to_global(u, v))
    }

    fn g_dot(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        self.validate_p(p)?;
        let t = self.p_to_t(p);
        let dt_dp = self.dt_dp();

        // Chain rule: dg/dp = dg/dt * dt/dp
        let u_dot = self.u_dot(t) * dt_dp;
        let v_dot = self.v_dot(t) * dt_dp;

        // Normalize to unit tangent
        let vel = self.local_velocity_to_global(u_dot, v_dot);
        let norm = vel.norm();
        if norm < f64::EPSILON {
            // Degenerate case: return heading direction
            Ok(Vector2::new(self.cos_heading, self.sin_heading))
        } else {
            Ok(vel / norm)
        }
    }

    fn heading(&self, p: f64) -> MalidriveResult<f64> {
        self.validate_p(p)?;
        let t = self.p_to_t(p);
        let u_dot = self.u_dot(t);
        let v_dot = self.v_dot(t);

        // Local heading
        let local_heading = v_dot.atan2(u_dot);

        // Add the initial heading to get global heading
        Ok(self.heading0 + local_heading)
    }

    fn heading_dot(&self, p: f64) -> MalidriveResult<f64> {
        self.validate_p(p)?;
        let t = self.p_to_t(p);
        let dt_dp = self.dt_dp();

        let u_d = self.u_dot(t);
        let v_d = self.v_dot(t);
        let u_dd = self.u_dot_dot(t);
        let v_dd = self.v_dot_dot(t);

        // Curvature formula: κ = (u'*v'' - v'*u'') / (u'² + v'²)^(3/2)
        let speed_sq = u_d * u_d + v_d * v_d;
        if speed_sq < f64::EPSILON {
            return Ok(0.0);
        }

        let curvature = (u_d * v_dd - v_d * u_dd) / speed_sq.powf(1.5);

        // heading_dot = curvature * ds/dp
        // But we need to account for the parametrization
        Ok(curvature * dt_dp * speed_sq.sqrt())
    }

    fn g_inverse(&self, xy: &Vector2<f64>) -> MalidriveResult<f64> {
        // Use iterative search
        let tolerance = self.linear_tolerance;
        let num_samples = 100;
        let dp = (self.p1 - self.p0) / (num_samples as f64);

        let mut best_p = self.p0;
        let mut best_dist_sq = f64::MAX;

        // Sample the curve
        for i in 0..=num_samples {
            let p = self.p0 + dp * (i as f64);
            let t = self.p_to_t(p);
            let pos = self.local_to_global(self.u(t), self.v(t));
            let dist_sq = (pos - xy).norm_squared();
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_p = p;
            }
        }

        // Refine using Newton-Raphson
        let mut p = best_p;
        for _ in 0..20 {
            let t = self.p_to_t(p);
            let pos = self.local_to_global(self.u(t), self.v(t));
            let diff = xy - pos;

            // Get tangent
            let dt_dp = self.dt_dp();
            let u_d = self.u_dot(t) * dt_dp;
            let v_d = self.v_dot(t) * dt_dp;
            let tangent = self.local_velocity_to_global(u_d, v_d);
            let tangent_len = tangent.norm();

            if tangent_len < f64::EPSILON {
                break;
            }

            // Project error onto tangent
            let dp_step = diff.dot(&tangent) / tangent_len;
            if dp_step.abs() < tolerance {
                break;
            }
            p += dp_step.clamp(-(self.p1 - self.p0) / 10.0, (self.p1 - self.p0) / 10.0);
            p = p.clamp(self.p0, self.p1);
        }

        // Verify the result
        let t = self.p_to_t(p);
        let final_pos = self.local_to_global(self.u(t), self.v(t));
        let dist = (final_pos - xy).norm();
        if dist > tolerance * 10.0 {
            return Err(MalidriveError::GeometryError(format!(
                "Point ({}, {}) not on ParamPoly3 curve (distance: {})",
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
    fn test_param_poly3_straight_line() {
        // u(t) = t, v(t) = 0 with normalized params [0,1] and length 100
        // This should be a straight line along the heading direction
        let curve = ParamPoly3GroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0, // heading east
            100.0,
            0.0,
            100.0,
            [0.0, 1.0, 0.0, 0.0], // u = t
            [0.0, 0.0, 0.0, 0.0], // v = 0
            ParamPoly3Range::Normalized,
        );

        // At p=0, t=0: (0, 0)
        let pos = curve.g(0.0).unwrap();
        assert_relative_eq!(pos.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-9);

        // At p=100, t=1: u=1, v=0 -> global (1, 0) rotated by heading
        let pos = curve.g(100.0).unwrap();
        assert_relative_eq!(pos.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_param_poly3_arc_length_range() {
        // u(t) = t, v(t) = 0 with arc length params [0, 100]
        let curve = ParamPoly3GroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            100.0,
            0.0,
            100.0,
            [0.0, 1.0, 0.0, 0.0], // u = t
            [0.0, 0.0, 0.0, 0.0], // v = 0
            ParamPoly3Range::ArcLength,
        );

        // At p=100, t=100: u=100, v=0
        let pos = curve.g(100.0).unwrap();
        assert_relative_eq!(pos.x, 100.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_param_poly3_with_offset() {
        let curve = ParamPoly3GroundCurve::new(
            TOLERANCE,
            Vector2::new(10.0, 20.0), // offset
            0.0,
            100.0,
            0.0,
            100.0,
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            ParamPoly3Range::Normalized,
        );

        let pos = curve.g(0.0).unwrap();
        assert_relative_eq!(pos.x, 10.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, 20.0, epsilon = 1e-9);
    }

    #[test]
    fn test_param_poly3_with_heading() {
        // Straight line with 90 degree heading (north)
        let curve = ParamPoly3GroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            PI / 2.0, // heading north
            100.0,
            0.0,
            100.0,
            [0.0, 1.0, 0.0, 0.0], // u = t
            [0.0, 0.0, 0.0, 0.0], // v = 0
            ParamPoly3Range::Normalized,
        );

        // At t=1, local coords are (1, 0), rotated by 90° gives (0, 1)
        let pos = curve.g(100.0).unwrap();
        assert_relative_eq!(pos.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_param_poly3_heading() {
        // Curve that bends: starts heading east, curves north
        let curve = ParamPoly3GroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            100.0,
            0.0,
            100.0,
            [0.0, 1.0, 0.0, 0.0],  // u = t (linear in u)
            [0.0, 0.0, 1.0, 0.0],  // v = t² (quadratic, curves upward)
            ParamPoly3Range::Normalized,
        );

        // At start: heading = atan2(dv/dt, du/dt) = atan2(0, 1) = 0
        let hdg = curve.heading(0.0).unwrap();
        assert_relative_eq!(hdg, 0.0, epsilon = 1e-6);

        // At end (t=1): dv/dt = 2t = 2, du/dt = 1, local_heading = atan2(2, 1)
        let hdg = curve.heading(100.0).unwrap();
        assert_relative_eq!(hdg, 2.0_f64.atan2(1.0), epsilon = 1e-6);
    }

    #[test]
    fn test_param_poly3_unit_tangent() {
        let curve = ParamPoly3GroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            100.0,
            0.0,
            100.0,
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            ParamPoly3Range::Normalized,
        );

        // g_dot should be a unit vector
        for s in [0.0, 25.0, 50.0, 75.0, 100.0] {
            let g_dot = curve.g_dot(s).unwrap();
            assert_relative_eq!(g_dot.norm(), 1.0, epsilon = 1e-9);
        }
    }

    #[test]
    fn test_param_poly3_from_xodr() {
        let curve = ParamPoly3GroundCurve::from_xodr(
            TOLERANCE,
            10.0,  // x
            20.0,  // y
            0.5,   // hdg
            50.0,  // length
            100.0, // s offset
            0.0, 1.0, 0.0, 0.0, // aU, bU, cU, dU
            0.0, 0.0, 0.0, 0.0, // aV, bV, cV, dV
            Some("normalized"),
        );

        assert_relative_eq!(curve.p0(), 100.0);
        assert_relative_eq!(curve.p1(), 150.0);
        assert_relative_eq!(curve.arc_length(), 50.0);
    }

    #[test]
    fn test_param_poly3_from_xodr_arc_length() {
        let curve = ParamPoly3GroundCurve::from_xodr(
            TOLERANCE,
            0.0, 0.0, 0.0,
            100.0,
            0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            Some("arcLength"),
        );

        assert_eq!(curve.param_range, ParamPoly3Range::ArcLength);
    }

    #[test]
    fn test_param_poly3_parameter_out_of_range() {
        let curve = ParamPoly3GroundCurve::new(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            100.0,
            0.0,
            100.0,
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            ParamPoly3Range::Normalized,
        );

        assert!(curve.g(-1.0).is_err());
        assert!(curve.g(101.0).is_err());
    }
}
