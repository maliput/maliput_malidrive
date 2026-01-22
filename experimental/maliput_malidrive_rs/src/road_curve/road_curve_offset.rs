//! Road curve offset computation.
//!
//! This module provides arc length computation for lanes that are laterally
//! offset from the road's reference line. The arc length of an offset curve
//! differs from the centerline because:
//! - Inner lanes (toward the center of curvature) are shorter
//! - Outer lanes (away from the center of curvature) are longer
//!
//! The key mappings provided are:
//! - `s(p)`: Arc length along the offset curve as a function of the reference parameter
//! - `p(s)`: Reference parameter as a function of arc length along the offset curve

use std::sync::Arc;

use crate::common::MalidriveResult;
use crate::road_curve::{Function, RoadCurve};

/// Minimum relative tolerance for integrator accuracy.
const MIN_RELATIVE_TOLERANCE: f64 = 1e-8;

/// Multiplier for converting relative tolerance to integrator accuracy.
const ACCURACY_MULTIPLIER: f64 = 1e-4;

/// Default number of samples for numerical integration.
const DEFAULT_NUM_SAMPLES: usize = 100;

/// Computes arc length integrals for a laterally offset road curve.
///
/// This struct provides mappings between the road parameter `p` and the
/// arc length `s` for a lane that is offset from the road's reference line.
///
/// # Mathematical Background
///
/// For a road curve W(p, r, h), the arc length along a curve at constant
/// (r, h) offset is:
///
/// ```text
/// s(p) = ∫[p0, p] |dW/dp| dp
/// ```
///
/// where `|dW/dp|` is the magnitude of the tangent vector at the offset position.
pub struct RoadCurveOffset {
    /// Reference to the road curve.
    road_curve: Arc<RoadCurve>,
    /// Lane offset function r(p).
    lane_offset: Arc<dyn Function>,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
    /// Total arc length of the offset curve.
    total_arc_length: f64,
    /// Relative tolerance for computations.
    relative_tolerance: f64,
    /// Precomputed samples for s(p) mapping.
    /// Each entry is (p, s) where s is the arc length at p.
    s_samples: Vec<(f64, f64)>,
}

impl RoadCurveOffset {
    /// Creates a new RoadCurveOffset.
    ///
    /// # Arguments
    /// * `road_curve` - The road curve to compute offsets for.
    /// * `lane_offset` - The function describing the lateral offset r(p).
    /// * `p0` - Start parameter.
    /// * `p1` - End parameter.
    /// * `integrator_accuracy_multiplier` - Multiplier to tune integration accuracy.
    ///
    /// # Errors
    /// Returns an error if p0 >= p1 or if integration fails.
    pub fn new(
        road_curve: Arc<RoadCurve>,
        lane_offset: Arc<dyn Function>,
        p0: f64,
        p1: f64,
        integrator_accuracy_multiplier: f64,
    ) -> MalidriveResult<Self> {
        assert!(p0 >= 0.0, "p0 must be non-negative");
        assert!(p0 <= p1, "p0 must be <= p1");
        assert!(
            integrator_accuracy_multiplier > 0.0,
            "integrator_accuracy_multiplier must be positive"
        );

        // Compute relative tolerance
        let relative_tolerance = (integrator_accuracy_multiplier
            * road_curve.linear_tolerance()
            / road_curve.l_max())
        .max(MIN_RELATIVE_TOLERANCE);

        // Build samples for s(p) mapping using numerical integration
        let (s_samples, total_arc_length) =
            Self::build_arc_length_samples(&road_curve, lane_offset.as_ref(), p0, p1)?;

        Ok(Self {
            road_curve,
            lane_offset,
            p0,
            p1,
            total_arc_length,
            relative_tolerance,
            s_samples,
        })
    }

    /// Returns the road curve.
    pub fn road_curve(&self) -> &RoadCurve {
        &self.road_curve
    }

    /// Returns the relative tolerance.
    pub fn relative_tolerance(&self) -> f64 {
        self.relative_tolerance
    }

    /// Returns the start parameter.
    pub fn p0(&self) -> f64 {
        self.p0
    }

    /// Returns the end parameter.
    pub fn p1(&self) -> f64 {
        self.p1
    }

    /// Returns the total arc length of the offset curve.
    pub fn total_arc_length(&self) -> f64 {
        self.total_arc_length
    }

    /// Computes the arc length s at parameter p.
    ///
    /// Uses linear interpolation between precomputed samples.
    pub fn calc_s_from_p(&self, p: f64) -> f64 {
        if p <= self.p0 {
            return 0.0;
        }
        if p >= self.p1 {
            return self.total_arc_length;
        }

        // Binary search to find the interval
        let idx = self
            .s_samples
            .binary_search_by(|(pi, _)| pi.partial_cmp(&p).unwrap())
            .unwrap_or_else(|i| i.saturating_sub(1));

        if idx >= self.s_samples.len() - 1 {
            return self.total_arc_length;
        }

        // Linear interpolation
        let (p0, s0) = self.s_samples[idx];
        let (p1, s1) = self.s_samples[idx + 1];

        if (p1 - p0).abs() < 1e-15 {
            return s0;
        }

        let t = (p - p0) / (p1 - p0);
        s0 + t * (s1 - s0)
    }

    /// Computes the parameter p at arc length s.
    ///
    /// Uses linear interpolation between precomputed samples.
    pub fn calc_p_from_s(&self, s: f64) -> f64 {
        if s <= 0.0 {
            return self.p0;
        }
        if s >= self.total_arc_length {
            return self.p1;
        }

        // Binary search to find the interval by s value
        let idx = self
            .s_samples
            .binary_search_by(|(_, si)| si.partial_cmp(&s).unwrap())
            .unwrap_or_else(|i| i.saturating_sub(1));

        if idx >= self.s_samples.len() - 1 {
            return self.p1;
        }

        // Linear interpolation
        let (p0, s0) = self.s_samples[idx];
        let (p1, s1) = self.s_samples[idx + 1];

        if (s1 - s0).abs() < 1e-15 {
            return p0;
        }

        let t = (s - s0) / (s1 - s0);
        p0 + t * (p1 - p0)
    }

    /// Returns a closure that computes s from p.
    pub fn s_from_p(&self) -> impl Fn(f64) -> f64 + '_ {
        move |p| self.calc_s_from_p(p)
    }

    /// Returns a closure that computes p from s.
    pub fn p_from_s(&self) -> impl Fn(f64) -> f64 + '_ {
        move |s| self.calc_p_from_s(s)
    }

    /// Builds the arc length samples using numerical integration (trapezoidal rule).
    fn build_arc_length_samples(
        road_curve: &RoadCurve,
        lane_offset: &dyn Function,
        p0: f64,
        p1: f64,
    ) -> MalidriveResult<(Vec<(f64, f64)>, f64)> {
        let num_samples = DEFAULT_NUM_SAMPLES.max(
            ((p1 - p0) / road_curve.scale_length() * 10.0).ceil() as usize + 1,
        );

        let dp = (p1 - p0) / (num_samples - 1) as f64;
        let mut samples = Vec::with_capacity(num_samples);
        let mut s = 0.0;

        for i in 0..num_samples {
            let p = p0 + i as f64 * dp;
            let p_clamped = p.min(p1).max(p0);

            samples.push((p_clamped, s));

            if i < num_samples - 1 {
                // Compute arc length increment using Simpson's rule for better accuracy
                let p_mid = p_clamped + dp / 2.0;
                let p_next = (p_clamped + dp).min(p1);

                let ds_dp_start = Self::arc_length_derivative(road_curve, lane_offset, p_clamped)?;
                let ds_dp_mid = Self::arc_length_derivative(road_curve, lane_offset, p_mid)?;
                let ds_dp_end = Self::arc_length_derivative(road_curve, lane_offset, p_next)?;

                // Simpson's rule: ∫f(x)dx ≈ (h/6) * (f(a) + 4*f(mid) + f(b))
                let ds = (dp / 6.0) * (ds_dp_start + 4.0 * ds_dp_mid + ds_dp_end);
                s += ds;
            }
        }

        // Ensure the last sample has the correct total arc length
        if let Some(last) = samples.last_mut() {
            last.1 = s;
        }

        Ok((samples, s))
    }

    /// Computes the arc length derivative ds/dp at parameter p.
    ///
    /// This is |dW/dp| at the offset position (p, r(p), 0).
    fn arc_length_derivative(
        road_curve: &RoadCurve,
        lane_offset: &dyn Function,
        p: f64,
    ) -> MalidriveResult<f64> {
        let r = lane_offset.f(p)?;
        let h = 0.0; // Arc length is computed at road surface level

        // Get the tangent vector magnitude at the offset position
        let w_dot = road_curve.w_dot_with_lane_offset(p, r, h, lane_offset)?;

        Ok(w_dot.norm())
    }
}

impl std::fmt::Debug for RoadCurveOffset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RoadCurveOffset")
            .field("p0", &self.p0)
            .field("p1", &self.p1)
            .field("total_arc_length", &self.total_arc_length)
            .field("relative_tolerance", &self.relative_tolerance)
            .field("num_samples", &self.s_samples.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::road_curve::{ArcGroundCurve, ConstantFunction, CubicPolynomial, LineGroundCurve};
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    // Match C++ test constants
    const K_LINEAR_TOLERANCE: f64 = 1e-12;
    const K_SCALE_LENGTH: f64 = 1.0;
    const K_P0: f64 = 10.0;
    const K_P1: f64 = 20.0;
    const K_XY0: (f64, f64) = (1.0, 2.0);
    const K_DXY: (f64, f64) = (3.0, 4.0); // norm = 5
    const K_R0: f64 = 0.0;
    const K_R_LEFT: f64 = 4.0;
    const K_R_RIGHT: f64 = -2.5;
    
    // Arc road curve constants
    const K_CURVATURE: f64 = -0.025; // Equivalent radius = 40m (right turn)
    const K_ARC_LENGTH: f64 = 100.0;

    fn make_constant_cubic_polynomial(value: f64, p0: f64, p1: f64) -> Arc<CubicPolynomial> {
        // In C++, CubicPolynomial(a,b,c,d,p0,p1) means f(p) = a*p³ + b*p² + c*p + d
        // So a constant function uses a=b=c=0, d=value
        // In Rust, CubicPolynomial::new(a,b,c,d,p0,p1) means f(p) = a + b*(p-p0) + c*(p-p0)² + d*(p-p0)³
        // So a constant function uses a=value, b=c=d=0
        Arc::new(CubicPolynomial::constant(value, p0, p1))
    }

    /// Creates a flat line road curve matching C++ FlatLineRoadCurveTest
    fn make_flat_line_road_curve() -> Arc<RoadCurve> {
        let ground_curve = Arc::new(
            LineGroundCurve::new(
                K_LINEAR_TOLERANCE,
                nalgebra::Vector2::new(K_XY0.0, K_XY0.1),
                nalgebra::Vector2::new(K_DXY.0, K_DXY.1),
                K_P0,
                K_P1,
            )
            .unwrap(),
        );
        let elevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, K_P0, K_P1));
        let superelevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, K_P0, K_P1));

        Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            K_LINEAR_TOLERANCE,
            K_SCALE_LENGTH,
        ))
    }

    /// Creates a flat arc road curve matching C++ FlatArcRoadCurveTest
    fn make_flat_arc_road_curve() -> Arc<RoadCurve> {
        let start_heading = PI / 3.0;

        let ground_curve = Arc::new(
            ArcGroundCurve::new(
                K_LINEAR_TOLERANCE,
                nalgebra::Vector2::new(K_XY0.0, K_XY0.1),
                start_heading,
                K_CURVATURE,
                K_ARC_LENGTH,
                K_P0,
                K_P1,
            )
            .unwrap(),
        );
        let elevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, K_P0, K_P1));
        let superelevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, K_P0, K_P1));

        Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            K_LINEAR_TOLERANCE,
            K_SCALE_LENGTH,
        ))
    }

    // Also keep the simple test helpers for basic tests
    const TOLERANCE: f64 = 1e-6;

    fn make_straight_road() -> Arc<RoadCurve> {
        let ground_curve = Arc::new(
            LineGroundCurve::from_heading(
                TOLERANCE,
                nalgebra::Vector2::new(0.0, 0.0),
                0.0,   // heading east
                100.0, // length
                0.0,   // p0
                100.0, // p1
            )
            .unwrap(),
        );
        let elevation = Arc::new(ConstantFunction::zero(0.0, 100.0));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, 100.0));

        Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            TOLERANCE,
            1.0,
        ))
    }

    fn make_arc_road() -> Arc<RoadCurve> {
        // 90-degree turn with radius 100m
        let arc_length = PI * 100.0 / 2.0; // quarter circle
        let ground_curve = Arc::new(
            ArcGroundCurve::new(
                TOLERANCE,
                nalgebra::Vector2::new(0.0, 0.0),
                0.0,        // heading east
                0.01,       // curvature (1/100m radius)
                arc_length, // arc_length
                0.0,        // p0
                arc_length, // p1
            )
            .unwrap(),
        );
        let elevation = Arc::new(ConstantFunction::zero(0.0, arc_length));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, arc_length));

        Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            TOLERANCE,
            1.0,
        ))
    }

    // =========================================================================
    // Tests matching C++ FlatLineRoadCurveTest
    // =========================================================================

    #[test]
    fn test_flat_line_road_curve() {
        let road_curve = make_flat_line_road_curve();
        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);

        let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset_0, K_P0, K_P1, 1.0).unwrap();

        // Verify road_curve accessor
        assert!(std::ptr::eq(dut.road_curve(), road_curve.as_ref()));
    }

    #[test]
    fn test_flat_line_relative_tolerance() {
        // Use larger tolerance to test non-clamped case
        let k_tolerance = 1e-3;
        let ground_curve = Arc::new(
            LineGroundCurve::new(
                k_tolerance,
                nalgebra::Vector2::new(K_XY0.0, K_XY0.1),
                nalgebra::Vector2::new(K_DXY.0, K_DXY.1),
                K_P0,
                K_P1,
            )
            .unwrap(),
        );
        let elevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, K_P0, K_P1));
        let superelevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, K_P0, K_P1));
        let road_curve = Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            k_tolerance,
            K_SCALE_LENGTH,
        ));

        let lane_offset = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let dut = RoadCurveOffset::new(road_curve, lane_offset, K_P0, K_P1, 1.0).unwrap();

        // C++ expects: kTolerance / kDXy.norm() = 1e-3 / 5.0 = 2e-4
        let dxy_norm = (K_DXY.0 * K_DXY.0 + K_DXY.1 * K_DXY.1).sqrt();
        assert_relative_eq!(dut.relative_tolerance(), k_tolerance / dxy_norm, epsilon = 1e-10);
    }

    #[test]
    fn test_flat_line_relative_tolerance_clamped() {
        let road_curve = make_flat_line_road_curve();
        let lane_offset = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);

        let dut = RoadCurveOffset::new(road_curve, lane_offset, K_P0, K_P1, 1.0).unwrap();

        // Should be clamped to minimum value 1e-8
        assert_relative_eq!(dut.relative_tolerance(), MIN_RELATIVE_TOLERANCE, epsilon = 1e-15);
    }

    #[test]
    fn test_flat_line_calc_s_from_p() {
        let road_curve = make_flat_line_road_curve();
        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        let delta_p = K_P1 - K_P0;
        let arc_length = (K_DXY.0 * K_DXY.0 + K_DXY.1 * K_DXY.1).sqrt(); // 5.0

        let ps = [
            K_P0,
            K_P0 + delta_p * 0.25,
            K_P0 + delta_p * 0.5,
            K_P0 + delta_p * 0.75,
            K_P1,
        ];
        let expected_ss = [
            0.0,
            0.25 * arc_length,
            0.5 * arc_length,
            0.75 * arc_length,
            arc_length,
        ];

        // For a flat line, all offsets should give the same arc length
        for lane_offset in [lane_offset_0, lane_offset_left, lane_offset_right] {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, K_P0, K_P1, 1.0).unwrap();
            for (p, expected_s) in ps.iter().zip(expected_ss.iter()) {
                assert_relative_eq!(dut.calc_s_from_p(*p), *expected_s, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    #[test]
    fn test_flat_line_p_from_s() {
        let road_curve = make_flat_line_road_curve();
        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        let delta_p = K_P1 - K_P0;
        let arc_length = (K_DXY.0 * K_DXY.0 + K_DXY.1 * K_DXY.1).sqrt(); // 5.0

        let expected_ps = [
            K_P0,
            K_P0 + delta_p * 0.25,
            K_P0 + delta_p * 0.5,
            K_P0 + delta_p * 0.75,
            K_P1,
        ];
        let ss = [
            0.0,
            0.25 * arc_length,
            0.5 * arc_length,
            0.75 * arc_length,
            arc_length,
        ];

        for lane_offset in [lane_offset_0, lane_offset_left, lane_offset_right] {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, K_P0, K_P1, 1.0).unwrap();
            let p_from_s = dut.p_from_s();
            for (s, expected_p) in ss.iter().zip(expected_ps.iter()) {
                assert_relative_eq!(p_from_s(*s), *expected_p, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    #[test]
    fn test_flat_line_s_from_p_closure() {
        let road_curve = make_flat_line_road_curve();
        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        let delta_p = K_P1 - K_P0;
        let arc_length = (K_DXY.0 * K_DXY.0 + K_DXY.1 * K_DXY.1).sqrt();

        let ps = [
            K_P0,
            K_P0 + delta_p * 0.25,
            K_P0 + delta_p * 0.5,
            K_P0 + delta_p * 0.75,
            K_P1,
        ];
        let expected_ss = [
            0.0,
            0.25 * arc_length,
            0.5 * arc_length,
            0.75 * arc_length,
            arc_length,
        ];

        for lane_offset in [lane_offset_0, lane_offset_left, lane_offset_right] {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, K_P0, K_P1, 1.0).unwrap();
            let s_from_p = dut.s_from_p();
            for (p, expected_s) in ps.iter().zip(expected_ss.iter()) {
                assert_relative_eq!(s_from_p(*p), *expected_s, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    // =========================================================================
    // Tests matching C++ FlatLineRoadCurveSubRangeTest
    // =========================================================================

    #[test]
    fn test_flat_line_sub_range_calc_s_from_p() {
        let road_curve = make_flat_line_road_curve();

        let k_p0_sub = 12.0;
        let k_p1_sub = 17.0;
        let delta_p_sub = k_p1_sub - k_p0_sub;
        let arc_length = (K_DXY.0 * K_DXY.0 + K_DXY.1 * K_DXY.1).sqrt(); // 5.0

        let ps_sub = [
            k_p0_sub,
            k_p0_sub + delta_p_sub * 0.25,
            k_p0_sub + delta_p_sub * 0.5,
            k_p0_sub + delta_p_sub * 0.75,
            k_p1_sub,
        ];
        // Sub-range is half the full range, so arc lengths are halved
        let ss_sub = [
            0.0,
            0.25 * arc_length / 2.0,
            0.5 * arc_length / 2.0,
            0.75 * arc_length / 2.0,
            arc_length / 2.0,
        ];

        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        for lane_offset in [lane_offset_0, lane_offset_left, lane_offset_right] {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, k_p0_sub, k_p1_sub, 1.0).unwrap();
            for (p, expected_s) in ps_sub.iter().zip(ss_sub.iter()) {
                assert_relative_eq!(dut.calc_s_from_p(*p), *expected_s, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    #[test]
    fn test_flat_line_sub_range_p_from_s() {
        let road_curve = make_flat_line_road_curve();

        let k_p0_sub = 12.0;
        let k_p1_sub = 17.0;
        let delta_p_sub = k_p1_sub - k_p0_sub;
        let arc_length = (K_DXY.0 * K_DXY.0 + K_DXY.1 * K_DXY.1).sqrt();

        let expected_ps_sub = [
            k_p0_sub,
            k_p0_sub + delta_p_sub * 0.25,
            k_p0_sub + delta_p_sub * 0.5,
            k_p0_sub + delta_p_sub * 0.75,
            k_p1_sub,
        ];
        let ss_sub = [
            0.0,
            0.25 * arc_length / 2.0,
            0.5 * arc_length / 2.0,
            0.75 * arc_length / 2.0,
            arc_length / 2.0,
        ];

        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        for lane_offset in [lane_offset_0, lane_offset_left, lane_offset_right] {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, k_p0_sub, k_p1_sub, 1.0).unwrap();
            let p_from_s = dut.p_from_s();
            for (s, expected_p) in ss_sub.iter().zip(expected_ps_sub.iter()) {
                assert_relative_eq!(p_from_s(*s), *expected_p, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    // =========================================================================
    // Tests matching C++ FlatArcRoadCurveTest
    // =========================================================================

    #[test]
    fn test_flat_arc_road_curve() {
        let road_curve = make_flat_arc_road_curve();
        let lane_offset = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);

        let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, K_P0, K_P1, 1.0).unwrap();

        assert!(std::ptr::eq(dut.road_curve(), road_curve.as_ref()));
    }

    #[test]
    fn test_flat_arc_relative_tolerance() {
        let k_tolerance = 1e-3;
        let start_heading = PI / 3.0;

        let ground_curve = Arc::new(
            ArcGroundCurve::new(
                k_tolerance,
                nalgebra::Vector2::new(K_XY0.0, K_XY0.1),
                start_heading,
                K_CURVATURE,
                K_ARC_LENGTH,
                K_P0,
                K_P1,
            )
            .unwrap(),
        );
        let elevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, K_P0, K_P1));
        let superelevation = Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, K_P0, K_P1));
        let road_curve = Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            k_tolerance,
            K_SCALE_LENGTH,
        ));

        let lane_offset = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let dut = RoadCurveOffset::new(road_curve, lane_offset, K_P0, K_P1, 1.0).unwrap();

        // C++ expects: kTolerance / kArcLength = 1e-3 / 100.0 = 1e-5
        assert_relative_eq!(dut.relative_tolerance(), k_tolerance / K_ARC_LENGTH, epsilon = 1e-10);
    }

    #[test]
    fn test_flat_arc_relative_tolerance_clamped() {
        let road_curve = make_flat_arc_road_curve();
        let lane_offset = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);

        let dut = RoadCurveOffset::new(road_curve, lane_offset, K_P0, K_P1, 1.0).unwrap();

        assert_relative_eq!(dut.relative_tolerance(), MIN_RELATIVE_TOLERANCE, epsilon = 1e-15);
    }

    #[test]
    fn test_flat_arc_calc_s_from_p() {
        let road_curve = make_flat_arc_road_curve();

        let radius_r0 = (1.0 / K_CURVATURE).abs(); // 40m
        let radius_r_left = (1.0 / K_CURVATURE).abs() + K_R_LEFT; // 44m
        let radius_r_right = (1.0 / K_CURVATURE).abs() + K_R_RIGHT; // 37.5m
        let d_theta = K_ARC_LENGTH / radius_r0;

        let delta_p = K_P1 - K_P0;
        let ps = [
            K_P0,
            K_P0 + delta_p * 0.25,
            K_P0 + delta_p * 0.5,
            K_P0 + delta_p * 0.75,
            K_P1,
        ];

        // Expected arc lengths for each offset
        let ss_r0 = [
            0.0,
            0.25 * K_ARC_LENGTH,
            0.5 * K_ARC_LENGTH,
            0.75 * K_ARC_LENGTH,
            K_ARC_LENGTH,
        ];
        let ss_left = [
            0.0,
            0.25 * d_theta * radius_r_left,
            0.5 * d_theta * radius_r_left,
            0.75 * d_theta * radius_r_left,
            d_theta * radius_r_left,
        ];
        let ss_right = [
            0.0,
            0.25 * d_theta * radius_r_right,
            0.5 * d_theta * radius_r_right,
            0.75 * d_theta * radius_r_right,
            d_theta * radius_r_right,
        ];

        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        let offsets_and_expected: [(Arc<CubicPolynomial>, &[f64; 5]); 3] = [
            (lane_offset_0, &ss_r0),
            (lane_offset_left, &ss_left),
            (lane_offset_right, &ss_right),
        ];

        for (lane_offset, expected_ss) in offsets_and_expected {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, K_P0, K_P1, 1.0).unwrap();
            for (p, expected_s) in ps.iter().zip(expected_ss.iter()) {
                assert_relative_eq!(dut.calc_s_from_p(*p), *expected_s, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    #[test]
    fn test_flat_arc_p_from_s() {
        let road_curve = make_flat_arc_road_curve();

        let radius_r0 = (1.0 / K_CURVATURE).abs();
        let radius_r_left = (1.0 / K_CURVATURE).abs() + K_R_LEFT;
        let radius_r_right = (1.0 / K_CURVATURE).abs() + K_R_RIGHT;
        let d_theta = K_ARC_LENGTH / radius_r0;

        let delta_p = K_P1 - K_P0;
        let expected_ps = [
            K_P0,
            K_P0 + delta_p * 0.25,
            K_P0 + delta_p * 0.5,
            K_P0 + delta_p * 0.75,
            K_P1,
        ];

        let ss_r0 = [
            0.0,
            0.25 * K_ARC_LENGTH,
            0.5 * K_ARC_LENGTH,
            0.75 * K_ARC_LENGTH,
            K_ARC_LENGTH,
        ];
        let ss_left = [
            0.0,
            0.25 * d_theta * radius_r_left,
            0.5 * d_theta * radius_r_left,
            0.75 * d_theta * radius_r_left,
            d_theta * radius_r_left,
        ];
        let ss_right = [
            0.0,
            0.25 * d_theta * radius_r_right,
            0.5 * d_theta * radius_r_right,
            0.75 * d_theta * radius_r_right,
            d_theta * radius_r_right,
        ];

        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        let offsets_and_ss: [(Arc<CubicPolynomial>, &[f64; 5]); 3] = [
            (lane_offset_0, &ss_r0),
            (lane_offset_left, &ss_left),
            (lane_offset_right, &ss_right),
        ];

        for (lane_offset, ss) in offsets_and_ss {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, K_P0, K_P1, 1.0).unwrap();
            let p_from_s = dut.p_from_s();
            for (s, expected_p) in ss.iter().zip(expected_ps.iter()) {
                assert_relative_eq!(p_from_s(*s), *expected_p, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    #[test]
    fn test_flat_arc_s_from_p_closure() {
        let road_curve = make_flat_arc_road_curve();

        let radius_r0 = (1.0 / K_CURVATURE).abs();
        let radius_r_left = (1.0 / K_CURVATURE).abs() + K_R_LEFT;
        let radius_r_right = (1.0 / K_CURVATURE).abs() + K_R_RIGHT;
        let d_theta = K_ARC_LENGTH / radius_r0;

        let delta_p = K_P1 - K_P0;
        let ps = [
            K_P0,
            K_P0 + delta_p * 0.25,
            K_P0 + delta_p * 0.5,
            K_P0 + delta_p * 0.75,
            K_P1,
        ];

        let ss_r0 = [
            0.0,
            0.25 * K_ARC_LENGTH,
            0.5 * K_ARC_LENGTH,
            0.75 * K_ARC_LENGTH,
            K_ARC_LENGTH,
        ];
        let ss_left = [
            0.0,
            0.25 * d_theta * radius_r_left,
            0.5 * d_theta * radius_r_left,
            0.75 * d_theta * radius_r_left,
            d_theta * radius_r_left,
        ];
        let ss_right = [
            0.0,
            0.25 * d_theta * radius_r_right,
            0.5 * d_theta * radius_r_right,
            0.75 * d_theta * radius_r_right,
            d_theta * radius_r_right,
        ];

        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        let offsets_and_expected: [(Arc<CubicPolynomial>, &[f64; 5]); 3] = [
            (lane_offset_0, &ss_r0),
            (lane_offset_left, &ss_left),
            (lane_offset_right, &ss_right),
        ];

        for (lane_offset, expected_ss) in offsets_and_expected {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, K_P0, K_P1, 1.0).unwrap();
            let s_from_p = dut.s_from_p();
            for (p, expected_s) in ps.iter().zip(expected_ss.iter()) {
                assert_relative_eq!(s_from_p(*p), *expected_s, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    // =========================================================================
    // Tests matching C++ FlatArcRoadCurveSubRangeTest
    // =========================================================================

    #[test]
    fn test_flat_arc_sub_range_calc_s_from_p() {
        let road_curve = make_flat_arc_road_curve();

        let k_p0_sub = 12.0;
        let k_p1_sub = 17.0;
        let delta_p_sub = k_p1_sub - k_p0_sub;

        let radius_r0 = (1.0 / K_CURVATURE).abs();
        let radius_r_left = (1.0 / K_CURVATURE).abs() + K_R_LEFT;
        let radius_r_right = (1.0 / K_CURVATURE).abs() + K_R_RIGHT;
        let d_theta = K_ARC_LENGTH / radius_r0;

        let ps_sub = [
            k_p0_sub,
            k_p0_sub + delta_p_sub * 0.25,
            k_p0_sub + delta_p_sub * 0.5,
            k_p0_sub + delta_p_sub * 0.75,
            k_p1_sub,
        ];

        // Sub-range is half the full range
        let ss_sub_r0 = [
            0.0,
            0.25 * K_ARC_LENGTH / 2.0,
            0.5 * K_ARC_LENGTH / 2.0,
            0.75 * K_ARC_LENGTH / 2.0,
            K_ARC_LENGTH / 2.0,
        ];
        let ss_sub_left = [
            0.0,
            0.25 * d_theta * radius_r_left / 2.0,
            0.5 * d_theta * radius_r_left / 2.0,
            0.75 * d_theta * radius_r_left / 2.0,
            d_theta * radius_r_left / 2.0,
        ];
        let ss_sub_right = [
            0.0,
            0.25 * d_theta * radius_r_right / 2.0,
            0.5 * d_theta * radius_r_right / 2.0,
            0.75 * d_theta * radius_r_right / 2.0,
            d_theta * radius_r_right / 2.0,
        ];

        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        let offsets_and_expected: [(Arc<CubicPolynomial>, &[f64; 5]); 3] = [
            (lane_offset_0, &ss_sub_r0),
            (lane_offset_left, &ss_sub_left),
            (lane_offset_right, &ss_sub_right),
        ];

        for (lane_offset, expected_ss) in offsets_and_expected {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, k_p0_sub, k_p1_sub, 1.0).unwrap();
            for (p, expected_s) in ps_sub.iter().zip(expected_ss.iter()) {
                assert_relative_eq!(dut.calc_s_from_p(*p), *expected_s, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    #[test]
    fn test_flat_arc_sub_range_p_from_s() {
        let road_curve = make_flat_arc_road_curve();

        let k_p0_sub = 12.0;
        let k_p1_sub = 17.0;
        let delta_p_sub = k_p1_sub - k_p0_sub;

        let radius_r0 = (1.0 / K_CURVATURE).abs();
        let radius_r_left = (1.0 / K_CURVATURE).abs() + K_R_LEFT;
        let radius_r_right = (1.0 / K_CURVATURE).abs() + K_R_RIGHT;
        let d_theta = K_ARC_LENGTH / radius_r0;

        let expected_ps_sub = [
            k_p0_sub,
            k_p0_sub + delta_p_sub * 0.25,
            k_p0_sub + delta_p_sub * 0.5,
            k_p0_sub + delta_p_sub * 0.75,
            k_p1_sub,
        ];

        let ss_sub_r0 = [
            0.0,
            0.25 * K_ARC_LENGTH / 2.0,
            0.5 * K_ARC_LENGTH / 2.0,
            0.75 * K_ARC_LENGTH / 2.0,
            K_ARC_LENGTH / 2.0,
        ];
        let ss_sub_left = [
            0.0,
            0.25 * d_theta * radius_r_left / 2.0,
            0.5 * d_theta * radius_r_left / 2.0,
            0.75 * d_theta * radius_r_left / 2.0,
            d_theta * radius_r_left / 2.0,
        ];
        let ss_sub_right = [
            0.0,
            0.25 * d_theta * radius_r_right / 2.0,
            0.5 * d_theta * radius_r_right / 2.0,
            0.75 * d_theta * radius_r_right / 2.0,
            d_theta * radius_r_right / 2.0,
        ];

        let lane_offset_0 = make_constant_cubic_polynomial(K_R0, K_P0, K_P1);
        let lane_offset_left = make_constant_cubic_polynomial(K_R_LEFT, K_P0, K_P1);
        let lane_offset_right = make_constant_cubic_polynomial(K_R_RIGHT, K_P0, K_P1);

        let offsets_and_ss: [(Arc<CubicPolynomial>, &[f64; 5]); 3] = [
            (lane_offset_0, &ss_sub_r0),
            (lane_offset_left, &ss_sub_left),
            (lane_offset_right, &ss_sub_right),
        ];

        for (lane_offset, ss) in offsets_and_ss {
            let dut = RoadCurveOffset::new(road_curve.clone(), lane_offset, k_p0_sub, k_p1_sub, 1.0).unwrap();
            let p_from_s = dut.p_from_s();
            for (s, expected_p) in ss.iter().zip(expected_ps_sub.iter()) {
                assert_relative_eq!(p_from_s(*s), *expected_p, epsilon = K_LINEAR_TOLERANCE);
            }
        }
    }

    // =========================================================================
    // Original basic tests (kept for backward compatibility)
    // =========================================================================

    #[test]
    fn test_road_curve_offset_creation() {
        let road_curve = make_straight_road();
        let lane_offset = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let offset = RoadCurveOffset::new(road_curve, lane_offset, 0.0, 100.0, 1.0).unwrap();

        assert_relative_eq!(offset.p0(), 0.0);
        assert_relative_eq!(offset.p1(), 100.0);
        // For a straight road with zero offset, arc length equals parameter range
        assert_relative_eq!(offset.total_arc_length(), 100.0, epsilon = 0.1);
    }

    #[test]
    fn test_straight_road_zero_offset() {
        let road_curve = make_straight_road();
        let lane_offset = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let offset = RoadCurveOffset::new(road_curve, lane_offset, 0.0, 100.0, 1.0).unwrap();

        // s(p) should equal p for straight road with zero offset
        assert_relative_eq!(offset.calc_s_from_p(0.0), 0.0, epsilon = 0.01);
        assert_relative_eq!(offset.calc_s_from_p(50.0), 50.0, epsilon = 0.5);
        assert_relative_eq!(offset.calc_s_from_p(100.0), 100.0, epsilon = 0.1);

        // p(s) should equal s for straight road with zero offset
        assert_relative_eq!(offset.calc_p_from_s(0.0), 0.0, epsilon = 0.01);
        assert_relative_eq!(offset.calc_p_from_s(50.0), 50.0, epsilon = 0.5);
        assert_relative_eq!(offset.calc_p_from_s(100.0), 100.0, epsilon = 0.1);
    }

    #[test]
    fn test_straight_road_with_constant_offset() {
        let road_curve = make_straight_road();
        // Lane is 10m to the left of the reference line
        let lane_offset = Arc::new(ConstantFunction::new(10.0, 0.0, 100.0));

        let offset = RoadCurveOffset::new(road_curve, lane_offset, 0.0, 100.0, 1.0).unwrap();

        // For a straight road, even with offset, arc length should equal parameter range
        // because the offset doesn't change the path length
        assert_relative_eq!(offset.total_arc_length(), 100.0, epsilon = 0.5);
    }

    #[test]
    fn test_arc_road_zero_offset() {
        let road_curve = make_arc_road();
        let arc_length = PI * 100.0 / 2.0;
        let lane_offset = Arc::new(ConstantFunction::zero(0.0, arc_length));

        let offset =
            RoadCurveOffset::new(road_curve, lane_offset, 0.0, arc_length, 1.0).unwrap();

        // For zero offset on centerline, arc length equals the ground curve arc length
        assert_relative_eq!(offset.total_arc_length(), arc_length, epsilon = 1.0);
    }

    #[test]
    fn test_arc_road_inner_lane() {
        let road_curve = make_arc_road();
        let arc_length = PI * 100.0 / 2.0;
        // For a left turn (positive curvature), r > 0 is toward the center (inner lane)
        // Lane is 10m to the left = toward the center of the left turn = inner lane (radius = 90m)
        let lane_offset = Arc::new(ConstantFunction::new(10.0, 0.0, arc_length));

        let offset =
            RoadCurveOffset::new(road_curve, lane_offset, 0.0, arc_length, 1.0).unwrap();

        // Inner lane should be shorter: arc_length * (90/100) = arc_length * 0.9
        let expected_arc_length = PI * 90.0 / 2.0;
        assert_relative_eq!(offset.total_arc_length(), expected_arc_length, epsilon = 2.0);
    }

    #[test]
    fn test_arc_road_outer_lane() {
        let road_curve = make_arc_road();
        let arc_length = PI * 100.0 / 2.0;
        // For a left turn (positive curvature), r < 0 is away from the center (outer lane)
        // Lane is 10m to the right = away from the center of the left turn = outer lane (radius = 110m)
        let lane_offset = Arc::new(ConstantFunction::new(-10.0, 0.0, arc_length));

        let offset =
            RoadCurveOffset::new(road_curve, lane_offset, 0.0, arc_length, 1.0).unwrap();

        // Outer lane should be longer: arc_length * (110/100) = arc_length * 1.1
        let expected_arc_length = PI * 110.0 / 2.0;
        assert_relative_eq!(offset.total_arc_length(), expected_arc_length, epsilon = 2.0);
    }

    #[test]
    fn test_s_from_p_and_p_from_s_inverse() {
        let road_curve = make_arc_road();
        let arc_length = PI * 100.0 / 2.0;
        let lane_offset = Arc::new(ConstantFunction::new(10.0, 0.0, arc_length));

        let offset =
            RoadCurveOffset::new(road_curve, lane_offset, 0.0, arc_length, 1.0).unwrap();

        // Test that p_from_s(s_from_p(p)) ≈ p
        for i in 0..=10 {
            let p = i as f64 * arc_length / 10.0;
            let s = offset.calc_s_from_p(p);
            let p_recovered = offset.calc_p_from_s(s);
            assert_relative_eq!(p_recovered, p, epsilon = 1.0);
        }
    }

    #[test]
    fn test_closure_interface() {
        let road_curve = make_straight_road();
        let lane_offset = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let offset = RoadCurveOffset::new(road_curve, lane_offset, 0.0, 100.0, 1.0).unwrap();

        let s_from_p = offset.s_from_p();
        let p_from_s = offset.p_from_s();

        assert_relative_eq!(s_from_p(50.0), 50.0, epsilon = 0.5);
        assert_relative_eq!(p_from_s(50.0), 50.0, epsilon = 0.5);
    }

    #[test]
    fn test_boundary_conditions() {
        let road_curve = make_straight_road();
        let lane_offset = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let offset = RoadCurveOffset::new(road_curve, lane_offset, 0.0, 100.0, 1.0).unwrap();

        // Test values outside the range
        assert_relative_eq!(offset.calc_s_from_p(-10.0), 0.0);
        assert_relative_eq!(offset.calc_s_from_p(110.0), offset.total_arc_length());

        assert_relative_eq!(offset.calc_p_from_s(-10.0), 0.0);
        assert_relative_eq!(offset.calc_p_from_s(offset.total_arc_length() + 10.0), 100.0);
    }

    #[test]
    fn test_road_with_elevation() {
        // Create a road with elevation (10% grade)
        let ground_curve = Arc::new(
            LineGroundCurve::from_heading(
                TOLERANCE,
                nalgebra::Vector2::new(0.0, 0.0),
                0.0,
                100.0, // length
                0.0,
                100.0,
            )
            .unwrap(),
        );
        // 10% grade: z = 0.1 * p
        let elevation = Arc::new(CubicPolynomial::linear(0.0, 0.1, 0.0, 100.0));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let road_curve = Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            TOLERANCE,
            1.0,
        ));

        let lane_offset = Arc::new(ConstantFunction::zero(0.0, 100.0));
        let offset = RoadCurveOffset::new(road_curve, lane_offset, 0.0, 100.0, 1.0).unwrap();

        // Arc length should be longer than flat road due to elevation
        // For 10% grade: arc_length = sqrt(1 + 0.1^2) * 100 ≈ 100.5
        let expected_arc_length = (1.0 + 0.1_f64.powi(2)).sqrt() * 100.0;
        assert_relative_eq!(offset.total_arc_length(), expected_arc_length, epsilon = 1.0);
    }
}
