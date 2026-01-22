//! Lane offset computation.
//!
//! Lane offset is used to compute the lateral position of a lane within a road.

use std::sync::Arc;

use crate::common::MalidriveResult;
use crate::road_curve::Function;

/// Computes the lateral offset for a lane at a given s-coordinate.
///
/// The lane offset combines:
/// 1. The lane offset from the reference line (if any)
/// 2. The accumulated widths of inner lanes
/// 3. Half of the current lane width (to get the center)
pub struct LaneOffset {
    /// The lane's lateral offset from the reference line.
    /// Positive = left of reference line.
    lane_offset_from_reference: Arc<dyn Function>,
    /// The accumulated width function of all lanes between reference and this lane.
    accumulated_inner_width: Arc<dyn Function>,
    /// Half of this lane's width function.
    half_lane_width: Arc<dyn Function>,
    /// Sign: 1.0 for left lanes, -1.0 for right lanes.
    sign: f64,
}

impl LaneOffset {
    /// Creates a new lane offset calculator.
    ///
    /// # Arguments
    /// * `lane_offset_from_reference` - The lateral offset of the lane section from the reference line.
    /// * `accumulated_inner_width` - The sum of widths of all lanes between reference and this lane.
    /// * `half_lane_width` - Half of this lane's width.
    /// * `is_left` - True if this is a left lane (positive r direction).
    pub fn new(
        lane_offset_from_reference: Arc<dyn Function>,
        accumulated_inner_width: Arc<dyn Function>,
        half_lane_width: Arc<dyn Function>,
        is_left: bool,
    ) -> Self {
        Self {
            lane_offset_from_reference,
            accumulated_inner_width,
            half_lane_width,
            sign: if is_left { 1.0 } else { -1.0 },
        }
    }

    /// Computes the r-coordinate (lateral offset) at parameter p.
    ///
    /// The r-coordinate is the distance from the reference line to the lane center.
    pub fn r(&self, p: f64) -> MalidriveResult<f64> {
        let offset = self.lane_offset_from_reference.f(p)?;
        let inner_width = self.accumulated_inner_width.f(p)?;
        let half_width = self.half_lane_width.f(p)?;

        // For left lanes: r = offset + inner_width + half_width
        // For right lanes: r = offset - inner_width - half_width
        Ok(offset + self.sign * (inner_width + half_width))
    }

    /// Computes the derivative dr/dp at parameter p.
    pub fn r_dot(&self, p: f64) -> MalidriveResult<f64> {
        let d_offset = self.lane_offset_from_reference.f_dot(p)?;
        let d_inner_width = self.accumulated_inner_width.f_dot(p)?;
        let d_half_width = self.half_lane_width.f_dot(p)?;

        Ok(d_offset + self.sign * (d_inner_width + d_half_width))
    }

    /// Returns the lane width at parameter p.
    pub fn width(&self, p: f64) -> MalidriveResult<f64> {
        Ok(self.half_lane_width.f(p)? * 2.0)
    }

    /// Returns the lane width derivative at parameter p.
    pub fn width_dot(&self, p: f64) -> MalidriveResult<f64> {
        Ok(self.half_lane_width.f_dot(p)? * 2.0)
    }

    /// Returns the minimum r bound (right edge) at parameter p.
    pub fn r_min(&self, p: f64) -> MalidriveResult<f64> {
        let center = self.r(p)?;
        let half_width = self.half_lane_width.f(p)?;
        Ok(center - half_width)
    }

    /// Returns the maximum r bound (left edge) at parameter p.
    pub fn r_max(&self, p: f64) -> MalidriveResult<f64> {
        let center = self.r(p)?;
        let half_width = self.half_lane_width.f(p)?;
        Ok(center + half_width)
    }

    /// Returns whether this is a left lane.
    pub fn is_left(&self) -> bool {
        self.sign > 0.0
    }

    /// Returns whether this is a right lane.
    pub fn is_right(&self) -> bool {
        self.sign < 0.0
    }
}

/// A simple lane offset for testing and simple cases.
#[derive(Debug)]
pub struct SimpleLaneOffset {
    /// Constant r offset for the lane center.
    r_center: f64,
    /// Constant half-width.
    half_width: f64,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
}

impl SimpleLaneOffset {
    /// Creates a simple constant lane offset.
    pub fn new(r_center: f64, width: f64, p0: f64, p1: f64) -> Self {
        Self {
            r_center,
            half_width: width / 2.0,
            p0,
            p1,
        }
    }

    /// Returns the r-coordinate (lateral offset) at any parameter.
    pub fn r(&self, _p: f64) -> f64 {
        self.r_center
    }

    /// Returns dr/dp (always 0 for constant offset).
    pub fn r_dot(&self, _p: f64) -> f64 {
        0.0
    }

    /// Returns the lane width.
    pub fn width(&self) -> f64 {
        self.half_width * 2.0
    }

    /// Returns the minimum r bound.
    pub fn r_min(&self) -> f64 {
        self.r_center - self.half_width
    }

    /// Returns the maximum r bound.
    pub fn r_max(&self) -> f64 {
        self.r_center + self.half_width
    }

    /// Returns the start parameter.
    pub fn p0(&self) -> f64 {
        self.p0
    }

    /// Returns the end parameter.
    pub fn p1(&self) -> f64 {
        self.p1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::road_curve::CubicPolynomial;
    use approx::assert_relative_eq;

    #[test]
    fn test_simple_lane_offset() {
        // A lane centered at r=5 with width 3.5
        let offset = SimpleLaneOffset::new(5.0, 3.5, 0.0, 100.0);

        assert_relative_eq!(offset.r(50.0), 5.0);
        assert_relative_eq!(offset.r_dot(50.0), 0.0);
        assert_relative_eq!(offset.width(), 3.5);
        assert_relative_eq!(offset.r_min(), 3.25);
        assert_relative_eq!(offset.r_max(), 6.75);
    }

    #[test]
    fn test_lane_offset_left_lane() {
        // Reference offset = 0
        let ref_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        // Inner width = 3.5 (one lane between reference and this lane)
        let inner_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        // Half width = 1.75 (3.5m lane)
        let half_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(1.75, 0.0, 100.0));

        let offset = LaneOffset::new(ref_offset, inner_width, half_width, true);

        // r = 0 + 1 * (3.5 + 1.75) = 5.25
        assert_relative_eq!(offset.r(50.0).unwrap(), 5.25);
        assert!(offset.is_left());
        assert!(!offset.is_right());
    }

    #[test]
    fn test_lane_offset_right_lane() {
        // Reference offset = 0
        let ref_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        // Inner width = 3.5
        let inner_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        // Half width = 1.75
        let half_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(1.75, 0.0, 100.0));

        let offset = LaneOffset::new(ref_offset, inner_width, half_width, false);

        // r = 0 + (-1) * (3.5 + 1.75) = -5.25
        assert_relative_eq!(offset.r(50.0).unwrap(), -5.25);
        assert!(!offset.is_left());
        assert!(offset.is_right());
    }

    #[test]
    fn test_lane_offset_with_reference_offset() {
        // Reference offset = 0.5 (road shifted left)
        let ref_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.5, 0.0, 100.0));
        // Inner width = 0 (this is the first lane)
        let inner_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        // Half width = 1.75
        let half_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(1.75, 0.0, 100.0));

        let offset = LaneOffset::new(ref_offset, inner_width, half_width, true);

        // r = 0.5 + 1 * (0 + 1.75) = 2.25
        assert_relative_eq!(offset.r(50.0).unwrap(), 2.25);
    }

    #[test]
    fn test_lane_offset_width() {
        let ref_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        let inner_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        let half_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(1.75, 0.0, 100.0));

        let offset = LaneOffset::new(ref_offset, inner_width, half_width, true);

        assert_relative_eq!(offset.width(50.0).unwrap(), 3.5);
    }

    #[test]
    fn test_lane_offset_bounds() {
        let ref_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        let inner_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        let half_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(1.75, 0.0, 100.0));

        let offset = LaneOffset::new(ref_offset, inner_width, half_width, true);

        // Center at r = 1.75
        // Min = 1.75 - 1.75 = 0
        // Max = 1.75 + 1.75 = 3.5
        assert_relative_eq!(offset.r_min(50.0).unwrap(), 0.0);
        assert_relative_eq!(offset.r_max(50.0).unwrap(), 3.5);
    }

    #[test]
    fn test_lane_offset_varying_width() {
        // Width that increases with s: w(s) = 3.5 + 0.01*s
        let ref_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        let inner_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));
        // Half width = 1.75 + 0.005*s
        let half_width: Arc<dyn Function> = Arc::new(CubicPolynomial::linear(1.75, 0.005, 0.0, 100.0));

        let offset = LaneOffset::new(ref_offset, inner_width, half_width, true);

        // At s=0: width = 3.5
        assert_relative_eq!(offset.width(0.0).unwrap(), 3.5);

        // At s=100: width = 3.5 + 1.0 = 4.5
        assert_relative_eq!(offset.width(100.0).unwrap(), 4.5);

        // Width derivative = 0.01
        assert_relative_eq!(offset.width_dot(50.0).unwrap(), 0.01);
    }
}
