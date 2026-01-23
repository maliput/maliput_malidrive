//! Lane offset computation.
//!
//! Lane offset is used to compute the lateral position of a lane within a road.
//!
//! This module provides a `MalidriveLaneOffset` struct that implements the `Function` trait,
//! following the C++ implementation pattern where lane offset is computed iteratively
//! from the center lane outward.

use std::sync::Arc;

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::Function;

/// Holds lane offset and width functions of the immediate adjacent lane (closer to center).
///
/// This is used to compute the offset of lanes that are not immediately adjacent to
/// the reference line. The offset for lane i is computed from lane i-1's offset and width.
#[derive(Clone)]
pub struct AdjacentLaneFunctions {
    /// LaneOffset function of the adjacent lane.
    pub offset: Arc<dyn Function>,
    /// LaneWidth function of the adjacent lane.
    pub width: Arc<dyn Function>,
}

impl AdjacentLaneFunctions {
    /// Creates a new AdjacentLaneFunctions.
    pub fn new(offset: Arc<dyn Function>, width: Arc<dyn Function>) -> Self {
        Self { offset, width }
    }
}

/// Lane offset function that implements the Function trait.
///
/// This is built from the previous lane's offset and width, plus the current lane's width,
/// according to:
///
/// `LaneOffset_i(p) = LaneOffset_{i-1}(p) + sign * LaneWidth_{i-1}(p) / 2 + sign * LaneWidth_i(p) / 2`
///
/// Where `p ∈ [p0, p1]` and sign is -1 for right lanes, +1 for left lanes.
///
/// The innermost lane (immediately adjacent to the center lane) uses the reference line
/// offset function when there are no adjacent lane functions.
pub struct MalidriveLaneOffset {
    /// Adjacent lane's offset and width functions (None for the innermost lane).
    adjacent_lane_functions: Option<AdjacentLaneFunctions>,
    /// This lane's width function.
    lane_width: Arc<dyn Function>,
    /// Reference line offset function.
    reference_line_offset: Arc<dyn Function>,
    /// True if the lane is to the right of the center lane.
    at_right: bool,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
    /// Linear tolerance for range validation.
    linear_tolerance: f64,
}

/// Constant for lane at right from center lane.
pub const AT_RIGHT_FROM_CENTER_LANE: bool = true;
/// Constant for lane at left from center lane.
pub const AT_LEFT_FROM_CENTER_LANE: bool = false;

impl MalidriveLaneOffset {
    /// Creates a new MalidriveLaneOffset.
    ///
    /// # Arguments
    /// * `adjacent_lane_functions` - Offset and width of the adjacent lane (closer to center).
    ///   None if this is the innermost lane.
    /// * `lane_width` - This lane's width function.
    /// * `reference_line_offset` - The reference line offset function.
    /// * `at_right` - True if the lane is at the right of the center lane.
    /// * `p0` - Start parameter (must be non-negative).
    /// * `p1` - End parameter (must be greater than p0).
    /// * `linear_tolerance` - Tolerance for range validation (must be positive).
    ///
    /// # Errors
    /// Returns an error if:
    /// - `p0` is negative
    /// - `p1` <= `p0`
    /// - `linear_tolerance` <= 0
    /// - Domain mismatches exist between functions
    pub fn new(
        adjacent_lane_functions: Option<AdjacentLaneFunctions>,
        lane_width: Arc<dyn Function>,
        reference_line_offset: Arc<dyn Function>,
        at_right: bool,
        p0: f64,
        p1: f64,
        linear_tolerance: f64,
    ) -> MalidriveResult<Self> {
        // Validate parameters matching C++ constructor
        if p0 < 0.0 {
            return Err(MalidriveError::ValidationError(
                format!("p0 must be non-negative, got {}", p0)
            ));
        }
        if p1 <= p0 {
            return Err(MalidriveError::ValidationError(
                format!("p1 must be greater than p0: p0={}, p1={}", p0, p1)
            ));
        }
        if linear_tolerance <= 0.0 {
            return Err(MalidriveError::ValidationError(
                format!("linear_tolerance must be positive, got {}", linear_tolerance)
            ));
        }

        // Validate lane_width domain
        if (lane_width.p0() - p0).abs() > linear_tolerance {
            return Err(MalidriveError::ValidationError(
                format!("lane_width p0 ({}) doesn't match p0 ({})", lane_width.p0(), p0)
            ));
        }
        if (lane_width.p1() - p1).abs() > linear_tolerance {
            return Err(MalidriveError::ValidationError(
                format!("lane_width p1 ({}) doesn't match p1 ({})", lane_width.p1(), p1)
            ));
        }

        // Validate reference_line_offset domain
        if reference_line_offset.p0() > p0 + linear_tolerance {
            return Err(MalidriveError::ValidationError(
                format!("reference_line_offset p0 ({}) must be <= p0 ({})",
                        reference_line_offset.p0(), p0)
            ));
        }
        if reference_line_offset.p1() < p1 - linear_tolerance {
            return Err(MalidriveError::ValidationError(
                format!("reference_line_offset p1 ({}) must be >= p1 ({})",
                        reference_line_offset.p1(), p1)
            ));
        }

        // Validate adjacent_lane_functions domain if present
        if let Some(ref adj) = adjacent_lane_functions {
            if (adj.offset.p0() - p0).abs() > linear_tolerance {
                return Err(MalidriveError::ValidationError(
                    format!("adjacent offset p0 ({}) doesn't match p0 ({})",
                            adj.offset.p0(), p0)
                ));
            }
            if (adj.offset.p1() - p1).abs() > linear_tolerance {
                return Err(MalidriveError::ValidationError(
                    format!("adjacent offset p1 ({}) doesn't match p1 ({})",
                            adj.offset.p1(), p1)
                ));
            }
            if (adj.width.p0() - p0).abs() > linear_tolerance {
                return Err(MalidriveError::ValidationError(
                    format!("adjacent width p0 ({}) doesn't match p0 ({})",
                            adj.width.p0(), p0)
                ));
            }
            if (adj.width.p1() - p1).abs() > linear_tolerance {
                return Err(MalidriveError::ValidationError(
                    format!("adjacent width p1 ({}) doesn't match p1 ({})",
                            adj.width.p1(), p1)
                ));
            }
        }

        Ok(Self {
            adjacent_lane_functions,
            lane_width,
            reference_line_offset,
            at_right,
            p0,
            p1,
            linear_tolerance,
        })
    }

    /// Returns the lane sign: -1 for right lanes, +1 for left lanes.
    fn lane_sign(&self) -> f64 {
        if self.at_right { -1.0 } else { 1.0 }
    }

    /// Validates and clamps parameter p to the valid range.
    fn validate_p(&self, p: f64) -> f64 {
        // Clamp p to the valid range with tolerance
        let epsilon = 1e-15; // Function::kEpsilon equivalent
        if p < self.p0 - self.linear_tolerance - epsilon {
            self.p0
        } else if p > self.p1 + self.linear_tolerance + epsilon {
            self.p1
        } else {
            p.clamp(self.p0, self.p1)
        }
    }

    /// Returns whether this lane is to the right of center.
    pub fn is_at_right(&self) -> bool {
        self.at_right
    }

    /// Returns whether this lane is to the left of center.
    pub fn is_at_left(&self) -> bool {
        !self.at_right
    }

    /// Returns the lane width function.
    pub fn lane_width(&self) -> &Arc<dyn Function> {
        &self.lane_width
    }
}

impl Function for MalidriveLaneOffset {
    fn f(&self, p: f64) -> MalidriveResult<f64> {
        let p = self.validate_p(p);
        let sign = self.lane_sign();

        // Get the adjacent lane's offset (or reference line offset for innermost lane)
        let lane_offset_i_1 = if let Some(ref adj) = self.adjacent_lane_functions {
            adj.offset.f(p)?
        } else {
            self.reference_line_offset.f(p)?
        };

        // Get the adjacent lane's width (or 0 for innermost lane)
        let lane_width_i_1 = if let Some(ref adj) = self.adjacent_lane_functions {
            adj.width.f(p)?
        } else {
            0.0
        };

        // LaneOffset_i = LaneOffset_{i-1} + sign * LaneWidth_{i-1}/2 + sign * LaneWidth_i/2
        Ok(lane_offset_i_1 + sign * lane_width_i_1 / 2.0 + sign * self.lane_width.f(p)? / 2.0)
    }

    fn f_dot(&self, p: f64) -> MalidriveResult<f64> {
        let p = self.validate_p(p);
        let sign = self.lane_sign();

        let lane_offset_i_1_dot = if let Some(ref adj) = self.adjacent_lane_functions {
            adj.offset.f_dot(p)?
        } else {
            self.reference_line_offset.f_dot(p)?
        };

        let lane_width_i_1_dot = if let Some(ref adj) = self.adjacent_lane_functions {
            adj.width.f_dot(p)?
        } else {
            0.0
        };

        Ok(lane_offset_i_1_dot + sign * lane_width_i_1_dot / 2.0 + sign * self.lane_width.f_dot(p)? / 2.0)
    }

    fn f_dot_dot(&self, p: f64) -> MalidriveResult<f64> {
        let p = self.validate_p(p);
        let sign = self.lane_sign();

        let lane_offset_i_1_dot_dot = if let Some(ref adj) = self.adjacent_lane_functions {
            adj.offset.f_dot_dot(p)?
        } else {
            self.reference_line_offset.f_dot_dot(p)?
        };

        let lane_width_i_1_dot_dot = if let Some(ref adj) = self.adjacent_lane_functions {
            adj.width.f_dot_dot(p)?
        } else {
            0.0
        };

        Ok(lane_offset_i_1_dot_dot + sign * lane_width_i_1_dot_dot / 2.0 + sign * self.lane_width.f_dot_dot(p)? / 2.0)
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

/// Legacy LaneOffset structure for backward compatibility.
///
/// Computes the lateral offset for a lane at a given s-coordinate using
/// a simpler approach with accumulated inner width.
pub struct LaneOffset {
    /// The lane's lateral offset from the reference line.
    lane_offset_from_reference: Arc<dyn Function>,
    /// The accumulated width function of all lanes between reference and this lane.
    accumulated_inner_width: Arc<dyn Function>,
    /// Half of this lane's width function.
    half_lane_width: Arc<dyn Function>,
    /// Sign: 1.0 for left lanes, -1.0 for right lanes.
    sign: f64,
}

impl LaneOffset {
    /// Creates a new lane offset calculator (legacy API).
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
    pub fn r(&self, p: f64) -> MalidriveResult<f64> {
        let offset = self.lane_offset_from_reference.f(p)?;
        let inner_width = self.accumulated_inner_width.f(p)?;
        let half_width = self.half_lane_width.f(p)?;

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

    const TOLERANCE: f64 = 1e-6;

    // ==========================================================================
    // MalidriveLaneOffset tests (new implementation)
    // ==========================================================================

    #[test]
    fn test_malidrive_lane_offset_innermost_right_lane() {
        // Innermost right lane (no adjacent lane functions)
        // reference_line_offset = 0, lane_width = 3.5
        // Expected offset = 0 + (-1) * 0 / 2 + (-1) * 3.5 / 2 = -1.75
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let dut = MalidriveLaneOffset::new(
            None,  // No adjacent lane
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            0.0,
            100.0,
            TOLERANCE,
        ).unwrap();

        assert!(dut.is_at_right());
        assert!(!dut.is_at_left());
        assert_relative_eq!(dut.f(50.0).unwrap(), -1.75, epsilon = TOLERANCE);
        assert_relative_eq!(dut.f_dot(50.0).unwrap(), 0.0, epsilon = TOLERANCE);
        assert_relative_eq!(dut.f_dot_dot(50.0).unwrap(), 0.0, epsilon = TOLERANCE);
    }

    #[test]
    fn test_malidrive_lane_offset_innermost_left_lane() {
        // Innermost left lane (no adjacent lane functions)
        // reference_line_offset = 0, lane_width = 3.5
        // Expected offset = 0 + (+1) * 0 / 2 + (+1) * 3.5 / 2 = 1.75
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let dut = MalidriveLaneOffset::new(
            None,
            lane_width,
            reference_line_offset,
            AT_LEFT_FROM_CENTER_LANE,
            0.0,
            100.0,
            TOLERANCE,
        ).unwrap();

        assert!(!dut.is_at_right());
        assert!(dut.is_at_left());
        assert_relative_eq!(dut.f(50.0).unwrap(), 1.75, epsilon = TOLERANCE);
    }

    #[test]
    fn test_malidrive_lane_offset_with_reference_line_offset() {
        // Innermost right lane with reference line offset = 0.5
        // Expected offset = 0.5 + (-1) * 0 / 2 + (-1) * 3.5 / 2 = 0.5 - 1.75 = -1.25
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.5, 0.0, 100.0));

        let dut = MalidriveLaneOffset::new(
            None,
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            0.0,
            100.0,
            TOLERANCE,
        ).unwrap();

        assert_relative_eq!(dut.f(50.0).unwrap(), -1.25, epsilon = TOLERANCE);
    }

    #[test]
    fn test_malidrive_lane_offset_with_adjacent_lane() {
        // Second right lane with adjacent lane functions
        // adjacent_offset = -1.75, adjacent_width = 3.5, this_width = 3.0
        // Expected offset = -1.75 + (-1) * 3.5 / 2 + (-1) * 3.0 / 2
        //                 = -1.75 - 1.75 - 1.5 = -5.0
        let adjacent_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(-1.75, 0.0, 100.0));
        let adjacent_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.0, 0.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let adjacent_lane_functions = AdjacentLaneFunctions::new(adjacent_offset, adjacent_width);

        let dut = MalidriveLaneOffset::new(
            Some(adjacent_lane_functions),
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            0.0,
            100.0,
            TOLERANCE,
        ).unwrap();

        assert_relative_eq!(dut.f(50.0).unwrap(), -5.0, epsilon = TOLERANCE);
    }

    #[test]
    fn test_malidrive_lane_offset_with_adjacent_lane_left() {
        // Second left lane with adjacent lane functions
        // adjacent_offset = 1.75, adjacent_width = 3.5, this_width = 3.0
        // Expected offset = 1.75 + (+1) * 3.5 / 2 + (+1) * 3.0 / 2
        //                 = 1.75 + 1.75 + 1.5 = 5.0
        let adjacent_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(1.75, 0.0, 100.0));
        let adjacent_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.0, 0.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let adjacent_lane_functions = AdjacentLaneFunctions::new(adjacent_offset, adjacent_width);

        let dut = MalidriveLaneOffset::new(
            Some(adjacent_lane_functions),
            lane_width,
            reference_line_offset,
            AT_LEFT_FROM_CENTER_LANE,
            0.0,
            100.0,
            TOLERANCE,
        ).unwrap();

        assert_relative_eq!(dut.f(50.0).unwrap(), 5.0, epsilon = TOLERANCE);
    }

    #[test]
    fn test_malidrive_lane_offset_varying_width() {
        // Lane with linearly increasing width: w(p) = 3.0 + 0.01 * p
        // At p=0: w=3.0, at p=100: w=4.0
        // Innermost right lane, reference offset = 0
        // At p=0: offset = -1.5
        // At p=100: offset = -2.0
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::linear(3.0, 0.01, 0.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let dut = MalidriveLaneOffset::new(
            None,
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            0.0,
            100.0,
            TOLERANCE,
        ).unwrap();

        assert_relative_eq!(dut.f(0.0).unwrap(), -1.5, epsilon = TOLERANCE);
        assert_relative_eq!(dut.f(100.0).unwrap(), -2.0, epsilon = TOLERANCE);

        // Derivative: d/dp(-w/2) = -0.01/2 = -0.005
        assert_relative_eq!(dut.f_dot(50.0).unwrap(), -0.005, epsilon = TOLERANCE);
    }

    #[test]
    fn test_malidrive_lane_offset_validation_negative_p0() {
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, -10.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, -10.0, 100.0));

        let result = MalidriveLaneOffset::new(
            None,
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            -10.0,  // Invalid: negative p0
            100.0,
            TOLERANCE,
        );

        assert!(result.is_err());
    }

    #[test]
    fn test_malidrive_lane_offset_validation_p1_not_greater_than_p0() {
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 50.0, 50.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let result = MalidriveLaneOffset::new(
            None,
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            50.0,
            50.0,  // Invalid: p1 == p0
            TOLERANCE,
        );

        assert!(result.is_err());
    }

    #[test]
    fn test_malidrive_lane_offset_validation_negative_tolerance() {
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let result = MalidriveLaneOffset::new(
            None,
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            0.0,
            100.0,
            -1.0,  // Invalid: negative tolerance
        );

        assert!(result.is_err());
    }

    #[test]
    fn test_malidrive_lane_offset_p0_p1() {
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 10.0, 90.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let dut = MalidriveLaneOffset::new(
            None,
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            10.0,
            90.0,
            TOLERANCE,
        ).unwrap();

        assert_relative_eq!(dut.p0(), 10.0, epsilon = TOLERANCE);
        assert_relative_eq!(dut.p1(), 90.0, epsilon = TOLERANCE);
    }

    #[test]
    fn test_malidrive_lane_offset_is_g1_contiguous() {
        let lane_width: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(3.5, 0.0, 100.0));
        let reference_line_offset: Arc<dyn Function> = Arc::new(CubicPolynomial::constant(0.0, 0.0, 100.0));

        let dut = MalidriveLaneOffset::new(
            None,
            lane_width,
            reference_line_offset,
            AT_RIGHT_FROM_CENTER_LANE,
            0.0,
            100.0,
            TOLERANCE,
        ).unwrap();

        assert!(dut.is_g1_contiguous());
    }

    // ==========================================================================
    // SimpleLaneOffset tests
    // ==========================================================================

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

    // ==========================================================================
    // Legacy LaneOffset tests (backward compatibility)
    // ==========================================================================

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
