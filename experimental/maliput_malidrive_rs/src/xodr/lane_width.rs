//! XODR Lane Width types.
//!
//! Lane width defines the width of a lane as a cubic polynomial function
//! of the s-coordinate along the lane section.

/// Lane width record describing the width as a cubic polynomial.
///
/// The width is defined as: width(ds) = a + b*ds + c*ds² + d*ds³
/// where ds is the distance from the start of the width record within the lane section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneWidth {
    /// Start position (s-offset) of this width record within the lane section.
    pub s_offset: f64,
    /// Constant coefficient (width at s_offset).
    pub a: f64,
    /// Linear coefficient.
    pub b: f64,
    /// Quadratic coefficient.
    pub c: f64,
    /// Cubic coefficient.
    pub d: f64,
}

impl LaneWidth {
    /// Creates a new lane width record.
    pub fn new(s_offset: f64, a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { s_offset, a, b, c, d }
    }

    /// Creates a constant lane width.
    pub fn constant(s_offset: f64, width: f64) -> Self {
        Self::new(s_offset, width, 0.0, 0.0, 0.0)
    }

    /// Evaluates the width at the given s-offset within the lane section.
    ///
    /// # Arguments
    /// * `s` - The s-offset at which to evaluate the width (relative to lane section start).
    ///
    /// # Returns
    /// The lane width value (always non-negative).
    pub fn width_at(&self, s: f64) -> f64 {
        let ds = s - self.s_offset;
        let width = self.a + self.b * ds + self.c * ds * ds + self.d * ds * ds * ds;
        width.max(0.0) // Width should never be negative
    }

    /// Evaluates the derivative of the width at the given s-offset.
    ///
    /// # Returns
    /// The rate of change of the width (d(width)/ds).
    pub fn width_dot_at(&self, s: f64) -> f64 {
        let ds = s - self.s_offset;
        self.b + 2.0 * self.c * ds + 3.0 * self.d * ds * ds
    }

    /// Evaluates the second derivative of the width at the given s-offset.
    pub fn width_dot_dot_at(&self, s: f64) -> f64 {
        let ds = s - self.s_offset;
        2.0 * self.c + 6.0 * self.d * ds
    }

    /// Returns true if this is a constant width (b, c, d are all zero).
    pub fn is_constant(&self) -> bool {
        self.b == 0.0 && self.c == 0.0 && self.d == 0.0
    }
}

impl Default for LaneWidth {
    fn default() -> Self {
        Self::new(0.0, 3.5, 0.0, 0.0, 0.0) // Default to 3.5m constant width
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_lane_width_creation() {
        let width = LaneWidth::new(0.0, 3.5, 0.1, 0.01, 0.001);
        assert_relative_eq!(width.s_offset, 0.0);
        assert_relative_eq!(width.a, 3.5);
        assert_relative_eq!(width.b, 0.1);
        assert_relative_eq!(width.c, 0.01);
        assert_relative_eq!(width.d, 0.001);
    }

    #[test]
    fn test_lane_width_constant() {
        let width = LaneWidth::constant(5.0, 4.0);
        assert_relative_eq!(width.s_offset, 5.0);
        assert_relative_eq!(width.a, 4.0);
        assert!(width.is_constant());
    }

    #[test]
    fn test_lane_width_evaluation() {
        // Constant width
        let width = LaneWidth::constant(0.0, 3.5);
        assert_relative_eq!(width.width_at(0.0), 3.5);
        assert_relative_eq!(width.width_at(100.0), 3.5);
        assert_relative_eq!(width.width_dot_at(100.0), 0.0);

        // Linear width: width(ds) = 3.0 + 0.1*ds
        let width = LaneWidth::new(0.0, 3.0, 0.1, 0.0, 0.0);
        assert_relative_eq!(width.width_at(0.0), 3.0);
        assert_relative_eq!(width.width_at(10.0), 4.0);
        assert_relative_eq!(width.width_dot_at(10.0), 0.1);
    }

    #[test]
    fn test_lane_width_with_s_offset() {
        // Width starting at s_offset=10: width(ds) = 3.5 + 0.2*ds where ds = s - 10
        let width = LaneWidth::new(10.0, 3.5, 0.2, 0.0, 0.0);
        assert_relative_eq!(width.width_at(10.0), 3.5);
        assert_relative_eq!(width.width_at(15.0), 4.5); // 3.5 + 0.2*5 = 4.5
    }

    #[test]
    fn test_lane_width_non_negative() {
        // Even with negative polynomial result, width should be non-negative
        let width = LaneWidth::new(0.0, 1.0, -0.5, 0.0, 0.0);
        assert_relative_eq!(width.width_at(0.0), 1.0);
        assert_relative_eq!(width.width_at(2.0), 0.0); // 1.0 - 1.0 = 0, clamped to 0
        assert_relative_eq!(width.width_at(3.0), 0.0); // Would be -0.5, clamped to 0
    }

    #[test]
    fn test_lane_width_default() {
        let width = LaneWidth::default();
        assert_relative_eq!(width.s_offset, 0.0);
        assert_relative_eq!(width.a, 3.5); // Standard lane width
        assert!(width.is_constant());
    }
}
