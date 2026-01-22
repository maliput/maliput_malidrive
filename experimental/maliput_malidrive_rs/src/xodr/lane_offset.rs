//! XODR Lane Offset types.
//!
//! Lane offset defines the lateral offset of the lane reference line from
//! the road reference line.

/// Lane offset record describing the lateral offset as a cubic polynomial.
///
/// The offset is defined as: offset(ds) = a + b*ds + c*ds² + d*ds³
/// where ds is the distance from the start of the lane offset record.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneOffset {
    /// Start position (s-coordinate) of this lane offset record.
    pub s: f64,
    /// Constant coefficient (offset at s).
    pub a: f64,
    /// Linear coefficient.
    pub b: f64,
    /// Quadratic coefficient.
    pub c: f64,
    /// Cubic coefficient.
    pub d: f64,
}

impl LaneOffset {
    /// Creates a new lane offset record.
    pub fn new(s: f64, a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { s, a, b, c, d }
    }

    /// Creates a constant lane offset.
    pub fn constant(s: f64, offset: f64) -> Self {
        Self::new(s, offset, 0.0, 0.0, 0.0)
    }

    /// Evaluates the offset at the given s-coordinate.
    ///
    /// # Arguments
    /// * `s_eval` - The s-coordinate at which to evaluate the offset.
    ///
    /// # Returns
    /// The lateral offset value.
    pub fn offset_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        self.a + self.b * ds + self.c * ds * ds + self.d * ds * ds * ds
    }

    /// Evaluates the derivative of the offset at the given s-coordinate.
    ///
    /// # Returns
    /// The rate of change of the offset (d(offset)/ds).
    pub fn offset_dot_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        self.b + 2.0 * self.c * ds + 3.0 * self.d * ds * ds
    }

    /// Evaluates the second derivative of the offset at the given s-coordinate.
    pub fn offset_dot_dot_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        2.0 * self.c + 6.0 * self.d * ds
    }

    /// Returns true if this is a constant offset (b, c, d are all zero).
    pub fn is_constant(&self) -> bool {
        self.b == 0.0 && self.c == 0.0 && self.d == 0.0
    }
}

impl Default for LaneOffset {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_lane_offset_creation() {
        let offset = LaneOffset::new(10.0, 1.0, 0.1, 0.01, 0.001);
        assert_relative_eq!(offset.s, 10.0);
        assert_relative_eq!(offset.a, 1.0);
        assert_relative_eq!(offset.b, 0.1);
        assert_relative_eq!(offset.c, 0.01);
        assert_relative_eq!(offset.d, 0.001);
    }

    #[test]
    fn test_lane_offset_constant() {
        let offset = LaneOffset::constant(5.0, 2.5);
        assert_relative_eq!(offset.s, 5.0);
        assert_relative_eq!(offset.a, 2.5);
        assert!(offset.is_constant());
    }

    #[test]
    fn test_lane_offset_evaluation() {
        // Constant offset
        let offset = LaneOffset::constant(0.0, 2.0);
        assert_relative_eq!(offset.offset_at(0.0), 2.0);
        assert_relative_eq!(offset.offset_at(10.0), 2.0);
        assert_relative_eq!(offset.offset_dot_at(10.0), 0.0);

        // Linear offset: offset(ds) = 1.0 + 0.1*ds
        let offset = LaneOffset::new(0.0, 1.0, 0.1, 0.0, 0.0);
        assert_relative_eq!(offset.offset_at(0.0), 1.0);
        assert_relative_eq!(offset.offset_at(10.0), 2.0);
        assert_relative_eq!(offset.offset_dot_at(10.0), 0.1);
    }

    #[test]
    fn test_lane_offset_quadratic() {
        // Quadratic offset: offset(ds) = 0 + 0*ds + 0.5*ds²
        let offset = LaneOffset::new(0.0, 0.0, 0.0, 0.5, 0.0);
        assert_relative_eq!(offset.offset_at(0.0), 0.0);
        assert_relative_eq!(offset.offset_at(2.0), 2.0); // 0.5 * 4 = 2
        assert_relative_eq!(offset.offset_dot_at(2.0), 2.0); // 2 * 0.5 * 2 = 2
        assert_relative_eq!(offset.offset_dot_dot_at(2.0), 1.0); // 2 * 0.5 = 1
    }

    #[test]
    fn test_lane_offset_with_s_start() {
        // Offset starting at s=10: offset(ds) = 1.0 + 0.2*ds where ds = s - 10
        let offset = LaneOffset::new(10.0, 1.0, 0.2, 0.0, 0.0);
        assert_relative_eq!(offset.offset_at(10.0), 1.0);
        assert_relative_eq!(offset.offset_at(15.0), 2.0); // 1.0 + 0.2*5 = 2.0
    }

    #[test]
    fn test_lane_offset_default() {
        let offset = LaneOffset::default();
        assert_relative_eq!(offset.s, 0.0);
        assert_relative_eq!(offset.a, 0.0);
        assert!(offset.is_constant());
    }
}
