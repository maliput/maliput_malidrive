//! Piecewise ground curve implementation.
//!
//! A piecewise ground curve is composed of multiple ground curve segments
//! concatenated together. This is the typical representation for a complete
//! OpenDRIVE road reference line.

use nalgebra::Vector2;
use std::sync::Arc;

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::GroundCurve;

/// A segment in a piecewise ground curve.
struct GroundCurveSegment {
    /// The ground curve for this segment.
    curve: Arc<dyn GroundCurve>,
}

impl std::fmt::Debug for GroundCurveSegment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GroundCurveSegment")
            .field("p0", &self.curve.p0())
            .field("p1", &self.curve.p1())
            .finish()
    }
}

/// A piecewise ground curve composed of multiple segments.
///
/// Segments are stored in order of increasing p values and must be contiguous
/// (the p1 of one segment equals the p0 of the next).
#[derive(Debug)]
pub struct PiecewiseGroundCurve {
    /// The segments, ordered by p0.
    segments: Vec<GroundCurveSegment>,
    /// Cached total arc length.
    total_arc_length: f64,
    /// Start parameter.
    p0: f64,
    /// End parameter.
    p1: f64,
    /// Linear tolerance.
    linear_tolerance: f64,
}

impl PiecewiseGroundCurve {
    /// Creates a new empty piecewise ground curve.
    pub fn new(linear_tolerance: f64) -> Self {
        Self {
            segments: Vec::new(),
            total_arc_length: 0.0,
            p0: 0.0,
            p1: 0.0,
            linear_tolerance,
        }
    }

    /// Creates a piecewise ground curve from a vector of segments.
    ///
    /// The segments must be provided in order of increasing p values.
    pub fn from_segments(
        linear_tolerance: f64,
        segments: Vec<Arc<dyn GroundCurve>>,
    ) -> MalidriveResult<Self> {
        if segments.is_empty() {
            return Ok(Self::new(linear_tolerance));
        }

        let mut curve = Self::new(linear_tolerance);
        for segment in segments {
            curve.add_segment(segment)?;
        }
        Ok(curve)
    }

    /// Adds a segment to the curve.
    ///
    /// The segment's p0 must equal the current curve's p1 (or be the first segment).
    pub fn add_segment(&mut self, curve: Arc<dyn GroundCurve>) -> MalidriveResult<()> {
        if !self.segments.is_empty() {
            let last_p1 = self.p1;
            let new_p0 = curve.p0();
            let gap = (last_p1 - new_p0).abs();
            if gap > self.linear_tolerance {
                return Err(MalidriveError::ValidationError(format!(
                    "Segment gap: last segment ends at p={}, new segment starts at p={} (gap={})",
                    last_p1, new_p0, gap
                )));
            }
        }

        if self.segments.is_empty() {
            self.p0 = curve.p0();
        }
        self.p1 = curve.p1();
        self.total_arc_length += curve.arc_length();

        self.segments.push(GroundCurveSegment { curve });
        Ok(())
    }

    /// Returns the number of segments.
    pub fn num_segments(&self) -> usize {
        self.segments.len()
    }

    /// Returns true if the curve is empty.
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Finds the segment containing parameter p.
    fn find_segment(&self, p: f64) -> MalidriveResult<&GroundCurveSegment> {
        // Binary search for the segment
        // We find the last segment where p0 <= p
        let index = self.segments.partition_point(|seg| seg.curve.p0() <= p);

        if index == 0 {
            // p is before all segments
            if self.segments.is_empty() {
                return Err(MalidriveError::ValidationError(
                    "No segments in piecewise curve".to_string(),
                ));
            }
            // Check if within tolerance of first segment
            let first = &self.segments[0];
            if p >= first.curve.p0() - self.linear_tolerance {
                return Ok(first);
            }
            return Err(MalidriveError::ParameterOutOfRange {
                parameter: "p".to_string(),
                value: p,
                min: self.p0,
                max: self.p1,
            });
        }

        // Check the segment at index-1
        let segment = &self.segments[index - 1];
        if p <= segment.curve.p1() + self.linear_tolerance {
            return Ok(segment);
        }

        // p is past the end of that segment but before the next (if any)
        if index < self.segments.len() {
            let next_segment = &self.segments[index];
            if p >= next_segment.curve.p0() - self.linear_tolerance {
                return Ok(next_segment);
            }
        }

        Err(MalidriveError::ParameterOutOfRange {
            parameter: "p".to_string(),
            value: p,
            min: self.p0,
            max: self.p1,
        })
    }

    /// Validates that p is within the curve's range.
    fn validate_p(&self, p: f64) -> MalidriveResult<()> {
        if self.segments.is_empty() {
            return Err(MalidriveError::ValidationError(
                "Empty piecewise curve".to_string(),
            ));
        }

        if p < self.p0 - self.linear_tolerance || p > self.p1 + self.linear_tolerance {
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

impl GroundCurve for PiecewiseGroundCurve {
    fn p_from_xodr_p(&self, xodr_p: f64) -> MalidriveResult<f64> {
        Ok(xodr_p)
    }

    fn g(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        self.validate_p(p)?;
        let segment = self.find_segment(p)?;
        segment.curve.g(p)
    }

    fn g_dot(&self, p: f64) -> MalidriveResult<Vector2<f64>> {
        self.validate_p(p)?;
        let segment = self.find_segment(p)?;
        segment.curve.g_dot(p)
    }

    fn heading(&self, p: f64) -> MalidriveResult<f64> {
        self.validate_p(p)?;
        let segment = self.find_segment(p)?;
        segment.curve.heading(p)
    }

    fn heading_dot(&self, p: f64) -> MalidriveResult<f64> {
        self.validate_p(p)?;
        let segment = self.find_segment(p)?;
        segment.curve.heading_dot(p)
    }

    fn g_inverse(&self, xy: &Vector2<f64>) -> MalidriveResult<f64> {
        if self.segments.is_empty() {
            return Err(MalidriveError::ValidationError(
                "Empty piecewise curve".to_string(),
            ));
        }

        // Search all segments for the closest point
        let mut best_p = self.p0;
        let mut best_dist_sq = f64::MAX;

        for segment in &self.segments {
            // Try to find the inverse on this segment
            match segment.curve.g_inverse(xy) {
                Ok(p) => {
                    let pos = segment.curve.g(p)?;
                    let dist_sq = (pos - xy).norm_squared();
                    if dist_sq < best_dist_sq {
                        best_dist_sq = dist_sq;
                        best_p = p;
                    }
                }
                Err(_) => {
                    // Try endpoints
                    if let Ok(pos) = segment.curve.g(segment.curve.p0()) {
                        let dist_sq = (pos - xy).norm_squared();
                        if dist_sq < best_dist_sq {
                            best_dist_sq = dist_sq;
                            best_p = segment.curve.p0();
                        }
                    }
                    if let Ok(pos) = segment.curve.g(segment.curve.p1()) {
                        let dist_sq = (pos - xy).norm_squared();
                        if dist_sq < best_dist_sq {
                            best_dist_sq = dist_sq;
                            best_p = segment.curve.p1();
                        }
                    }
                }
            }
        }

        // Verify result
        let final_pos = self.g(best_p)?;
        let dist = (final_pos - xy).norm();
        if dist > self.linear_tolerance * 10.0 {
            return Err(MalidriveError::GeometryError(format!(
                "Point ({}, {}) not on piecewise curve (distance: {})",
                xy.x, xy.y, dist
            )));
        }

        Ok(best_p)
    }

    fn arc_length(&self) -> f64 {
        self.total_arc_length
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
        // Check that all segments are G1 contiguous
        if self.segments.len() <= 1 {
            return true;
        }

        for seg in &self.segments {
            if !seg.curve.is_g1_contiguous() {
                return false;
            }
        }

        // Check continuity between segments
        let tolerance = self.linear_tolerance;
        for i in 0..self.segments.len() - 1 {
            let current = &self.segments[i];
            let next = &self.segments[i + 1];

            // Check position continuity
            if let (Ok(end_pos), Ok(start_pos)) = (
                current.curve.g(current.curve.p1()),
                next.curve.g(next.curve.p0()),
            ) {
                if (end_pos - start_pos).norm() > tolerance {
                    return false;
                }
            }

            // Check heading continuity
            if let (Ok(end_heading), Ok(start_heading)) = (
                current.curve.heading(current.curve.p1()),
                next.curve.heading(next.curve.p0()),
            ) {
                let diff = (end_heading - start_heading).abs();
                if diff > tolerance && (diff - std::f64::consts::PI * 2.0).abs() > tolerance {
                    return false;
                }
            }
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::road_curve::LineGroundCurve;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    const TOLERANCE: f64 = 1e-6;

    fn make_line(x0: f64, y0: f64, heading: f64, length: f64, p0: f64) -> Arc<dyn GroundCurve> {
        Arc::new(LineGroundCurve::from_heading(
            TOLERANCE,
            Vector2::new(x0, y0),
            heading,
            length,
            p0,
            p0 + length,
        ).unwrap())
    }

    #[test]
    fn test_empty_piecewise_curve() {
        let curve = PiecewiseGroundCurve::new(TOLERANCE);
        assert!(curve.is_empty());
        assert_eq!(curve.num_segments(), 0);
    }

    #[test]
    fn test_single_segment() {
        let segment = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![segment]).unwrap();

        assert_eq!(curve.num_segments(), 1);
        assert_relative_eq!(curve.p0(), 0.0);
        assert_relative_eq!(curve.p1(), 100.0);
        assert_relative_eq!(curve.arc_length(), 100.0);

        let pos = curve.g(50.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_two_connected_segments() {
        // Line going east for 100m, then north for 50m
        let seg1 = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let seg2 = make_line(100.0, 0.0, PI / 2.0, 50.0, 100.0);

        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg1, seg2]).unwrap();

        assert_eq!(curve.num_segments(), 2);
        assert_relative_eq!(curve.arc_length(), 150.0);

        // Test first segment
        let pos = curve.g(50.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-6);

        // Test junction point
        let pos = curve.g(100.0).unwrap();
        assert_relative_eq!(pos.x, 100.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-6);

        // Test second segment
        let pos = curve.g(125.0).unwrap();
        assert_relative_eq!(pos.x, 100.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 25.0, epsilon = 1e-6);
    }

    #[test]
    fn test_heading_across_segments() {
        let seg1 = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let seg2 = make_line(100.0, 0.0, PI / 2.0, 50.0, 100.0);

        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg1, seg2]).unwrap();

        // First segment heading
        assert_relative_eq!(curve.heading(50.0).unwrap(), 0.0, epsilon = 1e-9);

        // Second segment heading
        assert_relative_eq!(curve.heading(125.0).unwrap(), PI / 2.0, epsilon = 1e-9);
    }

    #[test]
    fn test_add_segment_incrementally() {
        let mut curve = PiecewiseGroundCurve::new(TOLERANCE);

        let seg1 = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        curve.add_segment(seg1).unwrap();
        assert_eq!(curve.num_segments(), 1);

        let seg2 = make_line(100.0, 0.0, 0.0, 50.0, 100.0);
        curve.add_segment(seg2).unwrap();
        assert_eq!(curve.num_segments(), 2);

        assert_relative_eq!(curve.arc_length(), 150.0);
    }

    #[test]
    fn test_gap_detection() {
        let mut curve = PiecewiseGroundCurve::new(TOLERANCE);

        let seg1 = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        curve.add_segment(seg1).unwrap();

        // Try to add a segment with a gap
        let seg2 = make_line(100.0, 0.0, 0.0, 50.0, 110.0); // starts at p=110, not 100
        let result = curve.add_segment(seg2);
        assert!(result.is_err());
    }

    #[test]
    fn test_parameter_out_of_range() {
        let seg = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg]).unwrap();

        assert!(curve.g(-1.0).is_err());
        assert!(curve.g(101.0).is_err());
    }

    #[test]
    fn test_g_inverse() {
        let seg = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg]).unwrap();

        // Point on curve
        let p = curve.g_inverse(&Vector2::new(50.0, 0.0)).unwrap();
        assert_relative_eq!(p, 50.0, epsilon = TOLERANCE);

        // Point at start
        let p = curve.g_inverse(&Vector2::new(0.0, 0.0)).unwrap();
        assert_relative_eq!(p, 0.0, epsilon = TOLERANCE);

        // Point at end
        let p = curve.g_inverse(&Vector2::new(100.0, 0.0)).unwrap();
        assert_relative_eq!(p, 100.0, epsilon = TOLERANCE);
    }

    // Additional tests for PiecewiseGroundCurve
    #[test]
    fn test_piecewise_is_g1_contiguous_single_segment() {
        let seg = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg]).unwrap();
        assert!(curve.is_g1_contiguous());
    }

    #[test]
    fn test_piecewise_is_g1_contiguous_connected() {
        // Two lines that connect smoothly (same position and heading)
        let seg1 = make_line(0.0, 0.0, 0.0, 50.0, 0.0);  // Goes east for 50m
        let seg2 = make_line(50.0, 0.0, 0.0, 50.0, 50.0); // Continues east for another 50m
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg1, seg2]).unwrap();
        assert!(curve.is_g1_contiguous());
    }

    #[test]
    fn test_piecewise_is_not_g1_heading_discontinuity() {
        // Two lines with different headings at the junction
        let seg1 = make_line(0.0, 0.0, 0.0, 100.0, 0.0);        // Goes east
        let seg2 = make_line(100.0, 0.0, PI / 4.0, 50.0, 100.0); // Goes NE (different heading)
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg1, seg2]).unwrap();
        assert!(!curve.is_g1_contiguous());
    }

    #[test]
    fn test_piecewise_g_inverse_multi_segment() {
        // Two connected segments going east then north
        let seg1 = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let seg2 = make_line(100.0, 0.0, PI / 2.0, 50.0, 100.0);
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg1, seg2]).unwrap();

        // Point in first segment
        let p = curve.g_inverse(&Vector2::new(50.0, 0.0)).unwrap();
        assert_relative_eq!(p, 50.0, epsilon = 1.0);

        // Point at junction
        let p = curve.g_inverse(&Vector2::new(100.0, 0.0)).unwrap();
        assert_relative_eq!(p, 100.0, epsilon = 1.0);

        // Point in second segment
        let p = curve.g_inverse(&Vector2::new(100.0, 25.0)).unwrap();
        assert_relative_eq!(p, 125.0, epsilon = 1.0);
    }

    #[test]
    fn test_piecewise_heading_dot() {
        let seg = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg]).unwrap();

        // Lines have zero heading_dot
        assert_relative_eq!(curve.heading_dot(50.0).unwrap(), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn test_piecewise_three_segments() {
        // L-shaped path: east, then north, then east again
        let seg1 = make_line(0.0, 0.0, 0.0, 50.0, 0.0);
        let seg2 = make_line(50.0, 0.0, PI / 2.0, 50.0, 50.0);
        let seg3 = make_line(50.0, 50.0, 0.0, 50.0, 100.0);

        let curve =
            PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg1, seg2, seg3]).unwrap();

        assert_eq!(curve.num_segments(), 3);
        assert_relative_eq!(curve.arc_length(), 150.0);
        assert_relative_eq!(curve.p0(), 0.0);
        assert_relative_eq!(curve.p1(), 150.0);

        // Check position at end of each segment
        let pos = curve.g(50.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-6);

        let pos = curve.g(100.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 50.0, epsilon = 1e-6);

        let pos = curve.g(150.0).unwrap();
        assert_relative_eq!(pos.x, 100.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 50.0, epsilon = 1e-6);
    }

    #[test]
    fn test_piecewise_properties() {
        let seg = make_line(0.0, 0.0, 0.0, 100.0, 0.0);
        let curve = PiecewiseGroundCurve::from_segments(TOLERANCE, vec![seg]).unwrap();

        assert_relative_eq!(curve.linear_tolerance(), TOLERANCE);
        // p_from_xodr_p should return the same value
        assert_relative_eq!(curve.p_from_xodr_p(50.0).unwrap(), 50.0);
    }
}
