//! Segment implementation for maliput_malidrive.
//!
//! This module provides a Segment implementation that holds a road_curve::RoadCurve.
//! Child MalidriveLanes of this segment will be constructed as offsets of the
//! reference curve.

use std::sync::{Arc, Weak};

use maliput::api::{Junction, Lane, MaliputError, MaliputResult, Segment, SegmentId};

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{Function, RoadCurve};

/// A concrete Segment implementation for maliput_malidrive.
///
/// The Segment holds a reference to a RoadCurve and a reference line offset function.
/// The incidence region of the Segment on the road_curve is delimited by the range
/// composed by p0 and p1.
pub struct MalidriveSegment {
    /// Segment identifier.
    id: SegmentId,
    /// The junction containing this segment.
    junction: Weak<dyn Junction>,
    /// The lanes in this segment (ordered from right to left).
    lanes: Vec<Arc<dyn Lane>>,
    /// The reference curve of the Segment shared by all Lanes.
    road_curve: Arc<RoadCurve>,
    /// The road reference line offset function of the Segment shared by all Lanes.
    reference_line_offset: Arc<dyn Function>,
    /// The value of the p parameter of road_curve that matches the start of the Segment.
    p0: f64,
    /// The value of the p parameter of road_curve that matches the finish of the Segment.
    p1: f64,
    /// When omit_nondriveable_lanes is true the non-drivable lanes should be omitted
    /// but their creation is necessary to be consistent with other lanes.
    hidden_lanes: Vec<Arc<dyn Lane>>,
}

impl MalidriveSegment {
    /// Creates a new MalidriveSegment.
    ///
    /// The incidence region of the Segment on the road_curve will be
    /// delimited by the range composed by p0 and p1.
    ///
    /// # Arguments
    /// * `id` - The id of the Segment.
    /// * `junction` - Weak reference to the parent Junction.
    /// * `road_curve` - The reference curve of the Segment shared by all Lanes.
    /// * `reference_line_offset` - The road reference line offset function of the Segment shared by all Lanes.
    /// * `p0` - The value of the p parameter of road_curve that matches the start of the Segment.
    /// * `p1` - The value of the p parameter of road_curve that matches the finish of the Segment.
    ///
    /// # Errors
    /// Returns an error when:
    /// - `p1` is negative
    /// - `p0` is greater than `p1`
    pub fn new(
        id: SegmentId,
        junction: Weak<dyn Junction>,
        road_curve: Arc<RoadCurve>,
        reference_line_offset: Arc<dyn Function>,
        p0: f64,
        p1: f64,
    ) -> MalidriveResult<Self> {
        if p1 < 0.0 {
            return Err(MalidriveError::ValidationError(
                "p1 must be non-negative".to_string(),
            ));
        }
        if p0 > p1 {
            return Err(MalidriveError::ValidationError(format!(
                "p0 ({}) must be less than or equal to p1 ({})",
                p0, p1
            )));
        }

        Ok(Self {
            id,
            junction,
            lanes: Vec::new(),
            road_curve,
            reference_line_offset,
            p0,
            p1,
            hidden_lanes: Vec::new(),
        })
    }

    /// Returns the lower bound range of p.
    pub fn p0(&self) -> f64 {
        self.p0
    }

    /// Returns the upper bound range of p.
    pub fn p1(&self) -> f64 {
        self.p1
    }

    /// Returns the reference curve.
    pub fn road_curve(&self) -> &RoadCurve {
        self.road_curve.as_ref()
    }

    /// Returns the Arc to the reference curve.
    pub fn road_curve_arc(&self) -> Arc<RoadCurve> {
        self.road_curve.clone()
    }

    /// Returns the reference line offset function.
    pub fn reference_line_offset(&self) -> &dyn Function {
        self.reference_line_offset.as_ref()
    }

    /// Returns the Arc to the reference line offset function.
    pub fn reference_line_offset_arc(&self) -> Arc<dyn Function> {
        self.reference_line_offset.clone()
    }

    /// Adds a lane to the segment.
    ///
    /// If `hide_lane` is false, the lane is added to the visible lanes.
    /// If `hide_lane` is true, the segment becomes the lane's owner but the lane is hidden.
    /// This behavior is needed to correctly compute the offset of the lanes when
    /// the non-drivable lanes are omitted from the Road Network.
    ///
    /// Lanes should be added in order from right to left (increasing index).
    pub fn add_lane(&mut self, lane: Arc<dyn Lane>, hide_lane: bool) {
        if hide_lane {
            self.hidden_lanes.push(lane);
        } else {
            self.lanes.push(lane);
        }
    }

    /// Sets the lanes for this segment (visible lanes only).
    pub fn set_lanes(&mut self, lanes: Vec<Arc<dyn Lane>>) {
        self.lanes = lanes;
    }

    /// Returns the number of hidden lanes.
    pub fn num_hidden_lanes(&self) -> usize {
        self.hidden_lanes.len()
    }

    /// Sets the junction reference.
    pub fn set_junction(&mut self, junction: Weak<dyn Junction>) {
        self.junction = junction;
    }
}

impl Segment for MalidriveSegment {
    fn id(&self) -> &SegmentId {
        &self.id
    }

    fn junction(&self) -> Arc<dyn Junction> {
        self.junction.upgrade().expect("Junction has been dropped")
    }

    fn num_lanes(&self) -> usize {
        self.lanes.len()
    }

    fn lane(&self, index: usize) -> MaliputResult<Arc<dyn Lane>> {
        self.lanes
            .get(index)
            .cloned()
            .ok_or_else(|| MaliputError::IndexOutOfBounds {
                index,
                max: self.lanes.len().saturating_sub(1),
            })
    }
}

impl std::fmt::Debug for MalidriveSegment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MalidriveSegment")
            .field("id", &self.id)
            .field("p0", &self.p0)
            .field("p1", &self.p1)
            .field("num_lanes", &self.lanes.len())
            .field("num_hidden_lanes", &self.hidden_lanes.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::road_curve::{CubicPolynomial, LineGroundCurve};
    use nalgebra::Vector2;
    use std::f64::consts::SQRT_2;

    /// Test constants mirroring the C++ test.
    const K_P0: f64 = 0.0;
    const K_P1: f64 = 100.0;
    const K_LINEAR_TOLERANCE: f64 = 1e-13;
    const K_SCALE_LENGTH: f64 = 1.0;

    /// Creates a zero cubic polynomial (constant function at 0).
    fn make_zero_cubic_polynomial(p0: f64, p1: f64) -> Arc<dyn Function> {
        Arc::new(CubicPolynomial::new(0.0, 0.0, 0.0, 0.0, p0, p1))
    }

    /// Creates a test RoadCurve with a LineGroundCurve.
    fn make_test_road_curve(p0: f64, p1: f64) -> Arc<RoadCurve> {
        let xy0 = Vector2::new(10.0, 10.0);
        // Direction vector matching C++ test: (kP1 - kP0) * sqrt(2) / 2 for both x and y
        let dxy = Vector2::new(
            (p1 - p0) * SQRT_2 / 2.0,
            (p1 - p0) * SQRT_2 / 2.0,
        );

        let ground_curve = Arc::new(
            LineGroundCurve::new(K_LINEAR_TOLERANCE, xy0, dxy, p0, p1)
                .expect("Failed to create LineGroundCurve"),
        );

        let elevation = make_zero_cubic_polynomial(p0, p1);
        let superelevation = make_zero_cubic_polynomial(p0, p1);

        Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            K_LINEAR_TOLERANCE,
            K_SCALE_LENGTH,
        ))
    }

    /// Test that construction succeeds with valid parameters.
    /// Mirrors C++ test: SegmentTest.Constructor
    #[test]
    fn test_constructor() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let result = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve,
            reference_line_offset,
            K_P0,
            K_P1,
        );

        assert!(result.is_ok());
    }

    /// Test constructor assertions for invalid parameters.
    /// Mirrors C++ test: SegmentTest.ConstructorAssertions
    #[test]
    fn test_constructor_assertions() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        // Test: p1 < p0 should fail
        let wrong_p0 = 5.0;
        let wrong_p1 = 3.0;
        let result = MalidriveSegment::new(
            id.clone(),
            Weak::<MockJunction>::new(),
            road_curve.clone(),
            reference_line_offset.clone(),
            wrong_p0,
            wrong_p1,
        );
        assert!(result.is_err());
        if let Err(MalidriveError::ValidationError(msg)) = result {
            assert!(msg.contains("p0") && msg.contains("p1"));
        } else {
            panic!("Expected ValidationError");
        }

        // Test: negative p1 should fail
        let result = MalidriveSegment::new(
            id.clone(),
            Weak::<MockJunction>::new(),
            road_curve.clone(),
            reference_line_offset.clone(),
            0.0,
            -1.0,
        );
        assert!(result.is_err());
        if let Err(MalidriveError::ValidationError(msg)) = result {
            assert!(msg.contains("non-negative"));
        } else {
            panic!("Expected ValidationError");
        }
    }

    /// Test road_curve accessor.
    /// Mirrors C++ test: SegmentTest.RoadCurve
    #[test]
    fn test_road_curve() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let dut = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve.clone(),
            reference_line_offset,
            K_P0,
            K_P1,
        )
        .expect("Failed to create segment");

        // Verify road_curve is the same instance (comparing Arc pointers)
        assert!(Arc::ptr_eq(&road_curve, &dut.road_curve_arc()));
    }

    /// Test reference_line_offset accessor.
    /// Mirrors C++ test: SegmentTest.ReferenceLineOffset
    #[test]
    fn test_reference_line_offset() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let dut = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve,
            reference_line_offset.clone(),
            K_P0,
            K_P1,
        )
        .expect("Failed to create segment");

        // Verify reference_line_offset is the same instance (comparing Arc pointers)
        assert!(Arc::ptr_eq(&reference_line_offset, &dut.reference_line_offset_arc()));
    }

    /// Test p0 and p1 accessors.
    #[test]
    fn test_p0_p1_accessors() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let dut = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve,
            reference_line_offset,
            K_P0,
            K_P1,
        )
        .expect("Failed to create segment");

        assert!((dut.p0() - K_P0).abs() < f64::EPSILON);
        assert!((dut.p1() - K_P1).abs() < f64::EPSILON);
    }

    /// Test id accessor via Segment trait.
    #[test]
    fn test_id() {
        let id = SegmentId::new("test_segment".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let dut = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve,
            reference_line_offset,
            K_P0,
            K_P1,
        )
        .expect("Failed to create segment");

        assert_eq!(dut.id().string(), "test_segment");
    }

    /// Test add_lane with hide_lane = false.
    #[test]
    fn test_add_lane_visible() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let mut dut = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve,
            reference_line_offset,
            K_P0,
            K_P1,
        )
        .expect("Failed to create segment");

        assert_eq!(dut.num_lanes(), 0);
        assert_eq!(dut.num_hidden_lanes(), 0);

        // Add a mock lane as visible
        let mock_lane: Arc<dyn Lane> = Arc::new(MockLane::new("lane_1"));
        dut.add_lane(mock_lane, false);

        assert_eq!(dut.num_lanes(), 1);
        assert_eq!(dut.num_hidden_lanes(), 0);
    }

    /// Test add_lane with hide_lane = true.
    #[test]
    fn test_add_lane_hidden() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let mut dut = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve,
            reference_line_offset,
            K_P0,
            K_P1,
        )
        .expect("Failed to create segment");

        assert_eq!(dut.num_lanes(), 0);
        assert_eq!(dut.num_hidden_lanes(), 0);

        // Add a mock lane as hidden
        let mock_lane: Arc<dyn Lane> = Arc::new(MockLane::new("hidden_lane_1"));
        dut.add_lane(mock_lane, true);

        assert_eq!(dut.num_lanes(), 0);
        assert_eq!(dut.num_hidden_lanes(), 1);
    }

    /// Test lane accessor via Segment trait.
    #[test]
    fn test_lane_accessor() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let mut dut = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve,
            reference_line_offset,
            K_P0,
            K_P1,
        )
        .expect("Failed to create segment");

        let mock_lane: Arc<dyn Lane> = Arc::new(MockLane::new("lane_1"));
        dut.add_lane(mock_lane, false);

        // Access valid index
        let lane_result = dut.lane(0);
        assert!(lane_result.is_ok());
        let lane = lane_result.unwrap();
        assert_eq!(lane.id().string(), "lane_1");

        // Access invalid index
        let lane_result = dut.lane(1);
        assert!(lane_result.is_err());
    }

    /// Test Debug implementation.
    #[test]
    fn test_debug() {
        let id = SegmentId::new("dut".to_string());
        let road_curve = make_test_road_curve(K_P0, K_P1);
        let reference_line_offset = make_zero_cubic_polynomial(K_P0, K_P1);

        let dut = MalidriveSegment::new(
            id,
            Weak::<MockJunction>::new(),
            road_curve,
            reference_line_offset,
            K_P0,
            K_P1,
        )
        .expect("Failed to create segment");

        let debug_str = format!("{:?}", dut);
        assert!(debug_str.contains("MalidriveSegment"));
        assert!(debug_str.contains("dut"));
        assert!(debug_str.contains("p0"));
        assert!(debug_str.contains("p1"));
    }

    // ==================== Mock implementations for testing ====================

    /// Mock Junction for testing.
    #[derive(Debug)]
    struct MockJunction;

    impl Junction for MockJunction {
        fn id(&self) -> &maliput::api::JunctionId {
            unimplemented!("MockJunction::id not implemented")
        }

        fn road_geometry(&self) -> Arc<dyn maliput::api::RoadGeometry> {
            unimplemented!("MockJunction::road_geometry not implemented")
        }

        fn num_segments(&self) -> usize {
            0
        }

        fn segment(&self, _index: usize) -> MaliputResult<Arc<dyn Segment>> {
            unimplemented!("MockJunction::segment not implemented")
        }
    }

    /// Mock Lane for testing.
    #[derive(Debug)]
    struct MockLane {
        id: maliput::api::LaneId,
    }

    impl MockLane {
        fn new(id: &str) -> Self {
            Self {
                id: maliput::api::LaneId::new(id.to_string()),
            }
        }
    }

    impl Lane for MockLane {
        fn id(&self) -> &maliput::api::LaneId {
            &self.id
        }

        fn segment(&self) -> Arc<dyn Segment> {
            unimplemented!("MockLane::segment not implemented")
        }

        fn index(&self) -> usize {
            0
        }

        fn to_left(&self) -> Option<Arc<dyn Lane>> {
            None
        }

        fn to_right(&self) -> Option<Arc<dyn Lane>> {
            None
        }

        fn length(&self) -> f64 {
            100.0
        }

        fn lane_bounds(&self, _s: f64) -> MaliputResult<maliput::api::RBounds> {
            maliput::api::RBounds::new(-2.0, 2.0)
        }

        fn segment_bounds(&self, _s: f64) -> MaliputResult<maliput::api::RBounds> {
            maliput::api::RBounds::new(-2.0, 2.0)
        }

        fn elevation_bounds(&self, _s: f64, _r: f64) -> MaliputResult<maliput::api::HBounds> {
            maliput::api::HBounds::new(0.0, 5.0)
        }

        fn lane_type(&self) -> maliput::api::LaneType {
            maliput::api::LaneType::Driving
        }

        fn to_inertial_position(
            &self,
            _lane_pos: &maliput::api::LanePosition,
        ) -> MaliputResult<maliput::api::InertialPosition> {
            Ok(maliput::api::InertialPosition::new(0.0, 0.0, 0.0))
        }

        fn get_curvature(
            &self,
            _lane_pos: &maliput::api::LanePosition,
        ) -> MaliputResult<f64> {
            Ok(0.0)
        }

        fn to_lane_position(
            &self,
            _inertial_pos: &maliput::api::InertialPosition,
        ) -> MaliputResult<maliput::api::LanePositionResult> {
            Ok(maliput::api::LanePositionResult {
                lane_position: maliput::api::LanePosition::new(0.0, 0.0, 0.0),
                nearest_position: maliput::api::InertialPosition::new(0.0, 0.0, 0.0),
                distance: 0.0,
            })
        }

        fn to_segment_position(
            &self,
            _inertial_pos: &maliput::api::InertialPosition,
        ) -> MaliputResult<maliput::api::LanePositionResult> {
            Ok(maliput::api::LanePositionResult {
                lane_position: maliput::api::LanePosition::new(0.0, 0.0, 0.0),
                nearest_position: maliput::api::InertialPosition::new(0.0, 0.0, 0.0),
                distance: 0.0,
            })
        }

        fn get_orientation(
            &self,
            _lane_pos: &maliput::api::LanePosition,
        ) -> MaliputResult<maliput::api::Rotation> {
            Ok(maliput::api::Rotation::identity())
        }

        fn eval_motion_derivatives(
            &self,
            _lane_pos: &maliput::api::LanePosition,
            _velocity: &maliput::api::IsoLaneVelocity,
        ) -> MaliputResult<maliput::api::LanePosition> {
            Ok(maliput::api::LanePosition::new(0.0, 0.0, 0.0))
        }

        fn get_branch_point(&self, _which_end: maliput::api::LaneEndWhich) -> Arc<dyn maliput::api::BranchPoint> {
            unimplemented!("MockLane::get_branch_point not implemented")
        }

        fn get_confluent_branches(
            &self,
            _which_end: maliput::api::LaneEndWhich,
        ) -> Vec<maliput::api::LaneEnd> {
            Vec::new()
        }

        fn get_ongoing_branches(
            &self,
            _which_end: maliput::api::LaneEndWhich,
        ) -> Vec<maliput::api::LaneEnd> {
            Vec::new()
        }

        fn get_default_branch(
            &self,
            _which_end: maliput::api::LaneEndWhich,
        ) -> Option<maliput::api::LaneEnd> {
            None
        }

        fn contains(&self, _lane_position: &maliput::api::LanePosition) -> bool {
            true
        }
    }
}
