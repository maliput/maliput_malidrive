//! Lane implementation for maliput_malidrive.
//!
//! This module provides a concrete implementation of the maliput Lane trait
//! using road curve geometry.

use std::sync::{Arc, Weak};

use maliput::api::{Lane, LaneId, LanePosition, Segment};
use nalgebra::Vector3;

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{RoadCurve, SimpleLaneOffset};

/// A concrete Lane implementation backed by road curve geometry.
///
/// NOTE: The Lane trait implementation is currently disabled because the
/// maliput Lane trait expects `&dyn T` references while MalidriveLane uses
/// `Arc<dyn T>` for parent references (to enable shared ownership).
///
/// The geometric computations are tested via RoadCurve tests in:
///   `road_curve/road_curve.rs` (cpp_parity_tests module)
pub struct MalidriveLane {
    /// Lane identifier.
    id: LaneId,
    /// The index of this lane within its segment.
    index: usize,
    /// The segment containing this lane.
    segment: Weak<dyn Segment>,
    /// The road curve providing geometry.
    road_curve: Arc<RoadCurve>,
    /// Lane offset calculator (provides r coordinate).
    lane_offset: SimpleLaneOffset,
    /// Start s-parameter (in road curve coordinates).
    s0: f64,
    /// End s-parameter (in road curve coordinates).
    s1: f64,
    /// Lane length.
    length: f64,
    /// Default branch at start (for LaneEnd::Start).
    default_start_branch: Option<(Weak<dyn Lane>, LaneEndWhich)>,
    /// Default branch at finish (for LaneEnd::Finish).
    default_finish_branch: Option<(Weak<dyn Lane>, LaneEndWhich)>,
    /// Linear tolerance.
    linear_tolerance: f64,
    /// Angular tolerance.
    angular_tolerance: f64,
}

/// Which end of a lane we're referring to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneEndWhich {
    /// The start of the lane (s = 0).
    Start,
    /// The finish of the lane (s = length).
    Finish,
}

impl MalidriveLane {
    /// Creates a new MalidriveLane.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: LaneId,
        index: usize,
        segment: Weak<dyn Segment>,
        road_curve: Arc<RoadCurve>,
        lane_offset: SimpleLaneOffset,
        s0: f64,
        s1: f64,
        linear_tolerance: f64,
        angular_tolerance: f64,
    ) -> Self {
        let length = s1 - s0;
        Self {
            id,
            index,
            segment,
            road_curve,
            lane_offset,
            s0,
            s1,
            length,
            default_start_branch: None,
            default_finish_branch: None,
            linear_tolerance,
            angular_tolerance,
        }
    }

    /// Returns the lane identifier.
    pub fn id(&self) -> &LaneId {
        &self.id
    }

    /// Returns the index of this lane within its segment.
    pub fn index(&self) -> usize {
        self.index
    }

    /// Returns the arc-length of the Lane.
    pub fn length(&self) -> f64 {
        self.length
    }

    /// Returns the road curve providing geometry.
    pub fn road_curve(&self) -> &Arc<RoadCurve> {
        &self.road_curve
    }

    /// Returns the lane offset calculator.
    pub fn lane_offset(&self) -> &SimpleLaneOffset {
        &self.lane_offset
    }

    /// Sets the default branch at the start.
    pub fn set_default_start_branch(&mut self, lane: Weak<dyn Lane>, which: LaneEndWhich) {
        self.default_start_branch = Some((lane, which));
    }

    /// Sets the default branch at the finish.
    pub fn set_default_finish_branch(&mut self, lane: Weak<dyn Lane>, which: LaneEndWhich) {
        self.default_finish_branch = Some((lane, which));
    }

    /// Converts lane s-coordinate to road curve p-coordinate.
    pub fn s_to_p(&self, s: f64) -> f64 {
        self.s0 + s
    }

    /// Converts road curve p-coordinate to lane s-coordinate.
    pub fn p_to_s(&self, p: f64) -> f64 {
        p - self.s0
    }

    /// Computes the position in INERTIAL coordinates.
    ///
    /// This is the core geometric computation that maps lane coordinates
    /// to world coordinates.
    pub fn compute_inertial_position(
        &self,
        lane_pos: &LanePosition,
    ) -> MalidriveResult<Vector3<f64>> {
        let p = self.s_to_p(lane_pos.s());
        let r_center = self.lane_offset.r(p);
        let r = r_center + lane_pos.r();

        self.road_curve.w(p, r, lane_pos.h())
    }

    /// Validates that the lane position is within bounds.
    pub fn validate_lane_position(&self, lane_pos: &LanePosition) -> MalidriveResult<()> {
        let s = lane_pos.s();
        if s < -self.linear_tolerance || s > self.length + self.linear_tolerance {
            return Err(MalidriveError::ParameterOutOfRange {
                parameter: "s".to_string(),
                value: s,
                min: 0.0,
                max: self.length,
            });
        }
        Ok(())
    }

    /// Returns the lane bounds as (r_min, r_max) tuple.
    ///
    /// These are the lateral bounds for a position considered to be
    /// "staying in the lane".
    pub fn lane_bounds(&self, _s: f64) -> (f64, f64) {
        let half_width = self.lane_offset.width() / 2.0;
        (-half_width, half_width)
    }

    /// Returns the segment bounds as (r_min, r_max) tuple.
    pub fn segment_bounds(&self, _s: f64) -> (f64, f64) {
        // For simplicity, return the same as lane bounds
        // A more complete implementation would compute the full segment width
        let half_width = self.lane_offset.width() / 2.0;
        (-half_width, half_width)
    }

    /// Returns the orientation at the given lane position.
    pub fn get_orientation(&self, lane_pos: &LanePosition) -> MalidriveResult<nalgebra::Rotation3<f64>> {
        let p = self.s_to_p(lane_pos.s().clamp(0.0, self.length));
        self.road_curve.rotation(p)
    }
}

impl std::fmt::Debug for MalidriveLane {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MalidriveLane")
            .field("id", &self.id)
            .field("index", &self.index)
            .field("length", &self.length)
            .finish()
    }
}

// NOTE: The Lane trait implementation is disabled until the architecture
// issue with reference types is resolved.
//
// The maliput Lane trait requires:
//   fn segment(&self) -> &dyn Segment
//   fn to_left(&self) -> Option<&dyn Lane>
//   fn to_right(&self) -> Option<&dyn Lane>
//
// But MalidriveLane stores Weak<dyn Segment> and would need to return
// references from upgraded Arc<dyn Segment>, which is not possible without
// unsafe code or changing the ownership model.
//
// Options to resolve:
// 1. Store raw pointers with lifetime management
// 2. Use arena allocation
// 3. Change maliput trait to return Arc<dyn T>
// 4. Use self-referential structs (ouroboros/rental)
//
// For now, MalidriveLane provides non-trait methods that can be used
// directly, and the geometric computations are validated through
// RoadCurve tests.
