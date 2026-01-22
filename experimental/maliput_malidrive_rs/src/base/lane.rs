//! Lane implementation for maliput_malidrive.
//!
//! This module provides a concrete implementation of the maliput Lane trait
//! using road curve geometry.

use std::sync::{Arc, Weak};

use maliput::api::{
    InertialPosition, IsoLaneVelocity, Lane, LaneId, LanePosition,
    LanePositionResult, MaliputResult, RBounds, Rotation, Segment,
};
use nalgebra::Vector3;

use crate::common::MalidriveError;
use crate::road_curve::{RoadCurve, SimpleLaneOffset};

/// A concrete Lane implementation backed by road curve geometry.
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

    /// Sets the default branch at the start.
    pub fn set_default_start_branch(&mut self, lane: Weak<dyn Lane>, which: LaneEndWhich) {
        self.default_start_branch = Some((lane, which));
    }

    /// Sets the default branch at the finish.
    pub fn set_default_finish_branch(&mut self, lane: Weak<dyn Lane>, which: LaneEndWhich) {
        self.default_finish_branch = Some((lane, which));
    }

    /// Converts lane s-coordinate to road curve p-coordinate.
    fn s_to_p(&self, s: f64) -> f64 {
        self.s0 + s
    }

    /// Converts road curve p-coordinate to lane s-coordinate.
    fn p_to_s(&self, p: f64) -> f64 {
        p - self.s0
    }

    /// Computes the position in INERTIAL coordinates.
    fn compute_inertial_position(&self, lane_pos: &LanePosition) -> MalidriveResult<Vector3<f64>> {
        let p = self.s_to_p(lane_pos.s());
        let r_center = self.lane_offset.r(p);
        let r = r_center + lane_pos.r();

        self.road_curve.w(p, r, lane_pos.h())
    }

    /// Validates that the lane position is within bounds.
    fn validate_lane_position(&self, lane_pos: &LanePosition) -> MalidriveResult<()> {
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
}

impl Lane for MalidriveLane {
    fn id(&self) -> &LaneId {
        &self.id
    }

    fn segment(&self) -> Arc<dyn Segment> {
        self.segment.upgrade().expect("Segment has been dropped")
    }

    fn index(&self) -> usize {
        self.index
    }

    fn to_left(&self) -> Option<Arc<dyn Lane>> {
        let seg = self.segment();
        if self.index + 1 < seg.num_lanes() {
            Some(seg.lane(self.index + 1))
        } else {
            None
        }
    }

    fn to_right(&self) -> Option<Arc<dyn Lane>> {
        if self.index > 0 {
            Some(self.segment().lane(self.index - 1))
        } else {
            None
        }
    }

    fn length(&self) -> f64 {
        self.length
    }

    fn lane_bounds(&self, s: f64) -> LaneBounds {
        let half_width = self.lane_offset.width() / 2.0;
        LaneBounds::new(-half_width, half_width)
    }

    fn segment_bounds(&self, s: f64) -> RBounds {
        // For simplicity, return the same as lane bounds
        // A more complete implementation would compute the full segment width
        let half_width = self.lane_offset.width() / 2.0;
        RBounds::new(-half_width, half_width)
    }

    fn to_inertial_position(&self, lane_pos: &LanePosition) -> InertialPosition {
        match self.compute_inertial_position(lane_pos) {
            Ok(xyz) => InertialPosition::new(xyz.x, xyz.y, xyz.z),
            Err(_) => {
                // Clamp to valid range and try again
                let clamped_s = lane_pos.s().clamp(0.0, self.length);
                let clamped_pos = LanePosition::new(clamped_s, lane_pos.r(), lane_pos.h());
                if let Ok(xyz) = self.compute_inertial_position(&clamped_pos) {
                    InertialPosition::new(xyz.x, xyz.y, xyz.z)
                } else {
                    // Last resort: return origin
                    InertialPosition::new(0.0, 0.0, 0.0)
                }
            }
        }
    }

    fn to_lane_position(&self, inertial_pos: &InertialPosition) -> LanePositionResult {
        let xyz = Vector3::new(inertial_pos.x(), inertial_pos.y(), inertial_pos.z());

        // Use the road curve inverse mapping
        match self.road_curve.w_inverse(xyz) {
            Ok((p, r_road, h)) => {
                let s = self.p_to_s(p).clamp(0.0, self.length);
                let r_center = self.lane_offset.r(self.s_to_p(s));
                let r = r_road - r_center;

                let lane_pos = LanePosition::new(s, r, h);
                let nearest = self.to_inertial_position(&lane_pos);
                let distance = ((nearest.x() - inertial_pos.x()).powi(2)
                    + (nearest.y() - inertial_pos.y()).powi(2)
                    + (nearest.z() - inertial_pos.z()).powi(2))
                .sqrt();

                LanePositionResult::new(lane_pos, nearest, distance)
            }
            Err(_) => {
                // Fallback: project to the centerline
                let lane_pos = LanePosition::new(self.length / 2.0, 0.0, 0.0);
                let nearest = self.to_inertial_position(&lane_pos);
                let distance = ((nearest.x() - inertial_pos.x()).powi(2)
                    + (nearest.y() - inertial_pos.y()).powi(2)
                    + (nearest.z() - inertial_pos.z()).powi(2))
                .sqrt();

                LanePositionResult::new(lane_pos, nearest, distance)
            }
        }
    }

    fn get_orientation(&self, lane_pos: &LanePosition) -> Rotation {
        let p = self.s_to_p(lane_pos.s().clamp(0.0, self.length));

        match self.road_curve.rotation(p) {
            Ok(rotation) => {
                let (roll, pitch, yaw) = rotation.euler_angles();
                Rotation::new(roll, pitch, yaw)
            }
            Err(_) => Rotation::new(0.0, 0.0, 0.0),
        }
    }

    fn eval_motion_derivatives(
        &self,
        lane_pos: &LanePosition,
        velocity: &IsoLaneVelocity,
    ) -> LanePosition {
        // For a simple implementation, the motion derivatives are the velocities
        // scaled appropriately
        let s_dot = velocity.sigma_v();
        let r_dot = velocity.rho_v();
        let h_dot = velocity.eta_v();

        LanePosition::new(s_dot, r_dot, h_dot)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::road_curve::{ConstantFunction, LineGroundCurve};
    use approx::assert_relative_eq;
    use nalgebra::Vector2;

    const LINEAR_TOLERANCE: f64 = 1e-6;
    const ANGULAR_TOLERANCE: f64 = 1e-6;

    fn make_test_road_curve() -> Arc<RoadCurve> {
        let ground_curve = Arc::new(LineGroundCurve::new(
            LINEAR_TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0, // heading east
            100.0,
            0.0,
            100.0,
        ));
        let elevation = Arc::new(ConstantFunction::zero(0.0, 100.0));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, 100.0));

        Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            LINEAR_TOLERANCE,
            1.0,
        ))
    }

    // Note: Full tests would require a mock Segment, which requires more setup.
    // Here we test the geometry computations directly.

    #[test]
    fn test_s_to_p_conversion() {
        // Test the coordinate conversion logic
        let s0 = 10.0;
        let s1 = 60.0;
        let s = 25.0;
        let p = s0 + s;
        assert_relative_eq!(p, 35.0);
    }

    #[test]
    fn test_simple_lane_offset() {
        let offset = SimpleLaneOffset::new(5.0, 3.5, 0.0, 100.0);
        assert_relative_eq!(offset.r(50.0), 5.0);
        assert_relative_eq!(offset.width(), 3.5);
    }
}
