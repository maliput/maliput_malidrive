//! Lane implementation for maliput_malidrive.
//!
//! This module provides a concrete implementation of the maliput Lane trait
//! using road curve geometry.

use std::sync::{Arc, RwLock, Weak};

use maliput::api::{
    BranchPoint, HBounds, InertialPosition, IsoLaneVelocity, Lane, LaneEnd, LaneEndWhich, LaneId,
    LanePosition, LanePositionResult, LaneType, MaliputError, MaliputResult, RBounds, Rotation,
    Segment,
};
use nalgebra::Vector3;

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{Function, RoadCurve};

/// A concrete Lane implementation backed by road curve geometry.
///
/// The geometric computations are based on the C++ implementation in
/// `maliput_malidrive/base/lane.cc`.
pub struct MalidriveLane {
    /// Lane identifier.
    id: LaneId,
    /// The index of this lane within its segment.
    index: usize,
    /// The segment containing this lane.
    segment: Weak<dyn Segment>,
    /// The road curve providing geometry.
    road_curve: Arc<RoadCurve>,
    /// Lane width function.
    lane_width: Arc<dyn Function>,
    /// Lane offset function (r-coordinate of lane centerline in reference frame).
    lane_offset: Arc<dyn Function>,
    /// Start p-parameter (in road curve coordinates).
    p0: f64,
    /// End p-parameter (in road curve coordinates).
    p1: f64,
    /// Lane length.
    length: f64,
    /// Elevation bounds.
    elevation_bounds: HBounds,
    /// Lane type.
    lane_type: LaneType,
    /// OpenDRIVE track ID.
    xodr_track: i32,
    /// OpenDRIVE lane ID.
    xodr_lane_id: i32,
    /// Linear tolerance.
    linear_tolerance: f64,
    /// Angular tolerance.
    #[allow(dead_code)]
    angular_tolerance: f64,
    /// Adjacent lane to the left (stored as Weak to avoid cycles).
    to_left: RwLock<Option<Weak<dyn Lane>>>,
    /// Adjacent lane to the right (stored as Weak to avoid cycles).
    to_right: RwLock<Option<Weak<dyn Lane>>>,
    /// Branch point at start.
    start_branch_point: RwLock<Option<Weak<dyn BranchPoint>>>,
    /// Branch point at finish.
    finish_branch_point: RwLock<Option<Weak<dyn BranchPoint>>>,
    /// Default branch at start.
    default_start_branch: RwLock<Option<LaneEnd>>,
    /// Default branch at finish.
    default_finish_branch: RwLock<Option<LaneEnd>>,
}

impl MalidriveLane {
    /// Creates a new MalidriveLane.
    ///
    /// # Arguments
    /// * `id` - Lane identifier.
    /// * `index` - Index within parent segment.
    /// * `segment` - Weak reference to parent segment.
    /// * `road_curve` - The road curve providing geometry.
    /// * `lane_width` - Function describing lane width vs p.
    /// * `lane_offset` - Function describing lane center offset vs p.
    /// * `p0` - Start parameter.
    /// * `p1` - End parameter.
    /// * `elevation_bounds` - Height bounds.
    /// * `lane_type` - Lane classification.
    /// * `xodr_track` - OpenDRIVE track ID.
    /// * `xodr_lane_id` - OpenDRIVE lane ID.
    /// * `linear_tolerance` - Linear tolerance.
    /// * `angular_tolerance` - Angular tolerance.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: LaneId,
        index: usize,
        segment: Weak<dyn Segment>,
        road_curve: Arc<RoadCurve>,
        lane_width: Arc<dyn Function>,
        lane_offset: Arc<dyn Function>,
        p0: f64,
        p1: f64,
        elevation_bounds: HBounds,
        lane_type: LaneType,
        xodr_track: i32,
        xodr_lane_id: i32,
        linear_tolerance: f64,
        angular_tolerance: f64,
    ) -> MalidriveResult<Self> {
        // Validate inputs
        if xodr_track < 0 {
            return Err(MalidriveError::ValidationError(
                "xodr_track must be non-negative".to_string(),
            ));
        }

        // Compute length based on road curve offset
        // For now, use the p-range as a simple approximation
        // A full implementation would use RoadCurveOffset for arc-length integration
        let length = p1 - p0;

        Ok(Self {
            id,
            index,
            segment,
            road_curve,
            lane_width,
            lane_offset,
            p0,
            p1,
            length,
            elevation_bounds,
            lane_type,
            xodr_track,
            xodr_lane_id,
            linear_tolerance,
            angular_tolerance,
            to_left: RwLock::new(None),
            to_right: RwLock::new(None),
            start_branch_point: RwLock::new(None),
            finish_branch_point: RwLock::new(None),
            default_start_branch: RwLock::new(None),
            default_finish_branch: RwLock::new(None),
        })
    }

    /// Returns the OpenDRIVE track ID.
    pub fn get_track(&self) -> i32 {
        self.xodr_track
    }

    /// Returns the OpenDRIVE lane ID.
    pub fn get_lane_id(&self) -> i32 {
        self.xodr_lane_id
    }

    /// Returns the TRACK Frame start `s` coordinate.
    pub fn get_track_s_start(&self) -> f64 {
        self.p0
    }

    /// Returns the TRACK Frame end `s` coordinate.
    pub fn get_track_s_end(&self) -> f64 {
        self.p1
    }

    /// Sets the adjacent lane to the left.
    pub fn set_to_left(&self, lane: Weak<dyn Lane>) {
        *self.to_left.write().unwrap() = Some(lane);
    }

    /// Sets the adjacent lane to the right.
    pub fn set_to_right(&self, lane: Weak<dyn Lane>) {
        *self.to_right.write().unwrap() = Some(lane);
    }

    /// Sets the branch point at the start.
    pub fn set_start_branch_point(&self, bp: Weak<dyn BranchPoint>) {
        *self.start_branch_point.write().unwrap() = Some(bp);
    }

    /// Sets the branch point at the finish.
    pub fn set_finish_branch_point(&self, bp: Weak<dyn BranchPoint>) {
        *self.finish_branch_point.write().unwrap() = Some(bp);
    }

    /// Sets the default branch at the start.
    pub fn set_default_start_branch(&self, lane_end: LaneEnd) {
        *self.default_start_branch.write().unwrap() = Some(lane_end);
    }

    /// Sets the default branch at the finish.
    pub fn set_default_finish_branch(&self, lane_end: LaneEnd) {
        *self.default_finish_branch.write().unwrap() = Some(lane_end);
    }

    /// Returns the road curve.
    pub fn road_curve(&self) -> &Arc<RoadCurve> {
        &self.road_curve
    }

    /// Returns the lane width function.
    pub fn lane_width_function(&self) -> &Arc<dyn Function> {
        &self.lane_width
    }

    /// Returns the lane offset function.
    pub fn lane_offset_function(&self) -> &Arc<dyn Function> {
        &self.lane_offset
    }

    /// Converts lane s-coordinate to road curve p-coordinate.
    ///
    /// For a full implementation, this would use RoadCurveOffset.
    /// For now, we use a simple linear mapping.
    pub fn p_from_s(&self, s: f64) -> f64 {
        self.p0 + s
    }

    /// Converts road curve p-coordinate to lane s-coordinate.
    pub fn s_from_p(&self, p: f64) -> f64 {
        p - self.p0
    }

    /// Converts `(p, r)` in LANE Frame to r-coordinate in road_curve Frame.
    pub fn to_reference_r(&self, p: f64, r: f64) -> MalidriveResult<f64> {
        let lane_center_r = self.lane_offset.f(p)?;
        Ok(lane_center_r + r)
    }

    /// Converts `(p, r)` in road_curve Frame to r-coordinate in LANE Frame.
    #[allow(dead_code)]
    pub fn to_lane_r(&self, p: f64, r: f64) -> MalidriveResult<f64> {
        let lane_center_r = self.lane_offset.f(p)?;
        Ok(r - lane_center_r)
    }

    /// Returns the lane width at parameter p.
    #[allow(dead_code)]
    pub fn lane_width_at(&self, p: f64) -> MalidriveResult<f64> {
        self.lane_width.f(p)
    }

    /// Converts lane_s coordinate to track_s coordinate.
    pub fn track_s_from_lane_s(&self, lane_s: f64) -> f64 {
        self.p_from_s(lane_s)
    }

    /// Converts track_s coordinate to lane_s coordinate.
    pub fn lane_s_from_track_s(&self, track_s: f64) -> f64 {
        self.s_from_p(track_s)
    }

    /// Validates the s-coordinate is within range.
    pub fn validate_s(&self, s: f64) -> MalidriveResult<f64> {
        if s < -self.linear_tolerance || s > self.length + self.linear_tolerance {
            return Err(MalidriveError::ParameterOutOfRange {
                parameter: "s".to_string(),
                value: s,
                min: 0.0,
                max: self.length,
            });
        }
        // Clamp to valid range
        Ok(s.clamp(0.0, self.length))
    }

    /// Computes the backend (inertial) position from lane position.
    fn do_to_backend_position(&self, lane_pos: &LanePosition) -> MalidriveResult<Vector3<f64>> {
        let s = self.validate_s(lane_pos.s())?;
        let p = self.p_from_s(s);
        let r = self.to_reference_r(p, lane_pos.r())?;
        self.road_curve.w(p, r, lane_pos.h())
    }

    /// Converts backend (inertial) position to lane frame coordinates.
    ///
    /// Uses Newton's method iteration as in the C++ implementation.
    fn backend_frame_to_lane_frame(&self, xyz: &Vector3<f64>) -> MalidriveResult<Vector3<f64>> {
        // Get initial estimate of p from the RoadCurve
        let (initial_p, _r, _h) = self.road_curve.w_inverse(*xyz)?;
        let mut p = initial_p;

        // Delta p, to be reduced iteratively
        let mut dp = 2.0 * self.linear_tolerance;

        const MAX_ITERATIONS: i32 = 16;

        // Newton's method iteration
        for _ in 0..MAX_ITERATIONS {
            if dp.abs() <= self.linear_tolerance {
                break;
            }

            // Clamp p to lane's range
            p = p.clamp(self.p0, self.p1);

            // Get the position in INERTIAL Frame at the centerline
            let lane_center_r = self.lane_offset.f(p)?;
            let w_p = self.road_curve.w(p, lane_center_r, 0.0)?;

            // Get the vector difference
            let w_delta = xyz - w_p;

            // Compute the centerline derivative with respect to p
            let w_dot = self.road_curve.w_dot(p, lane_center_r, 0.0)?;

            // Newton's method update
            let dot_product = w_dot.dot(&w_dot);
            if dot_product > 1e-15 {
                dp = w_delta.dot(&w_dot) / dot_product;
                p += dp;
            } else {
                break;
            }
        }

        // Clamp final p
        p = p.clamp(self.p0, self.p1);

        // Compute final result
        let w_p = self.road_curve.w(p, 0.0, 0.0)?;
        let w_delta = xyz - w_p;

        // Compute orthonormal basis at p
        let s_hat = self.road_curve.s_hat(p, 0.0, 0.0)?;
        let h_hat = self.road_curve.h_hat(p, &s_hat)?;
        let r_hat = h_hat.cross(&s_hat);

        let lane_center_r = self.lane_offset.f(p)?;
        let r = r_hat.dot(&w_delta) - lane_center_r;
        let h = h_hat.dot(&w_delta);

        Ok(Vector3::new(p, r, h))
    }

    /// Converts inertial position to lane/segment position.
    fn inertial_to_lane_segment_position(
        &self,
        use_lane_boundaries: bool,
        inertial_pos: &InertialPosition,
    ) -> MalidriveResult<LanePositionResult> {
        let backend_pos = Vector3::new(inertial_pos.x(), inertial_pos.y(), inertial_pos.z());

        let unconstrained_prh = self.backend_frame_to_lane_frame(&backend_pos)?;

        // Convert p to s
        let p = unconstrained_prh.x.clamp(self.p0, self.p1);
        let s = self.s_from_p(p);

        // Get r bounds
        let r_bounds = if use_lane_boundaries {
            self.do_lane_bounds(s)?
        } else {
            self.do_segment_bounds(s)?
        };

        let r = unconstrained_prh.y.clamp(r_bounds.min(), r_bounds.max());

        // Get h bounds
        let h_bounds = self.elevation_bounds;
        let h = unconstrained_prh.z.clamp(h_bounds.min(), h_bounds.max());

        let lane_position = LanePosition::new(s, r, h);
        let nearest_backend = self.do_to_backend_position(&lane_position)?;
        let nearest_position =
            InertialPosition::new(nearest_backend.x, nearest_backend.y, nearest_backend.z);

        let distance = ((backend_pos.x - nearest_backend.x).powi(2)
            + (backend_pos.y - nearest_backend.y).powi(2)
            + (backend_pos.z - nearest_backend.z).powi(2))
        .sqrt();

        Ok(LanePositionResult {
            lane_position,
            nearest_position,
            distance,
        })
    }

    /// Computes lane bounds at s.
    fn do_lane_bounds(&self, s: f64) -> MalidriveResult<RBounds> {
        let s = self.validate_s(s)?;
        let p = self.p_from_s(s);

        // Lane width function is a cubic polynomial and negative values are possible,
        // but negative widths are clamped to zero.
        let width = self.lane_width.f(p)?.max(0.0);
        let half_width = width / 2.0;

        RBounds::new(-half_width, half_width)
            .map_err(|e| MalidriveError::ValidationError(e.to_string()))
    }

    /// Computes segment bounds at s.
    fn do_segment_bounds(&self, s: f64) -> MalidriveResult<RBounds> {
        let s = self.validate_s(s)?;
        let lane_bounds = self.do_lane_bounds(s)?;

        let mut bound_left = lane_bounds.max();
        let mut bound_right = -lane_bounds.min();

        // Accumulate widths from lanes to the left
        let to_left_guard = self.to_left.read().unwrap();
        if let Some(ref weak_left) = *to_left_guard {
            if let Some(left_lane) = weak_left.upgrade() {
                // For segment bounds, we'd traverse all adjacent lanes
                // This is a simplified version - full implementation would traverse
                if let Ok(left_bounds) = left_lane.lane_bounds(s) {
                    bound_left += left_bounds.width();
                }
            }
        }

        // Accumulate widths from lanes to the right
        let to_right_guard = self.to_right.read().unwrap();
        if let Some(ref weak_right) = *to_right_guard {
            if let Some(right_lane) = weak_right.upgrade() {
                if let Ok(right_bounds) = right_lane.lane_bounds(s) {
                    bound_right += right_bounds.width();
                }
            }
        }

        // Ensure minimum bounds
        let tolerance = self.linear_tolerance;
        bound_left = bound_left.max(tolerance);
        bound_right = bound_right.max(tolerance);

        RBounds::new(-bound_right, bound_left)
            .map_err(|e| MalidriveError::ValidationError(e.to_string()))
    }

    /// Computes the curvature at the given lane position.
    ///
    /// Uses finite differences on the tangent vector as in the C++ implementation.
    fn do_get_curvature(&self, lane_pos: &LanePosition) -> MalidriveResult<f64> {
        let s = self.validate_s(lane_pos.s())?;
        let p = self.p_from_s(s);
        let r = lane_pos.r();
        let h = lane_pos.h();
        let r_total = self.to_reference_r(p, r)?;

        let delta = self.linear_tolerance;

        // Compute p values for finite difference
        let p_minus = self.p0.max(p - delta);
        let p_plus = self.p1.min(p + delta);
        let dp = p_plus - p_minus;

        // Guard against very short lanes
        if dp < delta / 10.0 {
            return Ok(0.0);
        }

        // Get tangent vectors at the two points
        let tangent_minus = self.road_curve.w_dot(p_minus, r_total, h)?;
        let tangent_plus = self.road_curve.w_dot(p_plus, r_total, h)?;

        let norm_minus = tangent_minus.norm();
        let norm_plus = tangent_plus.norm();

        if norm_minus < delta || norm_plus < delta {
            return Ok(0.0);
        }

        // Compute unit tangent vectors
        let t_minus = tangent_minus / norm_minus;
        let t_plus = tangent_plus / norm_plus;

        // Compute dT/dp
        let dt_dp = (t_plus - t_minus) / dp;

        // Convert to dT/ds
        let ds_dp = (norm_minus + norm_plus) / 2.0;

        // Curvature magnitude
        let curvature_magnitude = dt_dp.norm() / ds_dp;

        // Compute sign
        let t_center = (t_minus + t_plus) / 2.0;
        let cross = t_center.cross(&dt_dp);

        let s_hat = self.road_curve.s_hat(p, r_total, h)?;
        let h_hat = self.road_curve.h_hat(p, &s_hat)?;

        let sign = if cross.dot(&h_hat) >= 0.0 { 1.0 } else { -1.0 };

        Ok(sign * curvature_magnitude)
    }

    /// Computes the orientation at the given lane position.
    fn do_get_orientation(&self, lane_pos: &LanePosition) -> MalidriveResult<Rotation> {
        let s = self.validate_s(lane_pos.s())?;
        let p = self.p_from_s(s);

        let (roll, pitch, yaw) = self.road_curve.orientation(p)?;

        Ok(Rotation::from_rpy(roll, pitch, yaw))
    }

    /// Evaluates motion derivatives.
    fn do_eval_motion_derivatives(
        &self,
        position: &LanePosition,
        velocity: &IsoLaneVelocity,
    ) -> MalidriveResult<LanePosition> {
        let s = self.validate_s(position.s())?;
        let p = self.p_from_s(s);
        let r = self.to_reference_r(p, position.r())?;
        let h = position.h();

        // Compute ds/dsigma
        let w_dot_center = self.road_curve.w_dot(p, self.lane_offset.f(p)?, 0.0)?;
        let w_dot = self.road_curve.w_dot(p, r, h)?;

        let ds_dsigma = w_dot_center.norm() / w_dot.norm();

        Ok(LanePosition::new(
            ds_dsigma * velocity.sigma_v,
            velocity.rho_v,
            velocity.eta_v,
        ))
    }
}

impl std::fmt::Debug for MalidriveLane {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MalidriveLane")
            .field("id", &self.id)
            .field("index", &self.index)
            .field("length", &self.length)
            .field("p0", &self.p0)
            .field("p1", &self.p1)
            .field("lane_type", &self.lane_type)
            .finish()
    }
}

// Implement Send + Sync manually because of RwLock fields
unsafe impl Send for MalidriveLane {}
unsafe impl Sync for MalidriveLane {}

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
        self.to_left
            .read()
            .unwrap()
            .as_ref()
            .and_then(|w| w.upgrade())
    }

    fn to_right(&self) -> Option<Arc<dyn Lane>> {
        self.to_right
            .read()
            .unwrap()
            .as_ref()
            .and_then(|w| w.upgrade())
    }

    fn length(&self) -> f64 {
        self.length
    }

    fn lane_bounds(&self, s: f64) -> MaliputResult<RBounds> {
        self.do_lane_bounds(s)
            .map_err(|e| MaliputError::Geometric(e.to_string()))
    }

    fn segment_bounds(&self, s: f64) -> MaliputResult<RBounds> {
        self.do_segment_bounds(s)
            .map_err(|e| MaliputError::Geometric(e.to_string()))
    }

    fn elevation_bounds(&self, _s: f64, _r: f64) -> MaliputResult<HBounds> {
        Ok(self.elevation_bounds)
    }

    fn lane_type(&self) -> LaneType {
        self.lane_type
    }

    fn to_inertial_position(&self, lane_pos: &LanePosition) -> MaliputResult<InertialPosition> {
        let backend_pos = self
            .do_to_backend_position(lane_pos)
            .map_err(|e| MaliputError::Geometric(e.to_string()))?;
        Ok(InertialPosition::new(
            backend_pos.x,
            backend_pos.y,
            backend_pos.z,
        ))
    }

    fn get_curvature(&self, lane_pos: &LanePosition) -> MaliputResult<f64> {
        self.do_get_curvature(lane_pos)
            .map_err(|e| MaliputError::Geometric(e.to_string()))
    }

    fn to_lane_position(&self, inertial_pos: &InertialPosition) -> MaliputResult<LanePositionResult> {
        self.inertial_to_lane_segment_position(true, inertial_pos)
            .map_err(|e| MaliputError::Geometric(e.to_string()))
    }

    fn to_segment_position(
        &self,
        inertial_pos: &InertialPosition,
    ) -> MaliputResult<LanePositionResult> {
        self.inertial_to_lane_segment_position(false, inertial_pos)
            .map_err(|e| MaliputError::Geometric(e.to_string()))
    }

    fn get_orientation(&self, lane_pos: &LanePosition) -> MaliputResult<Rotation> {
        self.do_get_orientation(lane_pos)
            .map_err(|e| MaliputError::Geometric(e.to_string()))
    }

    fn eval_motion_derivatives(
        &self,
        position: &LanePosition,
        velocity: &IsoLaneVelocity,
    ) -> MaliputResult<LanePosition> {
        self.do_eval_motion_derivatives(position, velocity)
            .map_err(|e| MaliputError::Geometric(e.to_string()))
    }

    fn get_branch_point(&self, which_end: LaneEndWhich) -> Arc<dyn BranchPoint> {
        let bp = match which_end {
            LaneEndWhich::Start => self.start_branch_point.read().unwrap(),
            LaneEndWhich::Finish => self.finish_branch_point.read().unwrap(),
        };
        bp.as_ref()
            .and_then(|w| w.upgrade())
            .expect("BranchPoint has been dropped")
    }

    fn get_confluent_branches(&self, which_end: LaneEndWhich) -> Vec<LaneEnd> {
        let bp = self.get_branch_point(which_end);
        // Get the side that contains this lane
        let a_side = bp.get_a_side();
        let b_side = bp.get_b_side();

        // Check which side this lane is on
        for i in 0..a_side.size() {
            if let Ok(lane_end) = a_side.get(i) {
                if lane_end.lane.id() == &self.id {
                    // This lane is on the A side, return all A side ends
                    let mut result = Vec::new();
                    for j in 0..a_side.size() {
                        if let Ok(end) = a_side.get(j) {
                            result.push(end);
                        }
                    }
                    return result;
                }
            }
        }

        // Must be on B side
        let mut result = Vec::new();
        for j in 0..b_side.size() {
            if let Ok(end) = b_side.get(j) {
                result.push(end);
            }
        }
        result
    }

    fn get_ongoing_branches(&self, which_end: LaneEndWhich) -> Vec<LaneEnd> {
        let bp = self.get_branch_point(which_end);
        let a_side = bp.get_a_side();
        let b_side = bp.get_b_side();

        // Check which side this lane is on, return the opposite
        for i in 0..a_side.size() {
            if let Ok(lane_end) = a_side.get(i) {
                if lane_end.lane.id() == &self.id {
                    // This lane is on the A side, return B side
                    let mut result = Vec::new();
                    for j in 0..b_side.size() {
                        if let Ok(end) = b_side.get(j) {
                            result.push(end);
                        }
                    }
                    return result;
                }
            }
        }

        // Must be on B side, return A side
        let mut result = Vec::new();
        for j in 0..a_side.size() {
            if let Ok(end) = a_side.get(j) {
                result.push(end);
            }
        }
        result
    }

    fn get_default_branch(&self, which_end: LaneEndWhich) -> Option<LaneEnd> {
        match which_end {
            LaneEndWhich::Start => self.default_start_branch.read().unwrap().clone(),
            LaneEndWhich::Finish => self.default_finish_branch.read().unwrap().clone(),
        }
    }

    fn contains(&self, lane_position: &LanePosition) -> bool {
        let s = lane_position.s();
        let r = lane_position.r();
        let h = lane_position.h();

        // Check s bounds
        if s < 0.0 || s > self.length {
            return false;
        }

        // Check r bounds
        if let Ok(r_bounds) = self.lane_bounds(s) {
            if r < r_bounds.min() || r > r_bounds.max() {
                return false;
            }
        } else {
            return false;
        }

        // Check h bounds
        if h < self.elevation_bounds.min() || h > self.elevation_bounds.max() {
            return false;
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::road_curve::{ConstantFunction, GroundCurve, LineGroundCurve};
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    // Test constants matching the C++ test
    const K_LINEAR_TOLERANCE: f64 = 1e-11;
    const K_ANGULAR_TOLERANCE: f64 = 1e-6;
    const K_SCALE_LENGTH: f64 = 1.0;
    const K_P0: f64 = 0.0;
    const K_P1: f64 = 100.0;
    const K_WIDTH: f64 = 5.0;
    const K_LANE_OFFSET: f64 = 10.0;

    // Expected positions for a 45-degree line from (10, 12) with length 100*sqrt(2)
    const K_XY0: (f64, f64) = (10.0, 12.0);

    /// Creates a test road curve with a flat line geometry.
    fn create_test_road_curve() -> Arc<RoadCurve> {
        let dxy = (
            (K_P1 - K_P0) * std::f64::consts::FRAC_1_SQRT_2,
            (K_P1 - K_P0) * std::f64::consts::FRAC_1_SQRT_2,
        );

        let ground_curve: Arc<dyn GroundCurve> = Arc::new(
            LineGroundCurve::new(
                K_LINEAR_TOLERANCE,
                nalgebra::Vector2::new(K_XY0.0, K_XY0.1),
                nalgebra::Vector2::new(dxy.0, dxy.1),
                K_P0,
                K_P1,
            )
            .expect("Failed to create LineGroundCurve"),
        );

        let elevation: Arc<dyn Function> =
            Arc::new(ConstantFunction::new(0.0, K_P0, K_P1));
        let superelevation: Arc<dyn Function> =
            Arc::new(ConstantFunction::new(0.0, K_P0, K_P1));

        Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            K_LINEAR_TOLERANCE,
            K_SCALE_LENGTH,
        ))
    }

    /// Creates a test lane.
    fn create_test_lane(road_curve: Arc<RoadCurve>) -> MalidriveLane {
        let lane_width: Arc<dyn Function> = Arc::new(ConstantFunction::new(K_WIDTH, K_P0, K_P1));
        let lane_offset: Arc<dyn Function> =
            Arc::new(ConstantFunction::new(K_LANE_OFFSET, K_P0, K_P1));
        let elevation_bounds = HBounds::new(0.0, 5.0).unwrap();

        MalidriveLane::new(
            LaneId::new("test_lane".to_string()),
            0,
            Weak::<MockSegment>::new() as Weak<dyn Segment>,
            road_curve,
            lane_width,
            lane_offset,
            K_P0,
            K_P1,
            elevation_bounds,
            LaneType::Driving,
            1,  // xodr_track
            5,  // xodr_lane_id
            K_LINEAR_TOLERANCE,
            K_ANGULAR_TOLERANCE,
        )
        .unwrap()
    }

    // Mock segment for testing
    struct MockSegment;
    impl std::fmt::Debug for MockSegment {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "MockSegment")
        }
    }
    impl Segment for MockSegment {
        fn id(&self) -> &maliput::api::SegmentId {
            unimplemented!()
        }
        fn junction(&self) -> Arc<dyn maliput::api::Junction> {
            unimplemented!()
        }
        fn num_lanes(&self) -> usize {
            1
        }
        fn lane(&self, _index: usize) -> MaliputResult<Arc<dyn Lane>> {
            unimplemented!()
        }
    }

    #[test]
    fn test_constructor() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        assert_eq!(lane.id().string(), "test_lane");
        assert_eq!(lane.index(), 0);
        assert_eq!(lane.get_track(), 1);
        assert_eq!(lane.get_lane_id(), 5);
        assert_eq!(lane.lane_type(), LaneType::Driving);
    }

    #[test]
    fn test_constructor_invalid_xodr_track() {
        let road_curve = create_test_road_curve();
        let lane_width: Arc<dyn Function> = Arc::new(ConstantFunction::new(K_WIDTH, K_P0, K_P1));
        let lane_offset: Arc<dyn Function> =
            Arc::new(ConstantFunction::new(K_LANE_OFFSET, K_P0, K_P1));
        let elevation_bounds = HBounds::new(0.0, 5.0).unwrap();

        let result = MalidriveLane::new(
            LaneId::new("test_lane".to_string()),
            0,
            Weak::<MockSegment>::new() as Weak<dyn Segment>,
            road_curve,
            lane_width,
            lane_offset,
            K_P0,
            K_P1,
            elevation_bounds,
            LaneType::Driving,
            -1, // Invalid xodr_track
            5,
            K_LINEAR_TOLERANCE,
            K_ANGULAR_TOLERANCE,
        );

        assert!(result.is_err());
    }

    #[test]
    fn test_length() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // For a simple linear mapping, length = p1 - p0
        let expected_length = K_P1 - K_P0;
        assert_relative_eq!(lane.length(), expected_length, epsilon = K_LINEAR_TOLERANCE);
    }

    #[test]
    fn test_track_s_from_lane_s() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        assert_relative_eq!(
            lane.track_s_from_lane_s(0.0),
            K_P0,
            epsilon = K_LINEAR_TOLERANCE
        );
        assert_relative_eq!(
            lane.track_s_from_lane_s(50.0),
            50.0,
            epsilon = K_LINEAR_TOLERANCE
        );
        assert_relative_eq!(
            lane.track_s_from_lane_s(100.0),
            K_P1,
            epsilon = K_LINEAR_TOLERANCE
        );
    }

    #[test]
    fn test_lane_s_from_track_s() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        assert_relative_eq!(
            lane.lane_s_from_track_s(K_P0),
            0.0,
            epsilon = K_LINEAR_TOLERANCE
        );
        assert_relative_eq!(
            lane.lane_s_from_track_s(50.0),
            50.0,
            epsilon = K_LINEAR_TOLERANCE
        );
        assert_relative_eq!(
            lane.lane_s_from_track_s(K_P1),
            100.0,
            epsilon = K_LINEAR_TOLERANCE
        );
    }

    #[test]
    fn test_lane_bounds() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // At the beginning
        let bounds = lane.lane_bounds(0.0).unwrap();
        assert_relative_eq!(bounds.min(), -K_WIDTH / 2.0, epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(bounds.max(), K_WIDTH / 2.0, epsilon = K_LINEAR_TOLERANCE);

        // At the end
        let bounds = lane.lane_bounds(lane.length()).unwrap();
        assert_relative_eq!(bounds.min(), -K_WIDTH / 2.0, epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(bounds.max(), K_WIDTH / 2.0, epsilon = K_LINEAR_TOLERANCE);
    }

    #[test]
    fn test_elevation_bounds() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        let bounds = lane.elevation_bounds(0.0, 0.0).unwrap();
        assert_relative_eq!(bounds.min(), 0.0, epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(bounds.max(), 5.0, epsilon = K_LINEAR_TOLERANCE);
    }

    #[test]
    fn test_to_inertial_position_centerline() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // At s=0, r=0, h=0
        // The lane centerline is offset by K_LANE_OFFSET perpendicular to the line
        // Line heading is 45 degrees, so perpendicular is 135 degrees (-45 degrees from y-axis)
        let pos = lane
            .to_inertial_position(&LanePosition::new(0.0, 0.0, 0.0))
            .unwrap();

        // Expected position: start point + offset perpendicular to heading
        // Heading is 45 deg, perpendicular (left) is 135 deg
        // Offset of 10 in perpendicular direction: 10 * (-sin(45), cos(45)) = 10 * (-0.707, 0.707)
        let expected_x = K_XY0.0 + K_LANE_OFFSET * (-std::f64::consts::FRAC_1_SQRT_2);
        let expected_y = K_XY0.1 + K_LANE_OFFSET * std::f64::consts::FRAC_1_SQRT_2;

        assert_relative_eq!(pos.x(), expected_x, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), expected_y, epsilon = 1e-6);
        assert_relative_eq!(pos.z(), 0.0, epsilon = K_LINEAR_TOLERANCE);
    }

    #[test]
    fn test_to_inertial_position_with_lateral_offset() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // At s=0, r=1 (1m to the left), h=0
        let pos = lane
            .to_inertial_position(&LanePosition::new(0.0, 1.0, 0.0))
            .unwrap();

        // Expected: base position + additional 1m in perpendicular direction
        let total_offset = K_LANE_OFFSET + 1.0;
        let expected_x = K_XY0.0 + total_offset * (-std::f64::consts::FRAC_1_SQRT_2);
        let expected_y = K_XY0.1 + total_offset * std::f64::consts::FRAC_1_SQRT_2;

        assert_relative_eq!(pos.x(), expected_x, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), expected_y, epsilon = 1e-6);
    }

    #[test]
    fn test_get_orientation() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // For a flat line at 45 degrees, orientation should have yaw = PI/4
        let rotation = lane
            .get_orientation(&LanePosition::new(0.0, 0.0, 0.0))
            .unwrap();

        assert_relative_eq!(rotation.roll(), 0.0, epsilon = K_ANGULAR_TOLERANCE);
        assert_relative_eq!(rotation.pitch(), 0.0, epsilon = K_ANGULAR_TOLERANCE);
        assert_relative_eq!(rotation.yaw(), PI / 4.0, epsilon = K_ANGULAR_TOLERANCE);

        // Orientation should be the same everywhere for a straight line
        let rotation_mid = lane
            .get_orientation(&LanePosition::new(50.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(rotation_mid.yaw(), PI / 4.0, epsilon = K_ANGULAR_TOLERANCE);

        let rotation_end = lane
            .get_orientation(&LanePosition::new(100.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(rotation_end.yaw(), PI / 4.0, epsilon = K_ANGULAR_TOLERANCE);
    }

    #[test]
    fn test_get_curvature_straight_line() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // For a straight line, curvature should be zero
        let curvature = lane
            .get_curvature(&LanePosition::new(0.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(curvature, 0.0, epsilon = K_LINEAR_TOLERANCE);

        let curvature_mid = lane
            .get_curvature(&LanePosition::new(50.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(curvature_mid, 0.0, epsilon = K_LINEAR_TOLERANCE);
    }

    #[test]
    fn test_eval_motion_derivatives_straight_line() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // For a straight flat line, derivatives should pass through unchanged
        let velocity = IsoLaneVelocity::new(3.0, 2.0, 1.0);
        let derivatives = lane
            .eval_motion_derivatives(&LanePosition::new(50.0, 0.0, 0.0), &velocity)
            .unwrap();

        // For a straight line with constant offset, ds/dsigma = 1
        assert_relative_eq!(derivatives.s(), 3.0, epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(derivatives.r(), 2.0, epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(derivatives.h(), 1.0, epsilon = K_LINEAR_TOLERANCE);
    }

    #[test]
    fn test_contains() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // Inside the lane
        assert!(lane.contains(&LanePosition::new(50.0, 0.0, 2.5)));

        // Outside s bounds
        assert!(!lane.contains(&LanePosition::new(-1.0, 0.0, 0.0)));
        assert!(!lane.contains(&LanePosition::new(101.0, 0.0, 0.0)));

        // Outside r bounds (lane width is 5, so bounds are -2.5 to 2.5)
        assert!(!lane.contains(&LanePosition::new(50.0, 3.0, 0.0)));
        assert!(!lane.contains(&LanePosition::new(50.0, -3.0, 0.0)));

        // Outside h bounds (0 to 5)
        assert!(!lane.contains(&LanePosition::new(50.0, 0.0, -1.0)));
        assert!(!lane.contains(&LanePosition::new(50.0, 0.0, 6.0)));
    }

    #[test]
    fn test_validate_s() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // Valid s values
        assert!(lane.validate_s(0.0).is_ok());
        assert!(lane.validate_s(50.0).is_ok());
        assert!(lane.validate_s(100.0).is_ok());

        // Out of range
        assert!(lane.validate_s(-1.0).is_err());
        assert!(lane.validate_s(101.0).is_err());
    }

    #[test]
    fn test_to_lane_position_on_centerline() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // Get inertial position at centerline
        let inertial_pos = lane
            .to_inertial_position(&LanePosition::new(50.0, 0.0, 0.0))
            .unwrap();

        // Convert back
        let result = lane.to_lane_position(&inertial_pos).unwrap();

        assert_relative_eq!(result.lane_position.s(), 50.0, epsilon = 1e-3);
        assert_relative_eq!(result.lane_position.r(), 0.0, epsilon = 1e-3);
        assert_relative_eq!(result.lane_position.h(), 0.0, epsilon = 1e-3);
        assert_relative_eq!(result.distance, 0.0, epsilon = 1e-3);
    }

    // ============================================================================
    // C++ Parity Tests - Based on MalidriveFlatLineLaneFullyInitializedTest
    // ============================================================================
    // These tests replicate the test cases from lane_test.cc to ensure
    // consistent behavior between the C++ and Rust implementations.

    /// Test ToInertialPosition at centerline positions (from C++ test)
    #[test]
    fn cpp_parity_to_inertial_position_centerline() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // Expected values from C++ test: MalidriveFlatLineLaneFullyInitializedTest.ToInertialPosition
        // At s=0, r=0
        let pos = lane
            .to_inertial_position(&LanePosition::new(0.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 2.9289321881345254, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 19.071067811865476, epsilon = 1e-6);
        assert_relative_eq!(pos.z(), 0.0, epsilon = 1e-6);

        // At s=50, r=0
        let pos = lane
            .to_inertial_position(&LanePosition::new(50.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 38.2842712474619, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 54.426406871192846, epsilon = 1e-6);

        // At s=100, r=0
        let pos = lane
            .to_inertial_position(&LanePosition::new(100.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 73.63961030678928, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 89.78174593052022, epsilon = 1e-6);
    }

    /// Test ToInertialPosition to the left (from C++ test)
    #[test]
    fn cpp_parity_to_inertial_position_left() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // At s=0, r=1 (to the left)
        let pos = lane
            .to_inertial_position(&LanePosition::new(0.0, 1.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 2.2218254069479784, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 19.77817459305202, epsilon = 1e-6);

        // At s=50, r=1
        let pos = lane
            .to_inertial_position(&LanePosition::new(50.0, 1.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 37.577164466275356, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 55.13351365237939, epsilon = 1e-6);

        // At s=100, r=1
        let pos = lane
            .to_inertial_position(&LanePosition::new(100.0, 1.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 72.93250352560273, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 90.48885271170676, epsilon = 1e-6);
    }

    /// Test ToInertialPosition to the right (from C++ test)
    #[test]
    fn cpp_parity_to_inertial_position_right() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // At s=0, r=-2 (to the right)
        let pos = lane
            .to_inertial_position(&LanePosition::new(0.0, -2.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 4.34314575050762, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 17.65685424949238, epsilon = 1e-6);

        // At s=50, r=-2
        let pos = lane
            .to_inertial_position(&LanePosition::new(50.0, -2.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 39.698484809834994, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 53.01219330881975, epsilon = 1e-6);

        // At s=100, r=-2
        let pos = lane
            .to_inertial_position(&LanePosition::new(100.0, -2.0, 0.0))
            .unwrap();
        assert_relative_eq!(pos.x(), 75.05382386916237, epsilon = 1e-6);
        assert_relative_eq!(pos.y(), 88.36753236814712, epsilon = 1e-6);
    }

    /// Test GetOrientation (from C++ test)
    #[test]
    fn cpp_parity_get_orientation() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // Expected rotation for 45-degree line: roll=0, pitch=0, yaw=PI/4
        let expected_yaw = PI / 4.0;

        // At centerline
        let rot = lane
            .get_orientation(&LanePosition::new(0.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(rot.roll(), 0.0, epsilon = K_ANGULAR_TOLERANCE);
        assert_relative_eq!(rot.pitch(), 0.0, epsilon = K_ANGULAR_TOLERANCE);
        assert_relative_eq!(rot.yaw(), expected_yaw, epsilon = K_ANGULAR_TOLERANCE);

        let rot = lane
            .get_orientation(&LanePosition::new(50.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(rot.yaw(), expected_yaw, epsilon = K_ANGULAR_TOLERANCE);

        let rot = lane
            .get_orientation(&LanePosition::new(100.0, 0.0, 0.0))
            .unwrap();
        assert_relative_eq!(rot.yaw(), expected_yaw, epsilon = K_ANGULAR_TOLERANCE);

        // To the left (r=1)
        let rot = lane
            .get_orientation(&LanePosition::new(0.0, 1.0, 0.0))
            .unwrap();
        assert_relative_eq!(rot.yaw(), expected_yaw, epsilon = K_ANGULAR_TOLERANCE);

        // To the right (r=-2)
        let rot = lane
            .get_orientation(&LanePosition::new(0.0, -2.0, 0.0))
            .unwrap();
        assert_relative_eq!(rot.yaw(), expected_yaw, epsilon = K_ANGULAR_TOLERANCE);
    }

    /// Test GetCurvature for straight line (from C++ test)
    #[test]
    fn cpp_parity_get_curvature() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // For a straight line, curvature should be zero everywhere
        let expected_curvature = 0.0;

        // At centerline
        assert_relative_eq!(
            lane.get_curvature(&LanePosition::new(0.0, 0.0, 0.0))
                .unwrap(),
            expected_curvature,
            epsilon = K_LINEAR_TOLERANCE
        );
        assert_relative_eq!(
            lane.get_curvature(&LanePosition::new(50.0, 0.0, 0.0))
                .unwrap(),
            expected_curvature,
            epsilon = K_LINEAR_TOLERANCE
        );
        assert_relative_eq!(
            lane.get_curvature(&LanePosition::new(100.0, 0.0, 0.0))
                .unwrap(),
            expected_curvature,
            epsilon = K_LINEAR_TOLERANCE
        );

        // To the left
        assert_relative_eq!(
            lane.get_curvature(&LanePosition::new(50.0, 1.0, 0.0))
                .unwrap(),
            expected_curvature,
            epsilon = K_LINEAR_TOLERANCE
        );

        // To the right
        assert_relative_eq!(
            lane.get_curvature(&LanePosition::new(50.0, -2.0, 0.0))
                .unwrap(),
            expected_curvature,
            epsilon = K_LINEAR_TOLERANCE
        );
    }

    /// Test EvalMotionDerivatives (from C++ test)
    #[test]
    fn cpp_parity_eval_motion_derivatives() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        let velocity = IsoLaneVelocity::new(3.0, 2.0, 1.0);
        let expected = LanePosition::new(3.0, 2.0, 1.0);

        // At centerline
        let result = lane
            .eval_motion_derivatives(&LanePosition::new(0.0, 0.0, 0.0), &velocity)
            .unwrap();
        assert_relative_eq!(result.s(), expected.s(), epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(result.r(), expected.r(), epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(result.h(), expected.h(), epsilon = K_LINEAR_TOLERANCE);

        let result = lane
            .eval_motion_derivatives(&LanePosition::new(50.0, 0.0, 0.0), &velocity)
            .unwrap();
        assert_relative_eq!(result.s(), expected.s(), epsilon = K_LINEAR_TOLERANCE);

        let result = lane
            .eval_motion_derivatives(&LanePosition::new(100.0, 0.0, 0.0), &velocity)
            .unwrap();
        assert_relative_eq!(result.s(), expected.s(), epsilon = K_LINEAR_TOLERANCE);
    }

    /// Test lane bounds at various positions (from C++ test)
    #[test]
    fn cpp_parity_bounds() {
        let road_curve = create_test_road_curve();
        let lane = create_test_lane(road_curve);

        // Lane bounds should be -width/2 to width/2
        let bounds = lane.lane_bounds(0.0).unwrap();
        assert_relative_eq!(bounds.min(), -K_WIDTH / 2.0, epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(bounds.max(), K_WIDTH / 2.0, epsilon = K_LINEAR_TOLERANCE);

        let bounds = lane.lane_bounds(lane.length()).unwrap();
        assert_relative_eq!(bounds.min(), -K_WIDTH / 2.0, epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(bounds.max(), K_WIDTH / 2.0, epsilon = K_LINEAR_TOLERANCE);

        // Elevation bounds
        let h_bounds = lane.elevation_bounds(0.0, 0.0).unwrap();
        assert_relative_eq!(h_bounds.min(), 0.0, epsilon = K_LINEAR_TOLERANCE);
        assert_relative_eq!(h_bounds.max(), 5.0, epsilon = K_LINEAR_TOLERANCE);
    }
}
