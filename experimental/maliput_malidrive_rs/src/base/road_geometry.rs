//! RoadGeometry implementation for maliput_malidrive.

use std::sync::Arc;

use maliput::api::{
    BranchPoint, InertialPosition, Junction, Lane, RoadGeometry, RoadGeometryId,
    RoadPositionResult,
};

/// A concrete RoadGeometry implementation for maliput_malidrive.
pub struct MalidriveRoadGeometry {
    /// RoadGeometry identifier.
    id: RoadGeometryId,
    /// Linear tolerance.
    linear_tolerance: f64,
    /// Angular tolerance.
    angular_tolerance: f64,
    /// Scale length.
    scale_length: f64,
    /// Translation from inertial to backend frame.
    inertial_to_backend_frame_translation: [f64; 3],
    /// Junctions in this road geometry.
    junctions: Vec<Arc<dyn Junction>>,
    /// Branch points in this road geometry.
    branch_points: Vec<Arc<dyn BranchPoint>>,
    /// All lanes (for fast lookup).
    all_lanes: Vec<Arc<dyn Lane>>,
}

impl MalidriveRoadGeometry {
    /// Creates a new MalidriveRoadGeometry.
    pub fn new(
        id: RoadGeometryId,
        linear_tolerance: f64,
        angular_tolerance: f64,
        scale_length: f64,
        inertial_to_backend_frame_translation: [f64; 3],
    ) -> Self {
        Self {
            id,
            linear_tolerance,
            angular_tolerance,
            scale_length,
            inertial_to_backend_frame_translation,
            junctions: Vec::new(),
            branch_points: Vec::new(),
            all_lanes: Vec::new(),
        }
    }

    /// Adds a junction to this road geometry.
    pub fn add_junction(&mut self, junction: Arc<dyn Junction>) {
        // Collect lanes from the junction
        for seg_idx in 0..junction.num_segments() {
            let segment = junction.segment(seg_idx);
            for lane_idx in 0..segment.num_lanes() {
                self.all_lanes.push(segment.lane(lane_idx));
            }
        }
        self.junctions.push(junction);
    }

    /// Adds a branch point to this road geometry.
    pub fn add_branch_point(&mut self, branch_point: Arc<dyn BranchPoint>) {
        self.branch_points.push(branch_point);
    }

    /// Sets the junctions for this road geometry.
    pub fn set_junctions(&mut self, junctions: Vec<Arc<dyn Junction>>) {
        self.all_lanes.clear();
        for junction in &junctions {
            for seg_idx in 0..junction.num_segments() {
                let segment = junction.segment(seg_idx);
                for lane_idx in 0..segment.num_lanes() {
                    self.all_lanes.push(segment.lane(lane_idx));
                }
            }
        }
        self.junctions = junctions;
    }

    /// Sets the branch points for this road geometry.
    pub fn set_branch_points(&mut self, branch_points: Vec<Arc<dyn BranchPoint>>) {
        self.branch_points = branch_points;
    }
}

impl RoadGeometry for MalidriveRoadGeometry {
    fn id(&self) -> &RoadGeometryId {
        &self.id
    }

    fn num_junctions(&self) -> usize {
        self.junctions.len()
    }

    fn junction(&self, index: usize) -> Arc<dyn Junction> {
        Arc::clone(&self.junctions[index])
    }

    fn num_branch_points(&self) -> usize {
        self.branch_points.len()
    }

    fn branch_point(&self, index: usize) -> Arc<dyn BranchPoint> {
        Arc::clone(&self.branch_points[index])
    }

    fn by_id(&self) -> &dyn maliput::api::RoadGeometryIdIndex {
        self
    }

    fn to_road_position(&self, inertial_pos: &InertialPosition) -> RoadPositionResult {
        // Find the closest lane
        let mut best_result: Option<(Arc<dyn Lane>, maliput::api::LanePositionResult)> = None;
        let mut best_distance = f64::MAX;

        for lane in &self.all_lanes {
            let result = lane.to_lane_position(inertial_pos);
            if result.distance() < best_distance {
                best_distance = result.distance();
                best_result = Some((Arc::clone(lane), result));
            }
        }

        match best_result {
            Some((lane, lane_result)) => {
                RoadPositionResult::new(lane, lane_result.lane_position, lane_result.nearest_position, lane_result.distance)
            }
            None => {
                // No lanes found - return a default
                panic!("No lanes in road geometry");
            }
        }
    }

    fn linear_tolerance(&self) -> f64 {
        self.linear_tolerance
    }

    fn angular_tolerance(&self) -> f64 {
        self.angular_tolerance
    }

    fn scale_length(&self) -> f64 {
        self.scale_length
    }

    fn inertial_to_backend_frame_translation(&self) -> [f64; 3] {
        self.inertial_to_backend_frame_translation
    }
}

impl maliput::api::RoadGeometryIdIndex for MalidriveRoadGeometry {
    fn get_lane(&self, id: &maliput::api::LaneId) -> Option<Arc<dyn Lane>> {
        self.all_lanes
            .iter()
            .find(|lane| lane.id() == id)
            .cloned()
    }

    fn get_segment(&self, id: &maliput::api::SegmentId) -> Option<Arc<dyn maliput::api::Segment>> {
        for junction in &self.junctions {
            for seg_idx in 0..junction.num_segments() {
                let segment = junction.segment(seg_idx);
                if segment.id() == id {
                    return Some(segment);
                }
            }
        }
        None
    }

    fn get_junction(&self, id: &maliput::api::JunctionId) -> Option<Arc<dyn Junction>> {
        self.junctions
            .iter()
            .find(|junction| junction.id() == id)
            .cloned()
    }

    fn get_branch_point(
        &self,
        id: &maliput::api::BranchPointId,
    ) -> Option<Arc<dyn BranchPoint>> {
        self.branch_points
            .iter()
            .find(|bp| bp.id() == id)
            .cloned()
    }

    fn get_lanes(&self) -> Vec<Arc<dyn Lane>> {
        self.all_lanes.clone()
    }
}

impl std::fmt::Debug for MalidriveRoadGeometry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MalidriveRoadGeometry")
            .field("id", &self.id)
            .field("num_junctions", &self.junctions.len())
            .field("num_branch_points", &self.branch_points.len())
            .field("num_lanes", &self.all_lanes.len())
            .field("linear_tolerance", &self.linear_tolerance)
            .field("angular_tolerance", &self.angular_tolerance)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_road_geometry_creation() {
        let rg = MalidriveRoadGeometry::new(
            RoadGeometryId::new("test_rg"),
            1e-3,
            1e-3,
            1.0,
            [0.0, 0.0, 0.0],
        );

        assert_eq!(rg.id().string(), "test_rg");
        assert_eq!(rg.num_junctions(), 0);
        assert_eq!(rg.num_branch_points(), 0);
        assert_eq!(rg.linear_tolerance(), 1e-3);
        assert_eq!(rg.angular_tolerance(), 1e-3);
    }
}
