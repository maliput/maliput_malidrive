//! RoadGeometry implementation for maliput_malidrive.

use std::collections::HashMap;
use std::sync::Arc;

use maliput::api::{
    BranchPoint, BranchPointId, IdIndex, InertialPosition, Junction, JunctionId, Lane, LaneId,
    MaliputError, MaliputResult, RoadGeometry, RoadGeometryId, RoadPosition, RoadPositionResult,
    Segment, SegmentId,
};
use maliput::math::Vector3;

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
    inertial_to_backend_frame_translation: Vector3,
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
            inertial_to_backend_frame_translation: Vector3::new(
                inertial_to_backend_frame_translation[0],
                inertial_to_backend_frame_translation[1],
                inertial_to_backend_frame_translation[2],
            ),
            junctions: Vec::new(),
            branch_points: Vec::new(),
            all_lanes: Vec::new(),
        }
    }

    /// Adds a junction to this road geometry.
    pub fn add_junction(&mut self, junction: Arc<dyn Junction>) {
        // Collect lanes from the junction
        for seg_idx in 0..junction.num_segments() {
            if let Ok(segment) = junction.segment(seg_idx) {
                for lane_idx in 0..segment.num_lanes() {
                    if let Ok(lane) = segment.lane(lane_idx) {
                        self.all_lanes.push(lane);
                    }
                }
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
        self.junctions = junctions;
    }

    /// Sets the branch points for this road geometry.
    pub fn set_branch_points(&mut self, branch_points: Vec<Arc<dyn BranchPoint>>) {
        self.branch_points = branch_points;
    }

    /// Sets all lanes for this road geometry.
    pub fn set_all_lanes(&mut self, lanes: Vec<Arc<dyn Lane>>) {
        self.all_lanes = lanes;
    }
}

impl RoadGeometry for MalidriveRoadGeometry {
    fn id(&self) -> &RoadGeometryId {
        &self.id
    }

    fn num_junctions(&self) -> usize {
        self.junctions.len()
    }

    fn junction(&self, index: usize) -> MaliputResult<Arc<dyn Junction>> {
        self.junctions.get(index).cloned().ok_or_else(|| {
            MaliputError::IndexOutOfBounds {
                index,
                max: self.junctions.len().saturating_sub(1),
            }
        })
    }

    fn num_branch_points(&self) -> usize {
        self.branch_points.len()
    }

    fn branch_point(&self, index: usize) -> MaliputResult<Arc<dyn BranchPoint>> {
        self.branch_points.get(index).cloned().ok_or_else(|| {
            MaliputError::IndexOutOfBounds {
                index,
                max: self.branch_points.len().saturating_sub(1),
            }
        })
    }

    fn by_id(&self) -> Arc<dyn IdIndex> {
        Arc::new(MalidriveIdIndex::new(
            self.all_lanes.clone(),
            self.junctions.clone(),
            self.branch_points.clone(),
        ))
    }

    fn to_road_position(
        &self,
        inertial_pos: &InertialPosition,
        _hint: Option<&RoadPosition>,
    ) -> MaliputResult<RoadPositionResult> {
        // Find the closest lane
        let mut best_result: Option<(Arc<dyn Lane>, maliput::api::LanePositionResult)> = None;
        let mut best_distance = f64::MAX;

        for lane in &self.all_lanes {
            let result = lane.to_lane_position(inertial_pos)?;
            if result.distance < best_distance {
                best_distance = result.distance;
                best_result = Some((Arc::clone(lane), result));
            }
        }

        match best_result {
            Some((lane, lane_result)) => Ok(RoadPositionResult {
                road_position: RoadPosition::new(lane, lane_result.lane_position),
                nearest_position: lane_result.nearest_position,
                distance: lane_result.distance,
            }),
            None => Err(MaliputError::Validation(
                "No lanes in road geometry".to_string(),
            )),
        }
    }

    fn find_road_positions(
        &self,
        inertial_pos: &InertialPosition,
        radius: f64,
    ) -> MaliputResult<Vec<RoadPositionResult>> {
        if radius < 0.0 {
            return Err(MaliputError::Validation(
                "Radius must be non-negative".to_string(),
            ));
        }

        let mut results = Vec::new();

        for lane in &self.all_lanes {
            let result = lane.to_lane_position(inertial_pos)?;
            if result.distance <= radius {
                results.push(RoadPositionResult {
                    road_position: RoadPosition::new(Arc::clone(lane), result.lane_position),
                    nearest_position: result.nearest_position,
                    distance: result.distance,
                });
            }
        }

        Ok(results)
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

    fn inertial_to_backend_frame_translation(&self) -> Vector3 {
        self.inertial_to_backend_frame_translation
    }

    fn check_invariants(&self) -> Vec<String> {
        // For now, return an empty vector (no violations)
        // TODO: Implement proper invariant checking
        Vec::new()
    }

    fn geo_reference_info(&self) -> Option<String> {
        // TODO: Return geo-reference info from XODR file if available
        None
    }
}

/// IdIndex implementation for MalidriveRoadGeometry.
struct MalidriveIdIndex {
    lanes: Vec<Arc<dyn Lane>>,
    junctions: Vec<Arc<dyn Junction>>,
    branch_points: Vec<Arc<dyn BranchPoint>>,
}

impl MalidriveIdIndex {
    fn new(
        lanes: Vec<Arc<dyn Lane>>,
        junctions: Vec<Arc<dyn Junction>>,
        branch_points: Vec<Arc<dyn BranchPoint>>,
    ) -> Self {
        Self {
            lanes,
            junctions,
            branch_points,
        }
    }
}

impl std::fmt::Debug for MalidriveIdIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MalidriveIdIndex")
            .field("num_lanes", &self.lanes.len())
            .field("num_junctions", &self.junctions.len())
            .field("num_branch_points", &self.branch_points.len())
            .finish()
    }
}

impl IdIndex for MalidriveIdIndex {
    fn get_lane(&self, id: &LaneId) -> Option<Arc<dyn Lane>> {
        self.lanes.iter().find(|lane| lane.id() == id).cloned()
    }

    fn get_lanes(&self) -> HashMap<LaneId, Arc<dyn Lane>> {
        let mut map = HashMap::new();
        for lane in &self.lanes {
            // Clone the LaneId - lane.id() returns &LaneId
            let id: LaneId = lane.id().clone();
            map.insert(id, Arc::clone(lane));
        }
        map
    }

    fn get_segment(&self, id: &SegmentId) -> Option<Arc<dyn Segment>> {
        for junction in &self.junctions {
            for seg_idx in 0..junction.num_segments() {
                if let Ok(segment) = junction.segment(seg_idx) {
                    if segment.id() == id {
                        return Some(segment);
                    }
                }
            }
        }
        None
    }

    fn get_junction(&self, id: &JunctionId) -> Option<Arc<dyn Junction>> {
        self.junctions
            .iter()
            .find(|junction| junction.id() == id)
            .cloned()
    }

    fn get_branch_point(&self, id: &BranchPointId) -> Option<Arc<dyn BranchPoint>> {
        self.branch_points.iter().find(|bp| bp.id() == id).cloned()
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
            RoadGeometryId::new("test_rg".to_string()),
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
