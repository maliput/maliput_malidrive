//! RoadGeometry builder for constructing maliput road geometry from OpenDRIVE.
//!
//! This module implements the road geometry building process that transforms
//! parsed OpenDRIVE data into a complete maliput RoadGeometry with proper
//! junctions, segments, lanes, and branch points.
//!
//! # Implementation Status
//!
//! The following features from the C++ implementation are currently implemented:
//! - Basic road geometry construction from XODR data
//! - Junction, Segment, and Lane creation
//! - Road curve building (line, arc, spiral, param_poly3)
//! - Lane width function creation
//! - Basic branch point creation
//! - Drivable lane filtering
//!
//! # TODO: Missing Features (compared to C++ implementation)
//!
//! The following features need to be implemented for full parity:
//!
//! ## High Priority
//! - **Lane Offset Calculation**: Proper `LaneOffset` computation using adjacent lane functions
//!   (see C++ `road_curve::LaneOffset::AdjacentLaneFunctions`)
//! - **Reference Line Offset**: Create reference line offset for segments
//!   (see C++ `MakeReferenceLineOffset`)
//! - **Branch Point Connections**: Proper logic for connecting lane ends:
//!   - `FindConnectingLaneEndsForLaneEnd`
//!   - `SolveLaneEndsForInnerLaneSection`
//!   - `SolveLaneEndsForConnectingRoad`
//!   - `SolveLaneEndsForJunction`
//!   - `AttachLaneEndToBranchPoint`
//!   - `SetDefaultsToBranchPoints`
//!
//! ## Medium Priority
//! - **Lane Width Validation**: `VerifyNonNegativeLaneWidth` - validate lane widths are non-negative
//! - **Tolerance Range Support**: Automatic retry with different tolerances
//! - **Geometry Simplification**: Support for `SimplificationPolicy`
//!
//! ## Lower Priority
//! - **Lane Boundaries**: `BuildLaneBoundaries` with road marking support
//! - **Scaled Domain Function**: Wrap functions with scaled domains for piecewise curves
//! - **KD-Tree Strategy**: Initialization for efficient spatial queries
//! - **Parallel Build Policy**: Multi-threaded lane building support

use std::collections::HashMap;
use std::sync::{Arc, RwLock, Weak};

use maliput::api::{
    BranchPointId, HBounds, JunctionId, LaneId, LaneType as MaliputLaneType,
    RoadGeometryId, SegmentId,
};

use crate::base::{
    LaneEndWhich, MalidriveBranchPoint, MalidriveJunction, MalidriveLane, MalidriveRoadGeometry,
    MalidriveSegment,
};
use crate::builder::{BuilderParams, RoadCurveFactory};
use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{CubicPolynomial, Function, PiecewiseCubicPolynomial, RoadCurve};
use crate::xodr::{Junction as XodrJunction, Lane as XodrLane, LaneSection, LaneType as XodrLaneType, RoadHeader};

/// Strict linear tolerance constant matching C++.
pub const STRICT_LINEAR_TOLERANCE: f64 = 1e-6;
/// Strict angular tolerance constant matching C++.
pub const STRICT_ANGULAR_TOLERANCE: f64 = 1e-6;
/// Default scale length.
pub const SCALE_LENGTH: f64 = 1.0;

/// Returns default elevation bounds (0.0, 5.0).
fn default_elevation_bounds() -> HBounds {
    // min must be <= 0, max must be >= 0, so use 0.0 and 5.0
    HBounds::new(0.0, 5.0).expect("Default elevation bounds should be valid")
}

/// Information about a built lane for branch point creation.
#[derive(Clone)]
pub struct LaneInfo {
    /// The lane arc.
    pub lane: Arc<MalidriveLane>,
    /// The XODR road ID.
    pub xodr_road_id: String,
    /// The XODR lane section index.
    pub xodr_lane_section_index: usize,
    /// The XODR lane ID.
    pub xodr_lane_id: i32,
    /// Start s in track coordinates.
    pub track_s_start: f64,
    /// End s in track coordinates.
    pub track_s_end: f64,
}

/// Builder for constructing MalidriveRoadGeometry from OpenDRIVE data.
pub struct RoadGeometryBuilder {
    /// Builder parameters.
    params: BuilderParams,
    /// Road curve factory.
    road_curve_factory: RoadCurveFactory,
    /// Whether to omit non-drivable lanes.
    omit_nondrivable_lanes: bool,
}

impl RoadGeometryBuilder {
    /// Creates a new RoadGeometryBuilder with the given parameters.
    ///
    /// # Errors
    ///
    /// Returns an error if tolerances are negative.
    pub fn new(params: BuilderParams) -> MalidriveResult<Self> {
        // Validate parameters (matching C++ constructor validation)
        if params.linear_tolerance < 0.0 {
            return Err(MalidriveError::ValidationError(format!(
                "linear tolerance should be non-negative: {}",
                params.linear_tolerance
            )));
        }
        if params.angular_tolerance < 0.0 {
            return Err(MalidriveError::ValidationError(format!(
                "angular tolerance should be non-negative: {}",
                params.angular_tolerance
            )));
        }
        if params.scale_length < 0.0 {
            return Err(MalidriveError::ValidationError(format!(
                "scale_length should be non-negative: {}",
                params.scale_length
            )));
        }

        let road_curve_factory = RoadCurveFactory::new(params.linear_tolerance, params.scale_length);

        Ok(Self {
            omit_nondrivable_lanes: params.omit_nondrivable_lanes,
            params,
            road_curve_factory,
        })
    }

    /// Builds the road geometry from parsed OpenDRIVE roads and junctions.
    pub fn build(
        &self,
        roads: &[RoadHeader],
        xodr_junctions: &[XodrJunction],
    ) -> MalidriveResult<Arc<MalidriveRoadGeometry>> {
        // Build road curves for each road
        let road_curves = self.build_road_curves(roads)?;

        // Build junctions, segments, and lanes
        let (junctions, _segments, all_lane_infos) =
            self.build_junctions_segments_lanes(roads, xodr_junctions, &road_curves)?;

        // Build branch points
        let branch_points = self.build_branch_points(&all_lane_infos, roads)?;

        // Create the road geometry with all components
        let mut road_geometry = MalidriveRoadGeometry::new(
            RoadGeometryId::new(self.params.road_geometry_id.clone()),
            self.params.linear_tolerance,
            self.params.angular_tolerance,
            self.params.scale_length,
            self.params.inertial_to_backend_frame_translation,
        );

        // Set the built components
        road_geometry.set_junctions(junctions);
        road_geometry.set_branch_points(branch_points);

        // Collect all lanes
        let all_lanes: Vec<Arc<dyn maliput::api::Lane>> = all_lane_infos
            .iter()
            .map(|info| Arc::clone(&info.lane) as Arc<dyn maliput::api::Lane>)
            .collect();
        road_geometry.set_all_lanes(all_lanes);

        Ok(Arc::new(road_geometry))
    }

    /// Builds road curves for all roads.
    fn build_road_curves(
        &self,
        roads: &[RoadHeader],
    ) -> MalidriveResult<HashMap<String, Arc<RoadCurve>>> {
        let mut road_curves = HashMap::new();
        for road in roads {
            let road_curve = self.road_curve_factory.make_road_curve(road)?;
            road_curves.insert(road.id.clone(), road_curve);
        }
        Ok(road_curves)
    }

    /// Builds junctions, segments, and lanes.
    #[allow(clippy::type_complexity)]
    fn build_junctions_segments_lanes(
        &self,
        roads: &[RoadHeader],
        xodr_junctions: &[XodrJunction],
        road_curves: &HashMap<String, Arc<RoadCurve>>,
    ) -> MalidriveResult<(
        Vec<Arc<dyn maliput::api::Junction>>,
        Vec<Arc<MalidriveSegment>>,
        Vec<LaneInfo>,
    )> {
        let mut junctions: HashMap<String, Arc<RwLock<MalidriveJunction>>> = HashMap::new();
        let mut all_segments: Vec<Arc<MalidriveSegment>> = Vec::new();
        let mut all_lane_infos: Vec<LaneInfo> = Vec::new();

        // Create a map from XODR junction ID to junction info (for future use)
        let _xodr_junction_map: HashMap<&str, &XodrJunction> =
            xodr_junctions.iter().map(|j| (j.id.as_str(), j)).collect();

        for road in roads {
            let road_curve = road_curves
                .get(&road.id)
                .ok_or_else(|| MalidriveError::ParsingError(format!("No road curve for road {}", road.id)))?;

            let lanes_data = &road.lanes;

            for (section_idx, lane_section) in lanes_data.lane_sections.iter().enumerate() {
                // Calculate section s-range (p0 and p1 for the segment)
                let section_start = lane_section.s;
                let section_end = if section_idx + 1 < lanes_data.lane_sections.len() {
                    lanes_data.lane_sections[section_idx + 1].s
                } else {
                    road.length
                };

                // Compute junction ID - use XODR junction ID if road belongs to junction
                let junction_id = if road.junction != "-1" {
                    road.junction.clone()
                } else {
                    get_junction_id(&road.id, section_idx)
                };

                let segment_id = get_segment_id(&road.id, section_idx);

                // Get or create the junction
                let junction_arc = junctions
                    .entry(junction_id.clone())
                    .or_insert_with(|| {
                        Arc::new(RwLock::new(MalidriveJunction::new(
                            JunctionId::new(junction_id.clone()),
                            Weak::<MalidriveRoadGeometry>::new() as Weak<dyn maliput::api::RoadGeometry>,
                        )))
                    });

                // Create the segment - we need a Weak reference to the junction
                // Note: Using placeholder since we can't easily convert Arc<RwLock<T>> to Weak<dyn Trait>
                let junction_dyn_weak: Weak<dyn maliput::api::Junction> = 
                    Weak::<MalidriveJunction>::new() as Weak<dyn maliput::api::Junction>; // Placeholder for now

                // Create reference line offset function
                // TODO: Parse from XODR LaneOffset data. For now, use a zero constant function.
                let reference_line_offset: Arc<dyn Function> = Arc::new(
                    CubicPolynomial::constant(0.0, section_start, section_end)
                );

                // First build the lanes for this segment (we need them to add to segment)
                let segment_placeholder: Arc<MalidriveSegment> = Arc::new(MalidriveSegment::new(
                    SegmentId::new(segment_id.clone()),
                    junction_dyn_weak.clone(),
                    Arc::clone(road_curve),
                    Arc::clone(&reference_line_offset),
                    section_start,
                    section_end,
                )?);
                
                // Build lanes for this section
                let lane_infos = self.build_lanes_for_segment(
                    road,
                    section_idx,
                    lane_section,
                    road_curve,
                    &segment_placeholder,
                )?;
                
                // Now create the actual segment with the lanes
                let mut segment = MalidriveSegment::new(
                    SegmentId::new(segment_id.clone()),
                    junction_dyn_weak,
                    Arc::clone(road_curve),
                    reference_line_offset,
                    section_start,
                    section_end,
                )?;
                
                // Add lanes to segment (hide_lane = false for drivable lanes)
                for lane_info in &lane_infos {
                    segment.add_lane(Arc::clone(&lane_info.lane) as Arc<dyn maliput::api::Lane>, false);
                }
                
                let segment_arc = Arc::new(segment);
                all_segments.push(Arc::clone(&segment_arc));

                // Add segment to junction
                if let Ok(mut junction) = junction_arc.write() {
                    junction.add_segment(Arc::clone(&segment_arc) as Arc<dyn maliput::api::Segment>);
                }

                all_lane_infos.extend(lane_infos);
            }
        }

        // Convert junctions to trait objects
        // Note: We need to take ownership of the inner junction
        let junction_vec: Vec<Arc<dyn maliput::api::Junction>> = junctions
            .into_iter()
            .map(|(_, arc)| {
                // Take the junction out of the RwLock
                match Arc::try_unwrap(arc) {
                    Ok(rwlock) => Arc::new(rwlock.into_inner().unwrap()) as Arc<dyn maliput::api::Junction>,
                    Err(_) => panic!("Junction Arc is still being shared"),
                }
            })
            .collect();

        Ok((junction_vec, all_segments, all_lane_infos))
    }

    /// Builds lanes for a segment from a lane section.
    fn build_lanes_for_segment(
        &self,
        road: &RoadHeader,
        section_idx: usize,
        lane_section: &LaneSection,
        road_curve: &Arc<RoadCurve>,
        segment: &Arc<MalidriveSegment>,
    ) -> MalidriveResult<Vec<LaneInfo>> {
        let mut lane_infos = Vec::new();

        // Calculate section s-range
        let section_start = lane_section.s;
        let section_end = if section_idx + 1 < road.lanes.lane_sections.len() {
            road.lanes.lane_sections[section_idx + 1].s
        } else {
            road.length
        };

        // Collect drivable lanes (right lanes reversed, then left lanes in order)
        let mut drivable_lanes: Vec<&XodrLane> = Vec::new();

        // Right lanes in reverse order (so index 0 is rightmost)
        let mut right_lanes: Vec<_> = lane_section
            .right_lanes
            .iter()
            .filter(|l| !self.omit_nondrivable_lanes || is_drivable_lane_type(l.lane_type))
            .collect();
        right_lanes.reverse();
        drivable_lanes.extend(right_lanes);

        // Left lanes in forward order
        let left_lanes: Vec<_> = lane_section
            .left_lanes
            .iter()
            .filter(|l| !self.omit_nondrivable_lanes || is_drivable_lane_type(l.lane_type))
            .collect();
        drivable_lanes.extend(left_lanes);

        // Parse road ID to integer for lane ID generation
        let road_id_int: i32 = road
            .id
            .parse()
            .map_err(|_| MalidriveError::ParsingError(format!("Invalid road ID: {}", road.id)))?;

        // Create lanes
        for (lane_index, xodr_lane) in drivable_lanes.iter().enumerate() {
            let lane_id = get_lane_id(road_id_int, section_idx as i32, xodr_lane.id);

            // Build lane width function
            let width_fn = self.make_lane_width_function(&xodr_lane.widths, section_start, section_end);

            // Build lane offset function (r-coordinate of lane centerline)
            // TODO: Implement proper lane offset calculation using adjacent lane functions.
            // In C++, this uses `road_curve::LaneOffset` with `AdjacentLaneFunctions`:
            //   - For positive lane ids (left lanes): offset = sum of widths of lanes between center and this lane
            //   - For negative lane ids (right lanes): offset = -sum of widths of lanes between center and this lane
            //   - The offset function accounts for lane width variations along the road
            // Current placeholder: simple constant offset based on lane ID
            let lane_offset_fn: Arc<dyn Function> = Arc::new(
                CubicPolynomial::constant(xodr_lane.id as f64 * 1.75, section_start, section_end)
            );

            // Create the lane using correct API signature
            let segment_weak: Weak<dyn maliput::api::Segment> = Arc::downgrade(segment) as Weak<dyn maliput::api::Segment>;
            let lane = MalidriveLane::new(
                LaneId::new(lane_id.clone()),
                lane_index,
                segment_weak,
                Arc::clone(road_curve),
                width_fn,
                lane_offset_fn,
                section_start,  // p0
                section_end,    // p1
                default_elevation_bounds(),
                xodr_lane_type_to_maliput(xodr_lane.lane_type),
                road_id_int,    // xodr_track
                xodr_lane.id,   // xodr_lane_id
                self.params.linear_tolerance,
                self.params.angular_tolerance,
            )?;

            let lane_arc = Arc::new(lane);

            lane_infos.push(LaneInfo {
                lane: Arc::clone(&lane_arc),
                xodr_road_id: road.id.clone(),
                xodr_lane_section_index: section_idx,
                xodr_lane_id: xodr_lane.id,
                track_s_start: section_start,
                track_s_end: section_end,
            });
        }

        Ok(lane_infos)
    }

    /// Creates a lane width function from XODR lane widths.
    ///
    /// TODO: Add non-negative width validation matching C++ `VerifyNonNegativeLaneWidth`:
    /// - Validate that lane width polynomials produce non-negative values
    /// - Check coefficients satisfy: a >= 0, and polynomial >= 0 for entire domain
    fn make_lane_width_function(
        &self,
        widths: &[crate::xodr::LaneWidth],
        s0: f64,
        s1: f64,
    ) -> Arc<dyn Function> {
        if widths.is_empty() {
            // Default width
            return Arc::new(CubicPolynomial::constant(3.5, s0, s1));
        }

        let mut polynomials: Vec<CubicPolynomial> = Vec::new();

        for (i, width) in widths.iter().enumerate() {
            let start = s0 + width.s_offset;
            let end = if i + 1 < widths.len() {
                s0 + widths[i + 1].s_offset
            } else {
                s1
            };

            polynomials.push(CubicPolynomial::new(width.a, width.b, width.c, width.d, start, end));
        }

        Arc::new(PiecewiseCubicPolynomial::new(polynomials))
    }

    /// Builds branch points for all lanes.
    ///
    /// TODO: Implement full branch point connection logic matching C++:
    ///
    /// The C++ implementation has a sophisticated multi-step process:
    ///
    /// 1. `BuildBranchPointsForLanes` - main entry point that orchestrates:
    ///    - Creates empty branch points for each lane end
    ///    - Calls `FindConnectingLaneEndsForLaneEnd` to find connections
    ///    - Calls `AttachLaneEndToBranchPoint` to connect lane ends
    ///
    /// 2. `FindConnectingLaneEndsForLaneEnd` - determines connection type:
    ///    - `SolveLaneEndsForInnerLaneSection` - connects lanes in consecutive lane sections
    ///    - `SolveLaneEndsForConnectingRoad` - connects lanes via road predecessor/successor
    ///    - `SolveLaneEndsForJunction` - connects lanes from non-junction to junction roads
    ///    - `SolveLaneEndsWithinJunction` - connects lanes within a junction
    ///
    /// 3. `AttachLaneEndToBranchPoint` - adds lane end to appropriate branch point:
    ///    - Determines if lane end should be on A-side or B-side
    ///    - Creates new branch point if needed, or reuses existing
    ///
    /// 4. `SetDefaultsToBranchPoints` - sets the default branch for each lane end:
    ///    - Iterates through all branch points
    ///    - For each side (A/B), sets the first lane as the default continuation
    ///
    /// Current implementation: Creates basic branch points (one per lane end) without connections.
    fn build_branch_points(
        &self,
        lane_infos: &[LaneInfo],
        _roads: &[RoadHeader],
    ) -> MalidriveResult<Vec<Arc<dyn maliput::api::BranchPoint>>> {
        let mut branch_points: Vec<Arc<dyn maliput::api::BranchPoint>> = Vec::new();

        // For now, create simple branch points for each lane end
        for (idx, lane_info) in lane_infos.iter().enumerate() {
            let bp_start_id = get_branch_point_id(idx * 2);
            let bp_finish_id = get_branch_point_id(idx * 2 + 1);

            // Create branch points with lane ends on A-side
            // Use Weak<dyn Lane> as required by add_a_side
            let lane_weak: Weak<dyn maliput::api::Lane> = Arc::downgrade(&lane_info.lane) as Weak<dyn maliput::api::Lane>;

            let mut bp_start = MalidriveBranchPoint::new(
                BranchPointId::new(bp_start_id), 
                Weak::<MalidriveRoadGeometry>::new() as Weak<dyn maliput::api::RoadGeometry>
            );
            bp_start.add_a_side(lane_weak.clone(), LaneEndWhich::Start);

            let mut bp_finish = MalidriveBranchPoint::new(
                BranchPointId::new(bp_finish_id), 
                Weak::<MalidriveRoadGeometry>::new() as Weak<dyn maliput::api::RoadGeometry>
            );
            bp_finish.add_a_side(lane_weak, LaneEndWhich::Finish);

            branch_points.push(Arc::new(bp_start));
            branch_points.push(Arc::new(bp_finish));
        }

        Ok(branch_points)
    }

    /// Returns the builder parameters.
    pub fn params(&self) -> &BuilderParams {
        &self.params
    }
}

/// Checks if a lane type is drivable.
pub fn is_drivable_lane_type(lane_type: XodrLaneType) -> bool {
    matches!(
        lane_type,
        XodrLaneType::Driving | XodrLaneType::Entry | XodrLaneType::Exit | XodrLaneType::OffRamp |
        XodrLaneType::OnRamp | XodrLaneType::Parking | XodrLaneType::Stop | XodrLaneType::Shoulder |
        XodrLaneType::Biking | XodrLaneType::Sidewalk | XodrLaneType::Bidirectional |
        XodrLaneType::ConnectingRamp | XodrLaneType::MwyEntry | XodrLaneType::MwyExit
    )
}

/// Converts XODR lane type to maliput lane type.
fn xodr_lane_type_to_maliput(xodr_type: XodrLaneType) -> MaliputLaneType {
    match xodr_type {
        XodrLaneType::Driving => MaliputLaneType::Driving,
        XodrLaneType::Shoulder => MaliputLaneType::Shoulder,
        XodrLaneType::Parking => MaliputLaneType::Parking,
        XodrLaneType::Biking => MaliputLaneType::Biking,
        XodrLaneType::Walking | XodrLaneType::Sidewalk => MaliputLaneType::Walking,
        XodrLaneType::Entry | XodrLaneType::OnRamp | XodrLaneType::MwyEntry => MaliputLaneType::Entry,
        XodrLaneType::Exit | XodrLaneType::OffRamp | XodrLaneType::MwyExit => MaliputLaneType::Exit,
        XodrLaneType::Stop => MaliputLaneType::Stop,
        XodrLaneType::ConnectingRamp => MaliputLaneType::ConnectingRamp,
        XodrLaneType::Border => MaliputLaneType::Border,
        XodrLaneType::Curb => MaliputLaneType::Curb,
        XodrLaneType::Median => MaliputLaneType::Median,
        XodrLaneType::Restricted => MaliputLaneType::Restricted,
        _ => MaliputLaneType::Unknown,
    }
}

/// Generates a lane ID.
pub fn get_lane_id(xodr_road_id: i32, lane_section_index: i32, xodr_lane_id: i32) -> String {
    format!("{}_{}_{}", xodr_road_id, lane_section_index, xodr_lane_id)
}

/// Generates a segment ID.
pub fn get_segment_id(road_id: &str, lane_section_index: usize) -> String {
    format!("{}_{}", road_id, lane_section_index)
}

/// Generates a junction ID.
pub fn get_junction_id(road_id: &str, lane_section_index: usize) -> String {
    format!("{}_{}", road_id, lane_section_index)
}

/// Generates a branch point ID.
pub fn get_branch_point_id(index: usize) -> String {
    format!("bp_{}", index)
}

/// Convenience function to build a road geometry from OpenDRIVE data.
pub fn build_road_geometry(
    params: BuilderParams,
    roads: &[RoadHeader],
    junctions: &[XodrJunction],
) -> MalidriveResult<Arc<MalidriveRoadGeometry>> {
    let builder = RoadGeometryBuilder::new(params)?;
    builder.build(roads, junctions)
}

#[cfg(test)]
mod tests {
    use super::*;
    use maliput::api::RoadGeometry;

    #[test]
    fn test_builder_creation() {
        let params = BuilderParams::new("test.xodr").with_linear_tolerance(1e-3);
        let builder = RoadGeometryBuilder::new(params).unwrap();
        assert_eq!(builder.params().opendrive_file, "test.xodr");
        assert_eq!(builder.params().linear_tolerance, 1e-3);
    }

    #[test]
    fn test_builder_creation_negative_tolerance_fails() {
        let params = BuilderParams::new("test.xodr").with_linear_tolerance(-1.0);
        let result = RoadGeometryBuilder::new(params);
        assert!(result.is_err());
    }

    #[test]
    fn test_builder_creation_negative_angular_tolerance_fails() {
        let params = BuilderParams::new("test.xodr").with_angular_tolerance(-1.0);
        let result = RoadGeometryBuilder::new(params);
        assert!(result.is_err());
    }

    #[test]
    fn test_builder_creation_negative_scale_length_fails() {
        let params = BuilderParams::new("test.xodr").with_scale_length(-1.0);
        let result = RoadGeometryBuilder::new(params);
        assert!(result.is_err());
    }

    #[test]
    fn test_build_empty_road_geometry() {
        let params = BuilderParams::new("test.xodr");
        let builder = RoadGeometryBuilder::new(params).unwrap();
        let rg = builder.build(&[], &[]).unwrap();
        assert_eq!(rg.num_junctions(), 0);
    }

    #[test]
    fn test_is_drivable_lane_type() {
        assert!(is_drivable_lane_type(XodrLaneType::Driving));
        assert!(is_drivable_lane_type(XodrLaneType::Entry));
        assert!(is_drivable_lane_type(XodrLaneType::Parking));
        assert!(!is_drivable_lane_type(XodrLaneType::Border));
        assert!(!is_drivable_lane_type(XodrLaneType::Curb));
        assert!(!is_drivable_lane_type(XodrLaneType::None));
    }

    #[test]
    fn test_lane_id_generation() {
        assert_eq!(get_lane_id(1, 0, -1), "1_0_-1");
        assert_eq!(get_lane_id(1, 0, 1), "1_0_1");
        assert_eq!(get_lane_id(2, 1, -2), "2_1_-2");
    }

    #[test]
    fn test_segment_id_generation() {
        assert_eq!(get_segment_id("1", 0), "1_0");
        assert_eq!(get_segment_id("2", 1), "2_1");
    }

    #[test]
    fn test_junction_id_generation() {
        assert_eq!(get_junction_id("1", 0), "1_0");
        assert_eq!(get_junction_id("2", 1), "2_1");
    }

    #[test]
    fn test_branch_point_id_generation() {
        assert_eq!(get_branch_point_id(0), "bp_0");
        assert_eq!(get_branch_point_id(42), "bp_42");
    }
}
