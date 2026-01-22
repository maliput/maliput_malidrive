//! RoadGeometry builder for constructing maliput road geometry from OpenDRIVE.
//!
//! This module implements the road geometry building process that transforms
//! parsed OpenDRIVE data into a complete maliput RoadGeometry with proper
//! junctions, segments, lanes, and branch points.
//!
//! ## Build Process
//!
//! The build process follows these steps:
//! 1. Visit each road and create road curves from geometry
//! 2. For each lane section, create or find the corresponding junction
//! 3. Create segments for each lane section
//! 4. Build lanes with correct ordering (right lanes reversed, left lanes forward from center)
//! 5. Build branch points for lane connectivity
//! 6. Set default branch connections
//!
//! ## Lane Ordering
//!
//! In maliput, lanes within a segment are ordered from right to left with index 0
//! being the rightmost lane. The OpenDRIVE lane ordering is:
//! - Right lanes: -1, -2, -3, ... (outward from center)
//! - Center lane: 0 (reference line, non-drivable)
//! - Left lanes: 1, 2, 3, ... (outward from center)
//!
//! The builder reverses right lanes so that the maliput index matches the
//! geometric position from right to left.

use std::collections::HashMap;
use std::sync::{Arc, Weak};

use maliput::api::{
    BranchPointId, JunctionId, LaneId, RoadGeometryId, SegmentId,
};

use crate::base::{
    LaneEndWhich, MalidriveBranchPoint, MalidriveJunction, MalidriveLane,
    MalidriveRoadGeometry, MalidriveSegment,
};
use crate::builder::{BuilderParams, RoadCurveFactory};
use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{RoadCurve, SimpleLaneOffset};
use crate::xodr::{Junction as XodrJunction, Lane as XodrLane, LaneSection, LaneType, RoadHeader};

/// Information about a built lane for branch point creation.
#[derive(Clone)]
struct LaneInfo {
    /// The lane arc.
    lane: Arc<MalidriveLane>,
    /// The XODR road ID.
    xodr_road_id: String,
    /// The XODR lane section index.
    xodr_lane_section_index: usize,
    /// The XODR lane ID.
    xodr_lane_id: i32,
    /// Start s in track coordinates.
    track_s_start: f64,
    /// End s in track coordinates.
    track_s_end: f64,
}

/// Information about a segment being built.
struct SegmentBuildInfo {
    /// The segment.
    segment: Arc<MalidriveSegment>,
    /// Lane infos for this segment.
    lanes: Vec<LaneInfo>,
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
    pub fn new(params: BuilderParams) -> Self {
        let road_curve_factory = RoadCurveFactory::new(params.linear_tolerance, params.scale_length);

        Self {
            omit_nondrivable_lanes: params.omit_nondrivable_lanes,
            params,
            road_curve_factory,
        }
    }

    /// Builds the road geometry from parsed OpenDRIVE roads and junctions.
    pub fn build(
        &self,
        roads: &[RoadHeader],
        xodr_junctions: &[XodrJunction],
    ) -> MalidriveResult<Arc<MalidriveRoadGeometry>> {
        let road_geometry = Arc::new(MalidriveRoadGeometry::new(
            RoadGeometryId::new(&self.params.road_geometry_id),
            self.params.linear_tolerance,
            self.params.angular_tolerance,
            self.params.scale_length,
            self.params.inertial_to_backend_frame_translation,
        ));

        // Build road curves for each road
        let road_curves = self.build_road_curves(roads)?;

        // Build junctions, segments, and lanes
        let (junctions, all_lane_infos) =
            self.build_junctions_segments_lanes(roads, xodr_junctions, &road_curves, &road_geometry)?;

        // Build branch points
        let branch_points = self.build_branch_points(&all_lane_infos, &road_geometry)?;

        // Set the built components on the road geometry
        // Note: We need to use Arc::get_mut here since we have the only reference
        let rg = Arc::try_unwrap(road_geometry).map_err(|_| {
            MalidriveError::BuilderError("Failed to get exclusive access to road geometry".to_string())
        })?;
        
        let mut rg = rg;
        rg.set_junctions(junctions);
        rg.set_branch_points(branch_points);

        // Collect all lanes
        let all_lanes: Vec<Arc<dyn maliput::api::Lane>> = all_lane_infos
            .iter()
            .map(|info| Arc::clone(&info.lane) as Arc<dyn maliput::api::Lane>)
            .collect();
        rg.set_all_lanes(all_lanes);

        Ok(Arc::new(rg))
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
    fn build_junctions_segments_lanes(
        &self,
        roads: &[RoadHeader],
        xodr_junctions: &[XodrJunction],
        road_curves: &HashMap<String, Arc<RoadCurve>>,
        road_geometry: &Arc<MalidriveRoadGeometry>,
    ) -> MalidriveResult<(Vec<Arc<dyn maliput::api::Junction>>, Vec<LaneInfo>)> {
        let mut junctions: HashMap<String, Arc<MalidriveJunction>> = HashMap::new();
        let mut all_lane_infos: Vec<LaneInfo> = Vec::new();
        let rg_weak = Arc::downgrade(road_geometry) as Weak<dyn maliput::api::RoadGeometry>;

        // Create a map from XODR junction ID to junction info
        let xodr_junction_map: HashMap<&str, &XodrJunction> =
            xodr_junctions.iter().map(|j| (j.id.as_str(), j)).collect();

        for road in roads {
            let road_curve = road_curves
                .get(&road.id)
                .ok_or_else(|| MalidriveError::BuilderError(format!("No road curve for road {}", road.id)))?;

            let lanes_data = road
                .lanes
                .as_ref()
                .ok_or_else(|| MalidriveError::BuilderError(format!("Road {} has no lanes", road.id)))?;

            // Process each lane section
            for (section_idx, lane_section) in lanes_data.lane_sections.iter().enumerate() {
                // Determine the junction ID
                let junction_id = self.compute_junction_id(road, section_idx, &xodr_junction_map);

                // Get or create the junction
                let junction = junctions.entry(junction_id.clone()).or_insert_with(|| {
                    Arc::new(MalidriveJunction::new(
                        JunctionId::new(&junction_id),
                        Weak::clone(&rg_weak),
                    ))
                });

                // Compute lane section s-range
                let track_s_start = lane_section.s;
                let track_s_end = if section_idx + 1 < lanes_data.lane_sections.len() {
                    lanes_data.lane_sections[section_idx + 1].s
                } else {
                    road.length
                };

                // Create segment
                let segment_id = format!("{}_{}", road.id, section_idx);
                let segment = Arc::new(MalidriveSegment::new(
                    SegmentId::new(&segment_id),
                    Arc::downgrade(junction) as Weak<dyn maliput::api::Junction>,
                ));

                // Build lanes for this segment
                let lane_infos = self.build_lanes_for_segment(
                    road,
                    section_idx,
                    lane_section,
                    road_curve,
                    &segment,
                    track_s_start,
                    track_s_end,
                )?;

                // Set lanes on segment
                let lanes_for_segment: Vec<Arc<dyn maliput::api::Lane>> =
                    lane_infos.iter().map(|li| Arc::clone(&li.lane) as Arc<dyn maliput::api::Lane>).collect();

                // We need mutable access to segment - use Arc::get_mut
                // Since we just created the segment and haven't shared it yet, this should work
                if let Some(seg) = Arc::get_mut(&mut Arc::clone(&segment).into()) {
                    // This won't work directly, we need a different approach
                }

                // Alternative: Create segment with lanes already set
                let mut segment_with_lanes = MalidriveSegment::new(
                    SegmentId::new(&segment_id),
                    Arc::downgrade(junction) as Weak<dyn maliput::api::Junction>,
                );
                for lane in &lanes_for_segment {
                    segment_with_lanes.add_lane(Arc::clone(lane));
                }
                let segment = Arc::new(segment_with_lanes);

                // Add segment to junction
                // We need to get mutable access to the junction
                let junction_mut = Arc::get_mut(
                    junctions.get_mut(&junction_id).ok_or_else(|| {
                        MalidriveError::BuilderError("Junction not found".to_string())
                    })?,
                )
                .ok_or_else(|| MalidriveError::BuilderError("Cannot get mutable junction".to_string()))?;
                junction_mut.add_segment(segment as Arc<dyn maliput::api::Segment>);

                all_lane_infos.extend(lane_infos);
            }
        }

        // Convert junctions to vec
        let junctions_vec: Vec<Arc<dyn maliput::api::Junction>> = junctions
            .into_values()
            .map(|j| j as Arc<dyn maliput::api::Junction>)
            .collect();

        Ok((junctions_vec, all_lane_infos))
    }

    /// Computes the junction ID for a lane section.
    ///
    /// If the road belongs to an XODR junction, use that junction's ID.
    /// Otherwise, create a junction ID from the road ID and lane section index.
    fn compute_junction_id(
        &self,
        road: &RoadHeader,
        section_idx: usize,
        xodr_junctions: &HashMap<&str, &XodrJunction>,
    ) -> String {
        // Check if road belongs to an XODR junction
        if let Some(ref junction_ref) = road.junction {
            if junction_ref != "-1" && xodr_junctions.contains_key(junction_ref.as_str()) {
                return junction_ref.clone();
            }
        }
        
        // Create junction ID from road ID and section index
        format!("{}_{}", road.id, section_idx)
    }

    /// Builds lanes for a segment.
    ///
    /// Lanes are built from center outward, with right lanes reversed so that
    /// the maliput index 0 is the rightmost lane.
    fn build_lanes_for_segment(
        &self,
        road: &RoadHeader,
        section_idx: usize,
        lane_section: &LaneSection,
        road_curve: &Arc<RoadCurve>,
        segment: &Arc<MalidriveSegment>,
        track_s_start: f64,
        track_s_end: f64,
    ) -> MalidriveResult<Vec<LaneInfo>> {
        let mut lane_infos = Vec::new();
        let mut lane_index = 0;

        // Get drivable lanes
        let right_lanes = self.get_drivable_lanes(&lane_section.right_lanes);
        let left_lanes = self.get_drivable_lanes(&lane_section.left_lanes);

        // Build right lanes in reverse order (from innermost -1 to outermost -N)
        // But we want index 0 to be rightmost, so we iterate in reverse
        for xodr_lane in right_lanes.iter().rev() {
            let lane = self.build_lane(
                road,
                section_idx,
                xodr_lane,
                lane_index,
                road_curve,
                segment,
                track_s_start,
                track_s_end,
                lane_section,
            )?;

            lane_infos.push(LaneInfo {
                lane,
                xodr_road_id: road.id.clone(),
                xodr_lane_section_index: section_idx,
                xodr_lane_id: xodr_lane.id,
                track_s_start,
                track_s_end,
            });
            lane_index += 1;
        }

        // Build left lanes (from innermost 1 to outermost N)
        for xodr_lane in &left_lanes {
            let lane = self.build_lane(
                road,
                section_idx,
                xodr_lane,
                lane_index,
                road_curve,
                segment,
                track_s_start,
                track_s_end,
                lane_section,
            )?;

            lane_infos.push(LaneInfo {
                lane,
                xodr_road_id: road.id.clone(),
                xodr_lane_section_index: section_idx,
                xodr_lane_id: xodr_lane.id,
                track_s_start,
                track_s_end,
            });
            lane_index += 1;
        }

        Ok(lane_infos)
    }

    /// Returns drivable lanes from a list of lanes.
    fn get_drivable_lanes<'a>(&self, lanes: &'a [XodrLane]) -> Vec<&'a XodrLane> {
        lanes
            .iter()
            .filter(|lane| {
                if self.omit_nondrivable_lanes {
                    Self::is_drivable_lane_type(lane.lane_type)
                } else {
                    true
                }
            })
            .collect()
    }

    /// Checks if a lane type is drivable.
    fn is_drivable_lane_type(lane_type: LaneType) -> bool {
        matches!(
            lane_type,
            LaneType::Driving
                | LaneType::Entry
                | LaneType::Exit
                | LaneType::OffRamp
                | LaneType::OnRamp
                | LaneType::Parking
                | LaneType::Stop
                | LaneType::Shoulder
                | LaneType::Biking
                | LaneType::Sidewalk
                | LaneType::Bidirectional
                | LaneType::ConnectingRamp
                | LaneType::MwyEntry
                | LaneType::MwyExit
        )
    }

    /// Builds a single lane.
    #[allow(clippy::too_many_arguments)]
    fn build_lane(
        &self,
        road: &RoadHeader,
        section_idx: usize,
        xodr_lane: &XodrLane,
        lane_index: usize,
        road_curve: &Arc<RoadCurve>,
        segment: &Arc<MalidriveSegment>,
        track_s_start: f64,
        track_s_end: f64,
        lane_section: &LaneSection,
    ) -> MalidriveResult<Arc<MalidriveLane>> {
        // Compute lane ID: road_lane_section_lane
        let lane_id = format!("{}_{}_{}", road.id, section_idx, xodr_lane.id);

        // Compute lane offset (r coordinate of lane center relative to road reference line)
        let lane_offset = self.compute_lane_offset(xodr_lane, lane_section, track_s_start, track_s_end)?;

        let lane = MalidriveLane::new(
            LaneId::new(&lane_id),
            lane_index,
            Arc::downgrade(segment) as Weak<dyn maliput::api::Segment>,
            Arc::clone(road_curve),
            lane_offset,
            track_s_start,
            track_s_end,
            self.params.linear_tolerance,
            self.params.angular_tolerance,
        );

        Ok(Arc::new(lane))
    }

    /// Computes the lane offset (SimpleLaneOffset) for a lane.
    fn compute_lane_offset(
        &self,
        xodr_lane: &XodrLane,
        lane_section: &LaneSection,
        _track_s_start: f64,
        _track_s_end: f64,
    ) -> MalidriveResult<SimpleLaneOffset> {
        // Compute the offset from the reference line to the lane center
        // This requires summing up widths of lanes between the reference line and this lane
        
        let lane_id = xodr_lane.id;
        let mut offset = 0.0;
        let mut width = 3.5; // Default width if not specified

        if lane_id > 0 {
            // Left lane - offset is positive
            // Sum widths from lane 1 to lane_id - 1, plus half of this lane's width
            for id in 1..lane_id {
                if let Some(lane) = lane_section.lane_by_id(id) {
                    offset += lane.width_at(0.0);
                }
            }
            width = xodr_lane.width_at(0.0);
            offset += width / 2.0;
        } else if lane_id < 0 {
            // Right lane - offset is negative
            // Sum widths from lane -1 to lane_id + 1, plus half of this lane's width
            for id in (lane_id + 1)..0 {
                if let Some(lane) = lane_section.lane_by_id(id) {
                    offset -= lane.width_at(0.0);
                }
            }
            width = xodr_lane.width_at(0.0);
            offset -= width / 2.0;
        }

        Ok(SimpleLaneOffset::new(offset, width))
    }

    /// Builds branch points for all lanes.
    fn build_branch_points(
        &self,
        lane_infos: &[LaneInfo],
        road_geometry: &Arc<MalidriveRoadGeometry>,
    ) -> MalidriveResult<Vec<Arc<dyn maliput::api::BranchPoint>>> {
        let mut branch_points: Vec<Arc<MalidriveBranchPoint>> = Vec::new();
        let mut branch_point_counter = 0;
        let rg_weak = Arc::downgrade(road_geometry) as Weak<dyn maliput::api::RoadGeometry>;

        // Create branch points for each lane end
        // In a complete implementation, we would find connecting lanes and share branch points
        // For now, create a branch point for each unique lane end

        for lane_info in lane_infos {
            // Start branch point
            let bp_start_id = format!("bp_{}", branch_point_counter);
            branch_point_counter += 1;
            let mut bp_start = MalidriveBranchPoint::new(
                BranchPointId::new(&bp_start_id),
                Weak::clone(&rg_weak),
            );
            bp_start.add_a_side(
                Arc::downgrade(&lane_info.lane) as Weak<dyn maliput::api::Lane>,
                LaneEndWhich::Start,
            );
            branch_points.push(Arc::new(bp_start));

            // Finish branch point
            let bp_finish_id = format!("bp_{}", branch_point_counter);
            branch_point_counter += 1;
            let mut bp_finish = MalidriveBranchPoint::new(
                BranchPointId::new(&bp_finish_id),
                Weak::clone(&rg_weak),
            );
            bp_finish.add_a_side(
                Arc::downgrade(&lane_info.lane) as Weak<dyn maliput::api::Lane>,
                LaneEndWhich::Finish,
            );
            branch_points.push(Arc::new(bp_finish));
        }

        Ok(branch_points
            .into_iter()
            .map(|bp| bp as Arc<dyn maliput::api::BranchPoint>)
            .collect())
    }

    /// Returns the builder parameters.
    pub fn params(&self) -> &BuilderParams {
        &self.params
    }
}

/// Convenience function to build a road geometry from OpenDRIVE data.
pub fn build_road_geometry(
    params: BuilderParams,
    roads: &[RoadHeader],
    junctions: &[XodrJunction],
) -> MalidriveResult<Arc<MalidriveRoadGeometry>> {
    let builder = RoadGeometryBuilder::new(params);
    builder.build(roads, junctions)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_creation() {
        let params = BuilderParams::new("test.xodr").with_linear_tolerance(1e-3);

        let builder = RoadGeometryBuilder::new(params);
        assert_eq!(builder.params().opendrive_file, "test.xodr");
        assert_eq!(builder.params().linear_tolerance, 1e-3);
    }

    #[test]
    fn test_build_empty_road_geometry() {
        let params = BuilderParams::new("test.xodr");
        let builder = RoadGeometryBuilder::new(params);

        let rg = builder.build(&[], &[]).unwrap();
        assert_eq!(rg.num_junctions(), 0);
    }

    #[test]
    fn test_is_drivable_lane_type() {
        assert!(RoadGeometryBuilder::is_drivable_lane_type(LaneType::Driving));
        assert!(RoadGeometryBuilder::is_drivable_lane_type(LaneType::Entry));
        assert!(RoadGeometryBuilder::is_drivable_lane_type(LaneType::Parking));
        assert!(!RoadGeometryBuilder::is_drivable_lane_type(LaneType::Border));
        assert!(!RoadGeometryBuilder::is_drivable_lane_type(LaneType::Curb));
        assert!(!RoadGeometryBuilder::is_drivable_lane_type(LaneType::None));
    }
}
