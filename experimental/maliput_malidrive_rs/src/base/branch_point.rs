//! BranchPoint implementation for maliput_malidrive.

use std::sync::{Arc, Weak};

use maliput::api::{BranchPoint, BranchPointId, Lane, LaneEnd, LaneEndSet, MaliputError, MaliputResult, RoadGeometry};

/// A concrete BranchPoint implementation for maliput_malidrive.
pub struct MalidriveBranchPoint {
    /// BranchPoint identifier.
    id: BranchPointId,
    /// The road geometry containing this branch point.
    road_geometry: Weak<dyn RoadGeometry>,
    /// The A-side lane ends.
    a_side: MalidriveLaneEndSet,
    /// The B-side lane ends.
    b_side: MalidriveLaneEndSet,
}

impl MalidriveBranchPoint {
    /// Creates a new MalidriveBranchPoint.
    pub fn new(id: BranchPointId, road_geometry: Weak<dyn RoadGeometry>) -> Self {
        Self {
            id,
            road_geometry,
            a_side: MalidriveLaneEndSet::new(),
            b_side: MalidriveLaneEndSet::new(),
        }
    }

    /// Adds a lane end to the A-side.
    pub fn add_a_side(&mut self, lane: Weak<dyn Lane>, end: LaneEndWhich) {
        self.a_side.add(lane, end);
    }

    /// Adds a lane end to the B-side.
    pub fn add_b_side(&mut self, lane: Weak<dyn Lane>, end: LaneEndWhich) {
        self.b_side.add(lane, end);
    }

    /// Returns the A-side set mutably.
    pub fn a_side_mut(&mut self) -> &mut MalidriveLaneEndSet {
        &mut self.a_side
    }

    /// Returns the B-side set mutably.
    pub fn b_side_mut(&mut self) -> &mut MalidriveLaneEndSet {
        &mut self.b_side
    }
}

/// Which end of a lane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneEndWhich {
    /// The start of the lane (s = 0).
    Start,
    /// The finish of the lane (s = length).
    Finish,
}

impl BranchPoint for MalidriveBranchPoint {
    fn id(&self) -> &BranchPointId {
        &self.id
    }

    fn road_geometry(&self) -> Arc<dyn RoadGeometry> {
        self.road_geometry
            .upgrade()
            .expect("RoadGeometry has been dropped")
    }

    fn get_confluent_branches(&self, end: &LaneEnd) -> MaliputResult<Arc<dyn LaneEndSet>> {
        // Determine which side the given lane end is on
        if self.a_side.contains(end) {
            Ok(Arc::new(self.a_side.clone()) as Arc<dyn LaneEndSet>)
        } else if self.b_side.contains(end) {
            Ok(Arc::new(self.b_side.clone()) as Arc<dyn LaneEndSet>)
        } else {
            Err(MaliputError::Validation(format!(
                "LaneEnd not found in BranchPoint {}",
                self.id.string()
            )))
        }
    }

    fn get_ongoing_branches(&self, end: &LaneEnd) -> MaliputResult<Arc<dyn LaneEndSet>> {
        // Return the opposite side
        if self.a_side.contains(end) {
            Ok(Arc::new(self.b_side.clone()) as Arc<dyn LaneEndSet>)
        } else if self.b_side.contains(end) {
            Ok(Arc::new(self.a_side.clone()) as Arc<dyn LaneEndSet>)
        } else {
            Err(MaliputError::Validation(format!(
                "LaneEnd not found in BranchPoint {}",
                self.id.string()
            )))
        }
    }

    fn get_default_branch(&self, end: &LaneEnd) -> MaliputResult<Option<LaneEnd>> {
        // For now, return the first ongoing branch if available
        let ongoing = self.get_ongoing_branches(end)?;
        if ongoing.size() > 0 {
            Ok(Some(ongoing.get(0)?))
        } else {
            Ok(None)
        }
    }

    fn get_a_side(&self) -> Arc<dyn LaneEndSet> {
        Arc::new(self.a_side.clone())
    }

    fn get_b_side(&self) -> Arc<dyn LaneEndSet> {
        Arc::new(self.b_side.clone())
    }
}

impl std::fmt::Debug for MalidriveBranchPoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MalidriveBranchPoint")
            .field("id", &self.id)
            .field("a_side_count", &self.a_side.size())
            .field("b_side_count", &self.b_side.size())
            .finish()
    }
}

/// A concrete LaneEndSet implementation.
#[derive(Debug, Clone)]
pub struct MalidriveLaneEndSet {
    /// The lane ends in this set.
    lane_ends: Vec<(Weak<dyn Lane>, LaneEndWhich)>,
}

impl MalidriveLaneEndSet {
    /// Creates a new empty LaneEndSet.
    pub fn new() -> Self {
        Self {
            lane_ends: Vec::new(),
        }
    }

    /// Adds a lane end to this set.
    pub fn add(&mut self, lane: Weak<dyn Lane>, end: LaneEndWhich) {
        self.lane_ends.push((lane, end));
    }

    /// Checks if this set contains the given lane end.
    pub fn contains(&self, end: &LaneEnd) -> bool {
        self.lane_ends.iter().any(|(lane, which)| {
            if let Some(l) = lane.upgrade() {
                let matches_lane = l.id() == end.lane.id();
                let matches_end = matches!(
                    (which, &end.end),
                    (LaneEndWhich::Start, maliput::api::LaneEndWhich::Start)
                        | (LaneEndWhich::Finish, maliput::api::LaneEndWhich::Finish)
                );
                matches_lane && matches_end
            } else {
                false
            }
        })
    }
}

impl Default for MalidriveLaneEndSet {
    fn default() -> Self {
        Self::new()
    }
}

impl LaneEndSet for MalidriveLaneEndSet {
    fn size(&self) -> usize {
        self.lane_ends.len()
    }

    fn get(&self, index: usize) -> MaliputResult<LaneEnd> {
        let (lane, which) = self.lane_ends.get(index)
            .ok_or_else(|| MaliputError::IndexOutOfBounds {
                index,
                max: self.lane_ends.len().saturating_sub(1),
            })?;
        let lane_arc = lane.upgrade()
            .ok_or_else(|| MaliputError::Validation("Lane has been dropped".to_string()))?;
        let api_which = match which {
            LaneEndWhich::Start => maliput::api::LaneEndWhich::Start,
            LaneEndWhich::Finish => maliput::api::LaneEndWhich::Finish,
        };
        Ok(LaneEnd::new(lane_arc, api_which))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_branch_point_creation() {
        let id = BranchPointId::new("bp_1".to_string());
        assert_eq!(id.string(), "bp_1");
    }

    #[test]
    fn test_lane_end_set_creation() {
        let set = MalidriveLaneEndSet::new();
        assert_eq!(set.size(), 0);
    }
}
