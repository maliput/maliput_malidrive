//! XODR Lane Section types.
//!
//! A lane section groups lanes that share the same s-coordinate range within a road.
//! Lane sections are separated by changes in the lane configuration.

use crate::common::{MalidriveError, MalidriveResult};
use crate::xodr::Lane;

/// Lane section containing left, center, and right lanes.
///
/// In OpenDRIVE, lanes are organized relative to the road reference line:
/// - Left lanes: Positive IDs (1, 2, 3, ...) going outward from center
/// - Center lane: ID = 0 (the reference line itself, no width)
/// - Right lanes: Negative IDs (-1, -2, -3, ...) going outward from center
#[derive(Debug, Clone, PartialEq)]
pub struct LaneSection {
    /// Start s-coordinate of this lane section.
    pub s: f64,
    /// Whether this is a single-side lane section.
    pub single_side: bool,
    /// Left lanes (positive IDs, sorted by ID ascending: 1, 2, 3, ...).
    pub left_lanes: Vec<Lane>,
    /// Center lane (ID = 0).
    pub center_lane: Lane,
    /// Right lanes (negative IDs, sorted by ID descending: -1, -2, -3, ...).
    pub right_lanes: Vec<Lane>,
}

impl LaneSection {
    /// Creates a new lane section at the given s-coordinate.
    pub fn new(s: f64) -> Self {
        Self {
            s,
            single_side: false,
            left_lanes: Vec::new(),
            center_lane: Lane::new_center(),
            right_lanes: Vec::new(),
        }
    }

    /// Returns all lanes in order: left (highest ID first), center, right (lowest ID first).
    pub fn all_lanes(&self) -> Vec<&Lane> {
        let mut lanes = Vec::with_capacity(self.left_lanes.len() + 1 + self.right_lanes.len());
        
        // Left lanes in reverse order (highest ID first)
        for lane in self.left_lanes.iter().rev() {
            lanes.push(lane);
        }
        
        // Center lane
        lanes.push(&self.center_lane);
        
        // Right lanes
        for lane in &self.right_lanes {
            lanes.push(lane);
        }
        
        lanes
    }

    /// Returns a lane by its ID.
    pub fn lane_by_id(&self, id: i32) -> Option<&Lane> {
        if id == 0 {
            Some(&self.center_lane)
        } else if id > 0 {
            self.left_lanes.iter().find(|l| l.id == id)
        } else {
            self.right_lanes.iter().find(|l| l.id == id)
        }
    }

    /// Returns a mutable reference to a lane by its ID.
    pub fn lane_by_id_mut(&mut self, id: i32) -> Option<&mut Lane> {
        if id == 0 {
            Some(&mut self.center_lane)
        } else if id > 0 {
            self.left_lanes.iter_mut().find(|l| l.id == id)
        } else {
            self.right_lanes.iter_mut().find(|l| l.id == id)
        }
    }

    /// Adds a lane to the appropriate side based on its ID.
    pub fn add_lane(&mut self, lane: Lane) -> MalidriveResult<()> {
        if lane.id == 0 {
            self.center_lane = lane;
        } else if lane.id > 0 {
            // Insert maintaining ascending order by ID
            let pos = self.left_lanes.iter().position(|l| l.id > lane.id).unwrap_or(self.left_lanes.len());
            self.left_lanes.insert(pos, lane);
        } else {
            // Insert maintaining descending order by ID (so -1 comes before -2)
            let pos = self.right_lanes.iter().position(|l| l.id < lane.id).unwrap_or(self.right_lanes.len());
            self.right_lanes.insert(pos, lane);
        }
        Ok(())
    }

    /// Returns the number of lanes (excluding center).
    pub fn num_lanes(&self) -> usize {
        self.left_lanes.len() + self.right_lanes.len()
    }

    /// Returns the total number of lanes (including center).
    pub fn total_lanes(&self) -> usize {
        self.num_lanes() + 1
    }

    /// Returns the outermost left lane ID (highest positive ID), or None if no left lanes.
    pub fn outermost_left_id(&self) -> Option<i32> {
        self.left_lanes.last().map(|l| l.id)
    }

    /// Returns the outermost right lane ID (lowest negative ID), or None if no right lanes.
    pub fn outermost_right_id(&self) -> Option<i32> {
        self.right_lanes.last().map(|l| l.id)
    }

    /// Returns the innermost left lane (ID = 1) if it exists.
    pub fn innermost_left(&self) -> Option<&Lane> {
        self.left_lanes.first()
    }

    /// Returns the innermost right lane (ID = -1) if it exists.
    pub fn innermost_right(&self) -> Option<&Lane> {
        self.right_lanes.first()
    }

    /// Validates the lane section.
    pub fn validate(&self) -> MalidriveResult<()> {
        // Check that center lane has ID 0
        if self.center_lane.id != 0 {
            return Err(MalidriveError::ValidationError(
                "Center lane must have ID = 0".to_string(),
            ));
        }

        // Check that left lanes have positive IDs in ascending order
        let mut prev_id = 0;
        for lane in &self.left_lanes {
            if lane.id <= prev_id {
                return Err(MalidriveError::ValidationError(format!(
                    "Left lane IDs must be positive and ascending, got {} after {}",
                    lane.id, prev_id
                )));
            }
            prev_id = lane.id;
        }

        // Check that right lanes have negative IDs in descending order
        let mut prev_id = 0;
        for lane in &self.right_lanes {
            if lane.id >= prev_id {
                return Err(MalidriveError::ValidationError(format!(
                    "Right lane IDs must be negative and descending, got {} after {}",
                    lane.id, prev_id
                )));
            }
            prev_id = lane.id;
        }

        Ok(())
    }
}

impl Default for LaneSection {
    fn default() -> Self {
        Self::new(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xodr::LaneType;

    #[test]
    fn test_lane_section_creation() {
        let section = LaneSection::new(10.0);
        assert_eq!(section.s, 10.0);
        assert!(section.left_lanes.is_empty());
        assert!(section.right_lanes.is_empty());
        assert_eq!(section.center_lane.id, 0);
    }

    #[test]
    fn test_lane_section_add_lanes() {
        let mut section = LaneSection::new(0.0);
        
        // Add left lanes
        section.add_lane(Lane::new(1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(2, LaneType::Shoulder)).unwrap();
        
        // Add right lanes
        section.add_lane(Lane::new(-1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(-2, LaneType::Shoulder)).unwrap();

        assert_eq!(section.left_lanes.len(), 2);
        assert_eq!(section.right_lanes.len(), 2);
        assert_eq!(section.num_lanes(), 4);
        assert_eq!(section.total_lanes(), 5);
    }

    #[test]
    fn test_lane_section_lane_order() {
        let mut section = LaneSection::new(0.0);
        
        // Add in random order
        section.add_lane(Lane::new(2, LaneType::Shoulder)).unwrap();
        section.add_lane(Lane::new(-2, LaneType::Shoulder)).unwrap();
        section.add_lane(Lane::new(1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(-1, LaneType::Driving)).unwrap();

        // Check left lanes are sorted ascending: 1, 2
        assert_eq!(section.left_lanes[0].id, 1);
        assert_eq!(section.left_lanes[1].id, 2);

        // Check right lanes are sorted descending: -1, -2
        assert_eq!(section.right_lanes[0].id, -1);
        assert_eq!(section.right_lanes[1].id, -2);
    }

    #[test]
    fn test_lane_section_all_lanes() {
        let mut section = LaneSection::new(0.0);
        section.add_lane(Lane::new(2, LaneType::Shoulder)).unwrap();
        section.add_lane(Lane::new(1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(-1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(-2, LaneType::Shoulder)).unwrap();

        let all = section.all_lanes();
        assert_eq!(all.len(), 5);
        // Order: 2, 1, 0, -1, -2
        assert_eq!(all[0].id, 2);
        assert_eq!(all[1].id, 1);
        assert_eq!(all[2].id, 0);
        assert_eq!(all[3].id, -1);
        assert_eq!(all[4].id, -2);
    }

    #[test]
    fn test_lane_section_lane_by_id() {
        let mut section = LaneSection::new(0.0);
        section.add_lane(Lane::new(1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(-1, LaneType::Driving)).unwrap();

        assert!(section.lane_by_id(0).is_some());
        assert!(section.lane_by_id(1).is_some());
        assert!(section.lane_by_id(-1).is_some());
        assert!(section.lane_by_id(2).is_none());
    }

    #[test]
    fn test_lane_section_outermost() {
        let mut section = LaneSection::new(0.0);
        section.add_lane(Lane::new(1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(2, LaneType::Shoulder)).unwrap();
        section.add_lane(Lane::new(-1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(-2, LaneType::Shoulder)).unwrap();
        section.add_lane(Lane::new(-3, LaneType::Border)).unwrap();

        assert_eq!(section.outermost_left_id(), Some(2));
        assert_eq!(section.outermost_right_id(), Some(-3));
    }

    #[test]
    fn test_lane_section_validation() {
        let section = LaneSection::new(0.0);
        assert!(section.validate().is_ok());

        let mut section = LaneSection::new(0.0);
        section.add_lane(Lane::new(1, LaneType::Driving)).unwrap();
        section.add_lane(Lane::new(-1, LaneType::Driving)).unwrap();
        assert!(section.validate().is_ok());
    }
}
