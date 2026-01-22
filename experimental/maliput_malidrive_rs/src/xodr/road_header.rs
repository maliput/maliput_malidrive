//! XODR Road Header types.
//!
//! The road header contains the main road attributes including ID, name, length,
//! junction association, and references to the road geometry and lane structure.

use crate::common::{MalidriveError, MalidriveResult};
use crate::xodr::{
    ElevationProfile, Geometry, LaneOffset, LaneSection, LateralProfile, RoadLink, RoadType,
};

/// Hand traffic rule (driving side).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HandTrafficRule {
    /// Right-hand traffic (drive on the right, e.g., USA, Europe).
    #[default]
    Rht,
    /// Left-hand traffic (drive on the left, e.g., UK, Japan, Australia).
    Lht,
}

impl std::fmt::Display for HandTrafficRule {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HandTrafficRule::Rht => write!(f, "RHT"),
            HandTrafficRule::Lht => write!(f, "LHT"),
        }
    }
}

impl std::str::FromStr for HandTrafficRule {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_uppercase().as_str() {
            "RHT" => Ok(HandTrafficRule::Rht),
            "LHT" => Ok(HandTrafficRule::Lht),
            _ => Err(MalidriveError::InvalidAttribute {
                attribute: "rule".to_string(),
                value: s.to_string(),
            }),
        }
    }
}

/// Reference geometry containing plan view geometries for a road.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PlanView {
    /// Geometry elements that compose the road's reference line.
    pub geometries: Vec<Geometry>,
}

impl PlanView {
    /// Creates a new empty plan view.
    pub fn new() -> Self {
        Self {
            geometries: Vec::new(),
        }
    }

    /// Adds a geometry element.
    pub fn add_geometry(&mut self, geometry: Geometry) {
        self.geometries.push(geometry);
    }

    /// Returns the total length of all geometries.
    pub fn total_length(&self) -> f64 {
        self.geometries.iter().map(|g| g.length).sum()
    }

    /// Returns the geometry element that contains the given s-coordinate.
    pub fn geometry_at(&self, s: f64) -> Option<&Geometry> {
        self.geometries.iter().find(|g| s >= g.s_0 && s <= g.s_end())
    }

    /// Returns true if the plan view is empty.
    pub fn is_empty(&self) -> bool {
        self.geometries.is_empty()
    }
}

/// Lanes structure containing lane offsets and lane sections.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Lanes {
    /// Lane offset records.
    pub lane_offsets: Vec<LaneOffset>,
    /// Lane sections.
    pub lane_sections: Vec<LaneSection>,
}

impl Lanes {
    /// Creates a new empty lanes structure.
    pub fn new() -> Self {
        Self {
            lane_offsets: Vec::new(),
            lane_sections: Vec::new(),
        }
    }

    /// Adds a lane offset record.
    pub fn add_lane_offset(&mut self, offset: LaneOffset) {
        // Insert maintaining sorted order by s
        let pos = self
            .lane_offsets
            .iter()
            .position(|o| o.s > offset.s)
            .unwrap_or(self.lane_offsets.len());
        self.lane_offsets.insert(pos, offset);
    }

    /// Adds a lane section.
    pub fn add_lane_section(&mut self, section: LaneSection) {
        // Insert maintaining sorted order by s
        let pos = self
            .lane_sections
            .iter()
            .position(|s| s.s > section.s)
            .unwrap_or(self.lane_sections.len());
        self.lane_sections.insert(pos, section);
    }

    /// Returns the lane offset at the given s-coordinate.
    pub fn lane_offset_at(&self, s: f64) -> f64 {
        // Find the last lane offset with s <= given s
        self.lane_offsets
            .iter()
            .rev()
            .find(|o| o.s <= s)
            .map(|o| o.offset_at(s))
            .unwrap_or(0.0)
    }

    /// Returns the lane section that contains the given s-coordinate.
    pub fn lane_section_at(&self, s: f64) -> Option<&LaneSection> {
        // Find the last lane section with s <= given s
        self.lane_sections.iter().rev().find(|sec| sec.s <= s)
    }

    /// Returns the lane section index for a given s-coordinate.
    pub fn lane_section_index_at(&self, s: f64) -> Option<usize> {
        self.lane_sections
            .iter()
            .enumerate()
            .rev()
            .find(|(_, sec)| sec.s <= s)
            .map(|(i, _)| i)
    }

    /// Returns the length of a lane section by index.
    pub fn lane_section_length(&self, index: usize, road_length: f64) -> Option<f64> {
        if index >= self.lane_sections.len() {
            return None;
        }

        let start_s = self.lane_sections[index].s;
        let end_s = if index + 1 < self.lane_sections.len() {
            self.lane_sections[index + 1].s
        } else {
            road_length
        };

        Some(end_s - start_s)
    }
}

/// XODR Road header containing all road information.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadHeader {
    /// Road ID.
    pub id: String,
    /// Road name (optional).
    pub name: Option<String>,
    /// Total length of the road.
    pub length: f64,
    /// Junction ID this road belongs to (-1 if not part of a junction).
    pub junction: String,
    /// Hand traffic rule.
    pub rule: HandTrafficRule,
    /// Road link (predecessor/successor).
    pub link: Option<RoadLink>,
    /// Road types along the road.
    pub road_types: Vec<RoadType>,
    /// Plan view (reference line geometry).
    pub plan_view: PlanView,
    /// Elevation profile.
    pub elevation_profile: ElevationProfile,
    /// Lateral profile (superelevation).
    pub lateral_profile: LateralProfile,
    /// Lanes structure.
    pub lanes: Lanes,
}

impl RoadHeader {
    /// Creates a new road header with the given ID and length.
    pub fn new(id: impl Into<String>, length: f64) -> Self {
        Self {
            id: id.into(),
            name: None,
            length,
            junction: "-1".to_string(),
            rule: HandTrafficRule::default(),
            link: None,
            road_types: Vec::new(),
            plan_view: PlanView::new(),
            elevation_profile: ElevationProfile::default(),
            lateral_profile: LateralProfile::default(),
            lanes: Lanes::new(),
        }
    }

    /// Builder method to set the road name.
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Builder method to set the junction ID.
    pub fn with_junction(mut self, junction: impl Into<String>) -> Self {
        self.junction = junction.into();
        self
    }

    /// Builder method to set the road link.
    pub fn with_link(mut self, link: RoadLink) -> Self {
        self.link = Some(link);
        self
    }

    /// Builder method to set the plan view.
    pub fn with_plan_view(mut self, plan_view: PlanView) -> Self {
        self.plan_view = plan_view;
        self
    }

    /// Builder method to set the elevation profile.
    pub fn with_elevation_profile(mut self, profile: ElevationProfile) -> Self {
        self.elevation_profile = profile;
        self
    }

    /// Builder method to set the lateral profile.
    pub fn with_lateral_profile(mut self, profile: LateralProfile) -> Self {
        self.lateral_profile = profile;
        self
    }

    /// Builder method to set the lanes structure.
    pub fn with_lanes(mut self, lanes: Lanes) -> Self {
        self.lanes = lanes;
        self
    }

    /// Returns true if this road is part of a junction.
    pub fn is_junction_road(&self) -> bool {
        self.junction != "-1"
    }

    /// Returns the elevation at the given s-coordinate.
    pub fn elevation_at(&self, s: f64) -> f64 {
        self.elevation_profile.elevation_at(s)
    }

    /// Returns the superelevation at the given s-coordinate.
    pub fn superelevation_at(&self, s: f64) -> f64 {
        self.lateral_profile.superelevation_at(s)
    }

    /// Returns the lane offset at the given s-coordinate.
    pub fn lane_offset_at(&self, s: f64) -> f64 {
        self.lanes.lane_offset_at(s)
    }

    /// Returns the lane section length by index.
    pub fn lane_section_length(&self, index: usize) -> Option<f64> {
        self.lanes.lane_section_length(index, self.length)
    }

    /// Returns the lane section index for a given s-coordinate.
    pub fn lane_section_index(&self, s: f64) -> Option<usize> {
        self.lanes.lane_section_index_at(s)
    }

    /// Returns the road type at the given s-coordinate.
    pub fn road_type_at(&self, s: f64) -> Option<&RoadType> {
        self.road_types.iter().rev().find(|rt| rt.s <= s)
    }

    /// Validates the road header.
    pub fn validate(&self) -> MalidriveResult<()> {
        if self.length <= 0.0 {
            return Err(MalidriveError::ValidationError(format!(
                "Road {} has invalid length: {}",
                self.id, self.length
            )));
        }

        if self.plan_view.is_empty() {
            return Err(MalidriveError::ValidationError(format!(
                "Road {} has no plan view geometries",
                self.id
            )));
        }

        if self.lanes.lane_sections.is_empty() {
            return Err(MalidriveError::ValidationError(format!(
                "Road {} has no lane sections",
                self.id
            )));
        }

        // Validate plan view total length matches road length
        let plan_view_length = self.plan_view.total_length();
        let tolerance = 1e-3;
        if (plan_view_length - self.length).abs() > tolerance {
            return Err(MalidriveError::ValidationError(format!(
                "Road {} plan view length ({}) does not match road length ({})",
                self.id, plan_view_length, self.length
            )));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xodr::{Elevation, Geometry, Lane, LaneType, Superelevation};
    use nalgebra::Vector2;

    #[test]
    fn test_hand_traffic_rule() {
        assert_eq!("RHT".parse::<HandTrafficRule>().unwrap(), HandTrafficRule::Rht);
        assert_eq!("LHT".parse::<HandTrafficRule>().unwrap(), HandTrafficRule::Lht);
        assert!("INVALID".parse::<HandTrafficRule>().is_err());
    }

    #[test]
    fn test_road_header_creation() {
        let road = RoadHeader::new("1", 100.0);
        assert_eq!(road.id, "1");
        assert_eq!(road.length, 100.0);
        assert!(!road.is_junction_road());
    }

    #[test]
    fn test_road_header_junction() {
        let mut road = RoadHeader::new("1", 100.0);
        road.junction = "5".to_string();
        assert!(road.is_junction_road());
    }

    #[test]
    fn test_plan_view() {
        let mut plan_view = PlanView::new();
        plan_view.add_geometry(Geometry::new_line(0.0, Vector2::new(0.0, 0.0), 0.0, 50.0));
        plan_view.add_geometry(Geometry::new_line(50.0, Vector2::new(50.0, 0.0), 0.0, 50.0));

        assert_eq!(plan_view.total_length(), 100.0);
        assert!(plan_view.geometry_at(25.0).is_some());
        assert!(plan_view.geometry_at(75.0).is_some());
    }

    #[test]
    fn test_lanes_structure() {
        let mut lanes = Lanes::new();

        // Add lane offset
        lanes.add_lane_offset(LaneOffset::constant(0.0, 0.5));

        // Add lane section
        let mut section = LaneSection::new(0.0);
        section.add_lane(Lane::new(-1, LaneType::Driving)).unwrap();
        lanes.add_lane_section(section);

        assert_eq!(lanes.lane_offset_at(50.0), 0.5);
        assert!(lanes.lane_section_at(50.0).is_some());
    }

    #[test]
    fn test_road_header_profiles() {
        let mut road = RoadHeader::new("1", 100.0);

        // Add elevation
        road.elevation_profile.add_elevation(Elevation::linear(0.0, 0.0, 0.05));

        // Add superelevation
        road.lateral_profile
            .add_superelevation(Superelevation::constant(0.0, 0.02));

        assert_eq!(road.elevation_at(100.0), 5.0); // 0.05 * 100
        assert_eq!(road.superelevation_at(50.0), 0.02);
    }

    #[test]
    fn test_road_header_validation() {
        let mut road = RoadHeader::new("1", 100.0);

        // Invalid: no plan view
        assert!(road.validate().is_err());

        // Add plan view
        road.plan_view
            .add_geometry(Geometry::new_line(0.0, Vector2::new(0.0, 0.0), 0.0, 100.0));

        // Invalid: no lane sections
        assert!(road.validate().is_err());

        // Add lane section
        road.lanes.add_lane_section(LaneSection::new(0.0));

        // Now valid
        assert!(road.validate().is_ok());
    }
}
