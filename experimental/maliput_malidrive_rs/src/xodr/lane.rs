//! XODR Lane types and parsing.
//!
//! This module defines the lane types used in OpenDRIVE road descriptions.
//! Lanes are organized within lane sections and can have various properties
//! like width, speed limits, and road markings.

use crate::common::{MalidriveError, MalidriveResult};
use crate::xodr::{LaneLink, LaneWidth};

/// Types of lanes supported in OpenDRIVE.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LaneType {
    /// No specific type
    None,
    /// Driving lane (default for vehicles)
    #[default]
    Driving,
    /// Stop lane (for stopping/parking)
    Stop,
    /// Shoulder lane
    Shoulder,
    /// Biking lane
    Biking,
    /// Walking lane (pedestrians)
    Walking,
    /// Sidewalk
    Sidewalk,
    /// Border (non-drivable edge)
    Border,
    /// Restricted lane
    Restricted,
    /// Parking lane
    Parking,
    /// Curb
    Curb,
    /// Bidirectional lane (can be used in both directions)
    Bidirectional,
    /// Median (center divider)
    Median,
    /// Special purpose lanes
    Special1,
    Special2,
    Special3,
    /// Road works
    RoadWorks,
    /// Tram lane
    Tram,
    /// Rail lane
    Rail,
    /// Entry lane
    Entry,
    /// Exit lane
    Exit,
    /// Off ramp
    OffRamp,
    /// On ramp
    OnRamp,
    /// Connecting ramp
    ConnectingRamp,
    /// Slip lane
    SlipLane,
    /// Bus lane
    Bus,
    /// Taxi lane
    Taxi,
    /// High-occupancy vehicle lane
    Hov,
    /// Motorway entry
    MwyEntry,
    /// Motorway exit
    MwyExit,
}

impl std::fmt::Display for LaneType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LaneType::None => write!(f, "none"),
            LaneType::Driving => write!(f, "driving"),
            LaneType::Stop => write!(f, "stop"),
            LaneType::Shoulder => write!(f, "shoulder"),
            LaneType::Biking => write!(f, "biking"),
            LaneType::Walking => write!(f, "walking"),
            LaneType::Sidewalk => write!(f, "sidewalk"),
            LaneType::Border => write!(f, "border"),
            LaneType::Restricted => write!(f, "restricted"),
            LaneType::Parking => write!(f, "parking"),
            LaneType::Curb => write!(f, "curb"),
            LaneType::Bidirectional => write!(f, "bidirectional"),
            LaneType::Median => write!(f, "median"),
            LaneType::Special1 => write!(f, "special1"),
            LaneType::Special2 => write!(f, "special2"),
            LaneType::Special3 => write!(f, "special3"),
            LaneType::RoadWorks => write!(f, "roadWorks"),
            LaneType::Tram => write!(f, "tram"),
            LaneType::Rail => write!(f, "rail"),
            LaneType::Entry => write!(f, "entry"),
            LaneType::Exit => write!(f, "exit"),
            LaneType::OffRamp => write!(f, "offRamp"),
            LaneType::OnRamp => write!(f, "onRamp"),
            LaneType::ConnectingRamp => write!(f, "connectingRamp"),
            LaneType::SlipLane => write!(f, "slipLane"),
            LaneType::Bus => write!(f, "bus"),
            LaneType::Taxi => write!(f, "taxi"),
            LaneType::Hov => write!(f, "hov"),
            LaneType::MwyEntry => write!(f, "mwyEntry"),
            LaneType::MwyExit => write!(f, "mwyExit"),
        }
    }
}

impl std::str::FromStr for LaneType {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "none" => Ok(LaneType::None),
            "driving" => Ok(LaneType::Driving),
            "stop" => Ok(LaneType::Stop),
            "shoulder" => Ok(LaneType::Shoulder),
            "biking" => Ok(LaneType::Biking),
            "walking" => Ok(LaneType::Walking),
            "sidewalk" => Ok(LaneType::Sidewalk),
            "border" => Ok(LaneType::Border),
            "restricted" => Ok(LaneType::Restricted),
            "parking" => Ok(LaneType::Parking),
            "curb" => Ok(LaneType::Curb),
            "bidirectional" => Ok(LaneType::Bidirectional),
            "median" => Ok(LaneType::Median),
            "special1" => Ok(LaneType::Special1),
            "special2" => Ok(LaneType::Special2),
            "special3" => Ok(LaneType::Special3),
            "roadworks" => Ok(LaneType::RoadWorks),
            "tram" => Ok(LaneType::Tram),
            "rail" => Ok(LaneType::Rail),
            "entry" => Ok(LaneType::Entry),
            "exit" => Ok(LaneType::Exit),
            "offramp" => Ok(LaneType::OffRamp),
            "onramp" => Ok(LaneType::OnRamp),
            "connectingramp" => Ok(LaneType::ConnectingRamp),
            "sliplane" => Ok(LaneType::SlipLane),
            "bus" => Ok(LaneType::Bus),
            "taxi" => Ok(LaneType::Taxi),
            "hov" => Ok(LaneType::Hov),
            "mwyentry" => Ok(LaneType::MwyEntry),
            "mwyexit" => Ok(LaneType::MwyExit),
            _ => Err(MalidriveError::InvalidAttribute {
                attribute: "lane type".to_string(),
                value: s.to_string(),
            }),
        }
    }
}

impl LaneType {
    /// Returns true if this lane type is drivable by vehicles.
    pub fn is_drivable(&self) -> bool {
        matches!(
            self,
            LaneType::Driving
                | LaneType::Entry
                | LaneType::Exit
                | LaneType::OffRamp
                | LaneType::OnRamp
                | LaneType::ConnectingRamp
                | LaneType::Bidirectional
        )
    }
}

/// Lane speed record.
#[derive(Debug, Clone, PartialEq)]
pub struct LaneSpeed {
    /// Start position (s-offset within the lane section).
    pub s_offset: f64,
    /// Maximum speed value.
    pub max: f64,
    /// Speed unit.
    pub unit: crate::xodr::Unit,
}

impl LaneSpeed {
    /// Creates a new lane speed record.
    pub fn new(s_offset: f64, max: f64, unit: crate::xodr::Unit) -> Self {
        Self { s_offset, max, unit }
    }

    /// Returns the maximum speed in meters per second.
    pub fn max_m_per_s(&self) -> f64 {
        self.unit.to_m_per_s(self.max)
    }
}

/// XODR Lane definition.
///
/// A lane represents a single lane within a lane section. Lanes are
/// identified by integer IDs where:
/// - Positive IDs are left of the reference line
/// - Negative IDs are right of the reference line
/// - ID 0 is the center lane (reference line itself)
#[derive(Debug, Clone, PartialEq)]
pub struct Lane {
    /// Lane ID (negative = right, 0 = center, positive = left).
    pub id: i32,
    /// Lane type.
    pub lane_type: LaneType,
    /// Whether the lane maintains its level regardless of superelevation.
    pub level: bool,
    /// Lane link to predecessor/successor lanes.
    pub link: Option<LaneLink>,
    /// Lane width descriptions (polynomial).
    pub widths: Vec<LaneWidth>,
    /// Speed limits for this lane.
    pub speeds: Vec<LaneSpeed>,
}

impl Lane {
    /// Creates a new lane with the given ID and type.
    pub fn new(id: i32, lane_type: LaneType) -> Self {
        Self {
            id,
            lane_type,
            level: false,
            link: None,
            widths: Vec::new(),
            speeds: Vec::new(),
        }
    }

    /// Creates a new center lane (ID = 0).
    pub fn new_center() -> Self {
        Self::new(0, LaneType::None)
    }

    /// Returns true if this is the center lane.
    pub fn is_center(&self) -> bool {
        self.id == 0
    }

    /// Returns true if this is a left lane.
    pub fn is_left(&self) -> bool {
        self.id > 0
    }

    /// Returns true if this is a right lane.
    pub fn is_right(&self) -> bool {
        self.id < 0
    }

    /// Returns the width at the given s-offset within the lane section.
    pub fn width_at(&self, s_offset: f64) -> f64 {
        // Find the appropriate width polynomial
        for (i, width) in self.widths.iter().enumerate() {
            let next_s_offset = if i + 1 < self.widths.len() {
                self.widths[i + 1].s_offset
            } else {
                f64::MAX
            };

            if s_offset >= width.s_offset && s_offset < next_s_offset {
                let ds = s_offset - width.s_offset;
                return width.a + width.b * ds + width.c * ds * ds + width.d * ds * ds * ds;
            }
        }

        // Default to first width if no match
        if let Some(width) = self.widths.first() {
            let ds = s_offset - width.s_offset;
            width.a + width.b * ds + width.c * ds * ds + width.d * ds * ds * ds
        } else {
            0.0
        }
    }

    /// Validates the lane definition.
    pub fn validate(&self) -> MalidriveResult<()> {
        // Center lane must not have width definitions
        if self.is_center() && !self.widths.is_empty() {
            return Err(MalidriveError::ValidationError(
                "Center lane (id=0) should not have width definitions".to_string(),
            ));
        }

        // Non-center lanes should have at least one width definition
        if !self.is_center() && self.widths.is_empty() {
            return Err(MalidriveError::ValidationError(format!(
                "Lane {} must have at least one width definition",
                self.id
            )));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_lane_type_from_str() {
        assert_eq!(
            "driving".parse::<LaneType>().unwrap(),
            LaneType::Driving
        );
        assert_eq!(
            "shoulder".parse::<LaneType>().unwrap(),
            LaneType::Shoulder
        );
        assert_eq!("hov".parse::<LaneType>().unwrap(), LaneType::Hov);
        assert!("invalid".parse::<LaneType>().is_err());
    }

    #[test]
    fn test_lane_type_display() {
        assert_eq!(format!("{}", LaneType::Driving), "driving");
        assert_eq!(format!("{}", LaneType::Shoulder), "shoulder");
    }

    #[test]
    fn test_lane_type_is_drivable() {
        assert!(LaneType::Driving.is_drivable());
        assert!(LaneType::Entry.is_drivable());
        assert!(!LaneType::Shoulder.is_drivable());
        assert!(!LaneType::Sidewalk.is_drivable());
    }

    #[test]
    fn test_lane_creation() {
        let lane = Lane::new(-1, LaneType::Driving);
        assert_eq!(lane.id, -1);
        assert_eq!(lane.lane_type, LaneType::Driving);
        assert!(!lane.level);
        assert!(lane.is_right());
        assert!(!lane.is_left());
        assert!(!lane.is_center());
    }

    #[test]
    fn test_center_lane() {
        let lane = Lane::new_center();
        assert_eq!(lane.id, 0);
        assert!(lane.is_center());
        assert!(!lane.is_left());
        assert!(!lane.is_right());
    }

    #[test]
    fn test_lane_width_at() {
        let mut lane = Lane::new(-1, LaneType::Driving);
        lane.widths.push(LaneWidth::new(0.0, 3.5, 0.0, 0.0, 0.0));

        assert_relative_eq!(lane.width_at(0.0), 3.5);
        assert_relative_eq!(lane.width_at(50.0), 3.5);
    }

    #[test]
    fn test_lane_width_at_polynomial() {
        let mut lane = Lane::new(-1, LaneType::Driving);
        // Width varies linearly: w(s) = 3.0 + 0.1*s
        lane.widths.push(LaneWidth::new(0.0, 3.0, 0.1, 0.0, 0.0));

        assert_relative_eq!(lane.width_at(0.0), 3.0);
        assert_relative_eq!(lane.width_at(10.0), 4.0);
    }

    #[test]
    fn test_lane_speed() {
        let speed = LaneSpeed::new(0.0, 50.0, crate::xodr::Unit::Mph);
        assert_relative_eq!(speed.s_offset, 0.0);
        assert_relative_eq!(speed.max, 50.0);
        // 50 mph ≈ 22.352 m/s
        assert_relative_eq!(speed.max_m_per_s(), 22.352, epsilon = 0.001);
    }
}
