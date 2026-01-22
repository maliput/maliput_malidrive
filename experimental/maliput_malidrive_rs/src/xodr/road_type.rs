//! XODR Road Type definitions.
//!
//! Road types define the characteristics of road sections along the s-coordinate,
//! including speed limits and road classification.

use crate::common::MalidriveError;
use crate::xodr::Unit;

/// Road type classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RoadTypeType {
    /// Unknown road type.
    Unknown,
    /// Rural road.
    Rural,
    /// Motorway/freeway.
    Motorway,
    /// Town/urban road.
    #[default]
    Town,
    /// Low-speed road.
    LowSpeed,
    /// Pedestrian area.
    Pedestrian,
    /// Bicycle path.
    Bicycle,
    /// Town arterial road.
    TownArterial,
    /// Town collector road.
    TownCollector,
    /// Town express road.
    TownExpressway,
    /// Town local road.
    TownLocal,
    /// Town play street.
    TownPlayStreet,
    /// Town private road.
    TownPrivate,
}

impl std::fmt::Display for RoadTypeType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RoadTypeType::Unknown => write!(f, "unknown"),
            RoadTypeType::Rural => write!(f, "rural"),
            RoadTypeType::Motorway => write!(f, "motorway"),
            RoadTypeType::Town => write!(f, "town"),
            RoadTypeType::LowSpeed => write!(f, "lowSpeed"),
            RoadTypeType::Pedestrian => write!(f, "pedestrian"),
            RoadTypeType::Bicycle => write!(f, "bicycle"),
            RoadTypeType::TownArterial => write!(f, "townArterial"),
            RoadTypeType::TownCollector => write!(f, "townCollector"),
            RoadTypeType::TownExpressway => write!(f, "townExpressway"),
            RoadTypeType::TownLocal => write!(f, "townLocal"),
            RoadTypeType::TownPlayStreet => write!(f, "townPlayStreet"),
            RoadTypeType::TownPrivate => write!(f, "townPrivate"),
        }
    }
}

impl std::str::FromStr for RoadTypeType {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "unknown" => Ok(RoadTypeType::Unknown),
            "rural" => Ok(RoadTypeType::Rural),
            "motorway" => Ok(RoadTypeType::Motorway),
            "town" => Ok(RoadTypeType::Town),
            "lowspeed" => Ok(RoadTypeType::LowSpeed),
            "pedestrian" => Ok(RoadTypeType::Pedestrian),
            "bicycle" => Ok(RoadTypeType::Bicycle),
            "townarterial" => Ok(RoadTypeType::TownArterial),
            "towncollector" => Ok(RoadTypeType::TownCollector),
            "townexpressway" => Ok(RoadTypeType::TownExpressway),
            "townlocal" => Ok(RoadTypeType::TownLocal),
            "townplaystreet" => Ok(RoadTypeType::TownPlayStreet),
            "townprivate" => Ok(RoadTypeType::TownPrivate),
            _ => Err(MalidriveError::InvalidAttribute {
                attribute: "road type".to_string(),
                value: s.to_string(),
            }),
        }
    }
}

/// Speed limit record for a road type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Speed {
    /// Maximum speed value.
    pub max: f64,
    /// Speed unit.
    pub unit: Unit,
}

impl Speed {
    /// Creates a new speed limit.
    pub fn new(max: f64, unit: Unit) -> Self {
        Self { max, unit }
    }

    /// Creates a speed limit in km/h.
    pub fn kmh(max: f64) -> Self {
        Self::new(max, Unit::Kmh)
    }

    /// Creates a speed limit in mph.
    pub fn mph(max: f64) -> Self {
        Self::new(max, Unit::Mph)
    }

    /// Creates a speed limit in m/s.
    pub fn m_per_s(max: f64) -> Self {
        Self::new(max, Unit::Ms)
    }

    /// Returns the speed in m/s.
    pub fn max_m_per_s(&self) -> f64 {
        self.unit.to_m_per_s(self.max)
    }

    /// Returns the speed in km/h.
    pub fn max_kmh(&self) -> f64 {
        self.unit.to_kmh(self.max)
    }

    /// Returns the speed in mph.
    pub fn max_mph(&self) -> f64 {
        self.unit.to_mph(self.max)
    }
}

impl Default for Speed {
    fn default() -> Self {
        Self::kmh(50.0) // Default urban speed limit
    }
}

/// Road type record defining road characteristics for a section.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadType {
    /// Start s-coordinate of this road type section.
    pub s: f64,
    /// Road type classification.
    pub road_type: RoadTypeType,
    /// Country code (optional, ISO 3166-1 alpha-2 or alpha-3).
    pub country: Option<String>,
    /// Speed limit (optional).
    pub speed: Option<Speed>,
}

impl RoadType {
    /// Creates a new road type record.
    pub fn new(s: f64, road_type: RoadTypeType) -> Self {
        Self {
            s,
            road_type,
            country: None,
            speed: None,
        }
    }

    /// Creates a new road type with speed limit.
    pub fn with_speed(s: f64, road_type: RoadTypeType, speed: Speed) -> Self {
        Self {
            s,
            road_type,
            country: None,
            speed: Some(speed),
        }
    }

    /// Sets the country code.
    pub fn with_country(mut self, country: impl Into<String>) -> Self {
        self.country = Some(country.into());
        self
    }
}

impl Default for RoadType {
    fn default() -> Self {
        Self::new(0.0, RoadTypeType::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_road_type_type_from_str() {
        assert_eq!(
            "motorway".parse::<RoadTypeType>().unwrap(),
            RoadTypeType::Motorway
        );
        assert_eq!(
            "town".parse::<RoadTypeType>().unwrap(),
            RoadTypeType::Town
        );
        assert!("invalid".parse::<RoadTypeType>().is_err());
    }

    #[test]
    fn test_road_type_type_display() {
        assert_eq!(format!("{}", RoadTypeType::Motorway), "motorway");
        assert_eq!(format!("{}", RoadTypeType::Town), "town");
    }

    #[test]
    fn test_speed_creation() {
        let speed = Speed::kmh(100.0);
        assert_relative_eq!(speed.max, 100.0);
        assert_eq!(speed.unit, Unit::Kmh);

        let speed = Speed::mph(60.0);
        assert_relative_eq!(speed.max, 60.0);
        assert_eq!(speed.unit, Unit::Mph);
    }

    #[test]
    fn test_speed_conversion() {
        // 100 km/h
        let speed = Speed::kmh(100.0);
        assert_relative_eq!(speed.max_m_per_s(), 27.778, epsilon = 0.001);
        assert_relative_eq!(speed.max_kmh(), 100.0);
        assert_relative_eq!(speed.max_mph(), 62.137, epsilon = 0.001);

        // 60 mph
        let speed = Speed::mph(60.0);
        assert_relative_eq!(speed.max_m_per_s(), 26.822, epsilon = 0.001);
    }

    #[test]
    fn test_road_type() {
        let rt = RoadType::new(0.0, RoadTypeType::Motorway);
        assert_relative_eq!(rt.s, 0.0);
        assert_eq!(rt.road_type, RoadTypeType::Motorway);
        assert!(rt.speed.is_none());

        let rt = RoadType::with_speed(100.0, RoadTypeType::Town, Speed::kmh(50.0))
            .with_country("DE");
        assert_relative_eq!(rt.s, 100.0);
        assert_eq!(rt.road_type, RoadTypeType::Town);
        assert!(rt.speed.is_some());
        assert_eq!(rt.country, Some("DE".to_string()));
    }

    #[test]
    fn test_road_type_default() {
        let rt = RoadType::default();
        assert_relative_eq!(rt.s, 0.0);
        assert_eq!(rt.road_type, RoadTypeType::Town);
    }
}
