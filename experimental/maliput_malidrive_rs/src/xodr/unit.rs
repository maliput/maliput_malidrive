//! XODR Unit types.
//!
//! Units used for speed and distance measurements in OpenDRIVE.

use crate::common::MalidriveError;

/// Speed/distance units used in OpenDRIVE.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Unit {
    /// Meters per second.
    #[default]
    Ms,
    /// Miles per hour.
    Mph,
    /// Kilometers per hour.
    Kmh,
}

impl Unit {
    /// Conversion factor from mph to m/s.
    const MPH_TO_MS: f64 = 0.44704;
    /// Conversion factor from km/h to m/s.
    const KMH_TO_MS: f64 = 1.0 / 3.6;

    /// Converts a value from this unit to meters per second.
    pub fn to_m_per_s(&self, value: f64) -> f64 {
        match self {
            Unit::Ms => value,
            Unit::Mph => value * Self::MPH_TO_MS,
            Unit::Kmh => value * Self::KMH_TO_MS,
        }
    }

    /// Converts a value from this unit to kilometers per hour.
    pub fn to_kmh(&self, value: f64) -> f64 {
        match self {
            Unit::Ms => value * 3.6,
            Unit::Mph => value * 1.60934,
            Unit::Kmh => value,
        }
    }

    /// Converts a value from this unit to miles per hour.
    pub fn to_mph(&self, value: f64) -> f64 {
        match self {
            Unit::Ms => value / Self::MPH_TO_MS,
            Unit::Mph => value,
            Unit::Kmh => value / 1.60934,
        }
    }

    /// Creates a Unit from m/s and value, returning the value in the target unit.
    pub fn from_m_per_s(&self, ms_value: f64) -> f64 {
        match self {
            Unit::Ms => ms_value,
            Unit::Mph => ms_value / Self::MPH_TO_MS,
            Unit::Kmh => ms_value / Self::KMH_TO_MS,
        }
    }
}

impl std::fmt::Display for Unit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Unit::Ms => write!(f, "m/s"),
            Unit::Mph => write!(f, "mph"),
            Unit::Kmh => write!(f, "km/h"),
        }
    }
}

impl std::str::FromStr for Unit {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "m/s" | "ms" => Ok(Unit::Ms),
            "mph" => Ok(Unit::Mph),
            "km/h" | "kmh" => Ok(Unit::Kmh),
            _ => Err(MalidriveError::InvalidAttribute {
                attribute: "unit".to_string(),
                value: s.to_string(),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_unit_from_str() {
        assert_eq!("m/s".parse::<Unit>().unwrap(), Unit::Ms);
        assert_eq!("mph".parse::<Unit>().unwrap(), Unit::Mph);
        assert_eq!("km/h".parse::<Unit>().unwrap(), Unit::Kmh);
        assert!("invalid".parse::<Unit>().is_err());
    }

    #[test]
    fn test_unit_display() {
        assert_eq!(format!("{}", Unit::Ms), "m/s");
        assert_eq!(format!("{}", Unit::Mph), "mph");
        assert_eq!(format!("{}", Unit::Kmh), "km/h");
    }

    #[test]
    fn test_unit_conversion_to_ms() {
        // 1 m/s = 1 m/s
        assert_relative_eq!(Unit::Ms.to_m_per_s(1.0), 1.0);

        // 100 km/h ≈ 27.78 m/s
        assert_relative_eq!(Unit::Kmh.to_m_per_s(100.0), 27.778, epsilon = 0.001);

        // 60 mph ≈ 26.82 m/s
        assert_relative_eq!(Unit::Mph.to_m_per_s(60.0), 26.822, epsilon = 0.001);
    }

    #[test]
    fn test_unit_conversion_to_kmh() {
        // 27.78 m/s ≈ 100 km/h
        assert_relative_eq!(Unit::Ms.to_kmh(27.778), 100.0, epsilon = 0.01);

        // 60 mph ≈ 96.56 km/h
        assert_relative_eq!(Unit::Mph.to_kmh(60.0), 96.56, epsilon = 0.01);

        // 100 km/h = 100 km/h
        assert_relative_eq!(Unit::Kmh.to_kmh(100.0), 100.0);
    }

    #[test]
    fn test_unit_conversion_to_mph() {
        // 26.82 m/s ≈ 60 mph
        assert_relative_eq!(Unit::Ms.to_mph(26.822), 60.0, epsilon = 0.01);

        // 100 km/h ≈ 62.14 mph
        assert_relative_eq!(Unit::Kmh.to_mph(100.0), 62.137, epsilon = 0.01);

        // 60 mph = 60 mph
        assert_relative_eq!(Unit::Mph.to_mph(60.0), 60.0);
    }

    #[test]
    fn test_unit_from_ms() {
        // 27.78 m/s in various units
        let ms_value = 27.778;
        assert_relative_eq!(Unit::Ms.from_m_per_s(ms_value), 27.778);
        assert_relative_eq!(Unit::Kmh.from_m_per_s(ms_value), 100.0, epsilon = 0.01);
        assert_relative_eq!(Unit::Mph.from_m_per_s(ms_value), 62.137, epsilon = 0.01);
    }
}
