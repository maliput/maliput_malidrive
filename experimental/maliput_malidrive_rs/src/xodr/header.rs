//! XODR Header types.
//!
//! The header contains global information about the OpenDRIVE file.

/// OpenDRIVE file header.
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    /// OpenDRIVE format revision (major).
    pub rev_major: u32,
    /// OpenDRIVE format revision (minor).
    pub rev_minor: u32,
    /// Name of the map/database.
    pub name: Option<String>,
    /// Version of the database.
    pub version: Option<String>,
    /// Creation date.
    pub date: Option<String>,
    /// North coordinate (for georeferencing).
    pub north: Option<f64>,
    /// South coordinate (for georeferencing).
    pub south: Option<f64>,
    /// East coordinate (for georeferencing).
    pub east: Option<f64>,
    /// West coordinate (for georeferencing).
    pub west: Option<f64>,
    /// Vendor information.
    pub vendor: Option<String>,
    /// Georeference string (proj4 or WKT format).
    pub geo_reference: Option<String>,
    /// Inertial offset from the georeference origin.
    pub offset: Option<GeoOffset>,
}

impl Header {
    /// Creates a new header with the given revision.
    pub fn new(rev_major: u32, rev_minor: u32) -> Self {
        Self {
            rev_major,
            rev_minor,
            name: None,
            version: None,
            date: None,
            north: None,
            south: None,
            east: None,
            west: None,
            vendor: None,
            geo_reference: None,
            offset: None,
        }
    }

    /// Creates a header for OpenDRIVE 1.4.
    pub fn v1_4() -> Self {
        Self::new(1, 4)
    }

    /// Creates a header for OpenDRIVE 1.5.
    pub fn v1_5() -> Self {
        Self::new(1, 5)
    }

    /// Creates a header for OpenDRIVE 1.6.
    pub fn v1_6() -> Self {
        Self::new(1, 6)
    }

    /// Creates a header for OpenDRIVE 1.8.
    pub fn v1_8() -> Self {
        Self::new(1, 8)
    }

    /// Returns the OpenDRIVE version as a string.
    pub fn version_string(&self) -> String {
        format!("{}.{}", self.rev_major, self.rev_minor)
    }

    /// Returns true if the version is at least the given major.minor.
    pub fn is_at_least(&self, major: u32, minor: u32) -> bool {
        self.rev_major > major || (self.rev_major == major && self.rev_minor >= minor)
    }

    /// Returns true if a georeference is defined.
    pub fn has_geo_reference(&self) -> bool {
        self.geo_reference.is_some()
    }

    /// Returns true if bounding coordinates are defined.
    pub fn has_bounds(&self) -> bool {
        self.north.is_some() && self.south.is_some() && self.east.is_some() && self.west.is_some()
    }
}

impl Default for Header {
    fn default() -> Self {
        Self::v1_4()
    }
}

/// Geographic offset from the reference point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoOffset {
    /// X offset.
    pub x: f64,
    /// Y offset.
    pub y: f64,
    /// Z offset.
    pub z: f64,
    /// Heading rotation (in radians).
    pub hdg: f64,
}

impl GeoOffset {
    /// Creates a new geographic offset.
    pub fn new(x: f64, y: f64, z: f64, hdg: f64) -> Self {
        Self { x, y, z, hdg }
    }

    /// Creates a translation-only offset.
    pub fn translation(x: f64, y: f64, z: f64) -> Self {
        Self::new(x, y, z, 0.0)
    }

    /// Creates a zero offset.
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0)
    }
}

impl Default for GeoOffset {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_header_creation() {
        let header = Header::new(1, 4);
        assert_eq!(header.rev_major, 1);
        assert_eq!(header.rev_minor, 4);
        assert!(header.name.is_none());
    }

    #[test]
    fn test_header_versions() {
        assert_eq!(Header::v1_4().version_string(), "1.4");
        assert_eq!(Header::v1_5().version_string(), "1.5");
        assert_eq!(Header::v1_6().version_string(), "1.6");
        assert_eq!(Header::v1_8().version_string(), "1.8");
    }

    #[test]
    fn test_header_version_comparison() {
        let header = Header::v1_5();
        assert!(header.is_at_least(1, 4));
        assert!(header.is_at_least(1, 5));
        assert!(!header.is_at_least(1, 6));
        assert!(!header.is_at_least(2, 0));
    }

    #[test]
    fn test_header_geo_reference() {
        let mut header = Header::v1_4();
        assert!(!header.has_geo_reference());

        header.geo_reference = Some("+proj=utm +zone=32".to_string());
        assert!(header.has_geo_reference());
    }

    #[test]
    fn test_header_bounds() {
        let mut header = Header::v1_4();
        assert!(!header.has_bounds());

        header.north = Some(52.0);
        header.south = Some(51.0);
        header.east = Some(14.0);
        header.west = Some(13.0);
        assert!(header.has_bounds());
    }

    #[test]
    fn test_geo_offset() {
        let offset = GeoOffset::new(100.0, 200.0, 10.0, 0.5);
        assert_relative_eq!(offset.x, 100.0);
        assert_relative_eq!(offset.y, 200.0);
        assert_relative_eq!(offset.z, 10.0);
        assert_relative_eq!(offset.hdg, 0.5);
    }

    #[test]
    fn test_geo_offset_translation() {
        let offset = GeoOffset::translation(100.0, 200.0, 10.0);
        assert_relative_eq!(offset.hdg, 0.0);
    }

    #[test]
    fn test_geo_offset_zero() {
        let offset = GeoOffset::zero();
        assert_relative_eq!(offset.x, 0.0);
        assert_relative_eq!(offset.y, 0.0);
        assert_relative_eq!(offset.z, 0.0);
        assert_relative_eq!(offset.hdg, 0.0);
    }
}
