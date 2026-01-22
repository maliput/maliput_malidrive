//! XODR Elevation Profile types.
//!
//! Elevation profile defines the elevation of the road reference line
//! as a function of the s-coordinate.

/// Elevation record describing the elevation as a cubic polynomial.
///
/// The elevation is defined as: elevation(ds) = a + b*ds + c*ds² + d*ds³
/// where ds is the distance from the start of the elevation record.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Elevation {
    /// Start position (s-coordinate) of this elevation record.
    pub s: f64,
    /// Constant coefficient (elevation at s).
    pub a: f64,
    /// Linear coefficient (slope at s).
    pub b: f64,
    /// Quadratic coefficient.
    pub c: f64,
    /// Cubic coefficient.
    pub d: f64,
}

impl Elevation {
    /// Creates a new elevation record.
    pub fn new(s: f64, a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { s, a, b, c, d }
    }

    /// Creates a constant elevation.
    pub fn constant(s: f64, elevation: f64) -> Self {
        Self::new(s, elevation, 0.0, 0.0, 0.0)
    }

    /// Creates a linear elevation (constant slope).
    pub fn linear(s: f64, elevation: f64, slope: f64) -> Self {
        Self::new(s, elevation, slope, 0.0, 0.0)
    }

    /// Evaluates the elevation at the given s-coordinate.
    ///
    /// # Arguments
    /// * `s_eval` - The s-coordinate at which to evaluate the elevation.
    ///
    /// # Returns
    /// The elevation value (z-coordinate).
    pub fn elevation_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        self.a + self.b * ds + self.c * ds * ds + self.d * ds * ds * ds
    }

    /// Evaluates the derivative of the elevation (slope) at the given s-coordinate.
    ///
    /// # Returns
    /// The slope (d(elevation)/ds).
    pub fn slope_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        self.b + 2.0 * self.c * ds + 3.0 * self.d * ds * ds
    }

    /// Evaluates the second derivative of the elevation at the given s-coordinate.
    ///
    /// # Returns
    /// The curvature of the elevation profile.
    pub fn slope_dot_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        2.0 * self.c + 6.0 * self.d * ds
    }

    /// Returns true if this is a flat elevation (constant, b, c, d are all zero).
    pub fn is_flat(&self) -> bool {
        self.b == 0.0 && self.c == 0.0 && self.d == 0.0
    }

    /// Returns true if this is a linear elevation (c, d are zero).
    pub fn is_linear(&self) -> bool {
        self.c == 0.0 && self.d == 0.0
    }

    /// Converts the slope to a grade percentage.
    pub fn grade_percent_at(&self, s_eval: f64) -> f64 {
        self.slope_at(s_eval) * 100.0
    }
}

impl Default for Elevation {
    fn default() -> Self {
        Self::constant(0.0, 0.0)
    }
}

/// Elevation profile containing multiple elevation records.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ElevationProfile {
    /// Elevation records sorted by s-coordinate.
    pub elevations: Vec<Elevation>,
}

impl ElevationProfile {
    /// Creates a new empty elevation profile.
    pub fn new(elevations: Vec<Elevation>) -> Self {
        Self { elevations }
    }

    /// Creates an elevation profile with a single flat elevation at z=0.
    pub fn flat() -> Self {
        Self {
            elevations: vec![Elevation::default()],
        }
    }

    /// Adds an elevation record.
    pub fn add_elevation(&mut self, elevation: Elevation) {
        // Insert maintaining sorted order by s
        let pos = self.elevations.iter().position(|e| e.s > elevation.s).unwrap_or(self.elevations.len());
        self.elevations.insert(pos, elevation);
    }

    /// Evaluates the elevation at the given s-coordinate.
    pub fn elevation_at(&self, s: f64) -> f64 {
        self.find_elevation_at(s)
            .map(|e| e.elevation_at(s))
            .unwrap_or(0.0)
    }

    /// Evaluates the slope at the given s-coordinate.
    pub fn slope_at(&self, s: f64) -> f64 {
        self.find_elevation_at(s)
            .map(|e| e.slope_at(s))
            .unwrap_or(0.0)
    }

    /// Finds the elevation record that applies at the given s-coordinate.
    fn find_elevation_at(&self, s: f64) -> Option<&Elevation> {
        // Find the last elevation record with s <= given s
        self.elevations.iter().rev().find(|e| e.s <= s)
            .or_else(|| self.elevations.first())
    }

    /// Returns true if the profile is empty.
    pub fn is_empty(&self) -> bool {
        self.elevations.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_elevation_creation() {
        let elev = Elevation::new(10.0, 5.0, 0.1, 0.01, 0.001);
        assert_relative_eq!(elev.s, 10.0);
        assert_relative_eq!(elev.a, 5.0);
        assert_relative_eq!(elev.b, 0.1);
        assert_relative_eq!(elev.c, 0.01);
        assert_relative_eq!(elev.d, 0.001);
    }

    #[test]
    fn test_elevation_constant() {
        let elev = Elevation::constant(0.0, 10.0);
        assert_relative_eq!(elev.elevation_at(0.0), 10.0);
        assert_relative_eq!(elev.elevation_at(100.0), 10.0);
        assert_relative_eq!(elev.slope_at(100.0), 0.0);
        assert!(elev.is_flat());
    }

    #[test]
    fn test_elevation_linear() {
        // 5% grade starting at elevation 10
        let elev = Elevation::linear(0.0, 10.0, 0.05);
        assert_relative_eq!(elev.elevation_at(0.0), 10.0);
        assert_relative_eq!(elev.elevation_at(100.0), 15.0); // 10 + 0.05*100
        assert_relative_eq!(elev.slope_at(50.0), 0.05);
        assert_relative_eq!(elev.grade_percent_at(50.0), 5.0);
        assert!(elev.is_linear());
        assert!(!elev.is_flat());
    }

    #[test]
    fn test_elevation_profile() {
        let mut profile = ElevationProfile::new(Vec::new());
        profile.add_elevation(Elevation::constant(0.0, 0.0));
        profile.add_elevation(Elevation::linear(100.0, 0.0, 0.05));
        profile.add_elevation(Elevation::constant(200.0, 5.0));

        assert_relative_eq!(profile.elevation_at(0.0), 0.0);
        assert_relative_eq!(profile.elevation_at(50.0), 0.0);
        assert_relative_eq!(profile.elevation_at(100.0), 0.0);
        assert_relative_eq!(profile.elevation_at(150.0), 2.5); // 0 + 0.05*50
        assert_relative_eq!(profile.elevation_at(200.0), 5.0);
        assert_relative_eq!(profile.elevation_at(250.0), 5.0);
    }

    #[test]
    fn test_elevation_profile_flat() {
        let profile = ElevationProfile::flat();
        assert_relative_eq!(profile.elevation_at(0.0), 0.0);
        assert_relative_eq!(profile.elevation_at(1000.0), 0.0);
    }

    #[test]
    fn test_elevation_default() {
        let elev = Elevation::default();
        assert_relative_eq!(elev.s, 0.0);
        assert_relative_eq!(elev.a, 0.0);
        assert!(elev.is_flat());
    }
}
