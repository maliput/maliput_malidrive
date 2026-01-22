//! XODR Lateral Profile types (superelevation).
//!
//! Lateral profile defines the superelevation (banking) of the road surface
//! as a function of the s-coordinate.

/// Superelevation record describing the cross slope as a cubic polynomial.
///
/// The superelevation is defined as: superelevation(ds) = a + b*ds + c*ds² + d*ds³
/// where ds is the distance from the start of the superelevation record.
/// The value represents the tangent of the banking angle (rise/run).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Superelevation {
    /// Start position (s-coordinate) of this superelevation record.
    pub s: f64,
    /// Constant coefficient (superelevation at s).
    pub a: f64,
    /// Linear coefficient.
    pub b: f64,
    /// Quadratic coefficient.
    pub c: f64,
    /// Cubic coefficient.
    pub d: f64,
}

impl Superelevation {
    /// Creates a new superelevation record.
    pub fn new(s: f64, a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { s, a, b, c, d }
    }

    /// Creates a constant superelevation.
    pub fn constant(s: f64, superelevation: f64) -> Self {
        Self::new(s, superelevation, 0.0, 0.0, 0.0)
    }

    /// Creates a flat superelevation (no banking).
    pub fn flat(s: f64) -> Self {
        Self::constant(s, 0.0)
    }

    /// Evaluates the superelevation at the given s-coordinate.
    ///
    /// # Arguments
    /// * `s_eval` - The s-coordinate at which to evaluate the superelevation.
    ///
    /// # Returns
    /// The superelevation value (tangent of banking angle).
    pub fn superelevation_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        self.a + self.b * ds + self.c * ds * ds + self.d * ds * ds * ds
    }

    /// Evaluates the derivative of the superelevation at the given s-coordinate.
    ///
    /// # Returns
    /// The rate of change of superelevation.
    pub fn superelevation_dot_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        self.b + 2.0 * self.c * ds + 3.0 * self.d * ds * ds
    }

    /// Evaluates the second derivative of the superelevation.
    pub fn superelevation_dot_dot_at(&self, s_eval: f64) -> f64 {
        let ds = s_eval - self.s;
        2.0 * self.c + 6.0 * self.d * ds
    }

    /// Returns the banking angle in radians at the given s-coordinate.
    pub fn angle_at(&self, s_eval: f64) -> f64 {
        self.superelevation_at(s_eval).atan()
    }

    /// Returns true if this is a flat superelevation (no banking).
    pub fn is_flat(&self) -> bool {
        self.a == 0.0 && self.b == 0.0 && self.c == 0.0 && self.d == 0.0
    }

    /// Returns true if this is a constant superelevation (b, c, d are zero).
    pub fn is_constant(&self) -> bool {
        self.b == 0.0 && self.c == 0.0 && self.d == 0.0
    }
}

impl Default for Superelevation {
    fn default() -> Self {
        Self::flat(0.0)
    }
}

/// Lateral profile containing superelevation records.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LateralProfile {
    /// Superelevation records sorted by s-coordinate.
    pub superelevations: Vec<Superelevation>,
}

impl LateralProfile {
    /// Creates a new lateral profile with the given superelevations.
    pub fn new(superelevations: Vec<Superelevation>) -> Self {
        Self { superelevations }
    }

    /// Creates a lateral profile with flat superelevation.
    pub fn flat() -> Self {
        Self {
            superelevations: vec![Superelevation::default()],
        }
    }

    /// Adds a superelevation record.
    pub fn add_superelevation(&mut self, superelevation: Superelevation) {
        // Insert maintaining sorted order by s
        let pos = self.superelevations.iter().position(|e| e.s > superelevation.s).unwrap_or(self.superelevations.len());
        self.superelevations.insert(pos, superelevation);
    }

    /// Evaluates the superelevation at the given s-coordinate.
    pub fn superelevation_at(&self, s: f64) -> f64 {
        self.find_superelevation_at(s)
            .map(|e| e.superelevation_at(s))
            .unwrap_or(0.0)
    }

    /// Evaluates the banking angle at the given s-coordinate.
    pub fn angle_at(&self, s: f64) -> f64 {
        self.find_superelevation_at(s)
            .map(|e| e.angle_at(s))
            .unwrap_or(0.0)
    }

    /// Finds the superelevation record that applies at the given s-coordinate.
    fn find_superelevation_at(&self, s: f64) -> Option<&Superelevation> {
        // Find the last superelevation record with s <= given s
        self.superelevations.iter().rev().find(|e| e.s <= s)
            .or_else(|| self.superelevations.first())
    }

    /// Returns true if the profile is empty.
    pub fn is_empty(&self) -> bool {
        self.superelevations.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_superelevation_creation() {
        let sup = Superelevation::new(10.0, 0.05, 0.001, 0.0001, 0.00001);
        assert_relative_eq!(sup.s, 10.0);
        assert_relative_eq!(sup.a, 0.05);
        assert_relative_eq!(sup.b, 0.001);
    }

    #[test]
    fn test_superelevation_constant() {
        // 5% banking
        let sup = Superelevation::constant(0.0, 0.05);
        assert_relative_eq!(sup.superelevation_at(0.0), 0.05);
        assert_relative_eq!(sup.superelevation_at(100.0), 0.05);
        assert_relative_eq!(sup.superelevation_dot_at(100.0), 0.0);
        assert!(sup.is_constant());
        assert!(!sup.is_flat());
    }

    #[test]
    fn test_superelevation_flat() {
        let sup = Superelevation::flat(0.0);
        assert_relative_eq!(sup.superelevation_at(0.0), 0.0);
        assert_relative_eq!(sup.angle_at(0.0), 0.0);
        assert!(sup.is_flat());
    }

    #[test]
    fn test_superelevation_angle() {
        // superelevation of 0.05 ≈ 2.86 degrees
        let sup = Superelevation::constant(0.0, 0.05);
        let angle = sup.angle_at(0.0);
        assert_relative_eq!(angle, 0.05_f64.atan());
        assert_relative_eq!(angle.to_degrees(), 2.862, epsilon = 0.001);
    }

    #[test]
    fn test_lateral_profile() {
        let mut profile = LateralProfile::new(Vec::new());
        profile.add_superelevation(Superelevation::flat(0.0));
        profile.add_superelevation(Superelevation::constant(100.0, 0.05));
        profile.add_superelevation(Superelevation::flat(200.0));

        assert_relative_eq!(profile.superelevation_at(50.0), 0.0);
        assert_relative_eq!(profile.superelevation_at(150.0), 0.05);
        assert_relative_eq!(profile.superelevation_at(250.0), 0.0);
    }

    #[test]
    fn test_lateral_profile_flat() {
        let profile = LateralProfile::flat();
        assert_relative_eq!(profile.superelevation_at(0.0), 0.0);
        assert_relative_eq!(profile.superelevation_at(1000.0), 0.0);
    }

    #[test]
    fn test_superelevation_default() {
        let sup = Superelevation::default();
        assert_relative_eq!(sup.s, 0.0);
        assert_relative_eq!(sup.a, 0.0);
        assert!(sup.is_flat());
    }
}
