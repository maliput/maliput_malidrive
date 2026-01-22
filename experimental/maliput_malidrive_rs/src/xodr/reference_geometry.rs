//! XODR Reference Geometry types.
//!
//! Contains the overall reference geometry for a road including plan view,
//! elevation profile, and lateral profile.

use crate::xodr::{ElevationProfile, Geometry, LateralProfile, PlanView};

/// Reference geometry for a road combining plan view and profiles.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ReferenceGeometry {
    /// Plan view (2D reference line in XY plane).
    pub plan_view: PlanView,
    /// Elevation profile (Z as function of s).
    pub elevation_profile: ElevationProfile,
    /// Lateral profile (superelevation as function of s).
    pub lateral_profile: LateralProfile,
}

impl ReferenceGeometry {
    /// Creates a new empty reference geometry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a reference geometry with just a plan view.
    pub fn with_plan_view(plan_view: PlanView) -> Self {
        Self {
            plan_view,
            elevation_profile: ElevationProfile::flat(),
            lateral_profile: LateralProfile::flat(),
        }
    }

    /// Creates a reference geometry from a single geometry element.
    pub fn from_geometry(geometry: Geometry) -> Self {
        let mut plan_view = PlanView::new();
        plan_view.add_geometry(geometry);
        Self::with_plan_view(plan_view)
    }

    /// Returns the total length of the reference geometry.
    pub fn length(&self) -> f64 {
        self.plan_view.total_length()
    }

    /// Returns the elevation at the given s-coordinate.
    pub fn elevation_at(&self, s: f64) -> f64 {
        self.elevation_profile.elevation_at(s)
    }

    /// Returns the superelevation at the given s-coordinate.
    pub fn superelevation_at(&self, s: f64) -> f64 {
        self.lateral_profile.superelevation_at(s)
    }

    /// Returns the geometry at the given s-coordinate.
    pub fn geometry_at(&self, s: f64) -> Option<&Geometry> {
        self.plan_view.geometry_at(s)
    }

    /// Returns true if the reference geometry is valid.
    pub fn is_valid(&self) -> bool {
        !self.plan_view.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xodr::{Elevation, Superelevation};
    use approx::assert_relative_eq;
    use nalgebra::Vector2;

    #[test]
    fn test_reference_geometry_creation() {
        let rg = ReferenceGeometry::new();
        assert!(!rg.is_valid());
        assert_relative_eq!(rg.length(), 0.0);
    }

    #[test]
    fn test_reference_geometry_from_geometry() {
        let geom = Geometry::new_line(0.0, Vector2::new(0.0, 0.0), 0.0, 100.0);
        let rg = ReferenceGeometry::from_geometry(geom);

        assert!(rg.is_valid());
        assert_relative_eq!(rg.length(), 100.0);
    }

    #[test]
    fn test_reference_geometry_profiles() {
        let geom = Geometry::new_line(0.0, Vector2::new(0.0, 0.0), 0.0, 100.0);
        let mut rg = ReferenceGeometry::from_geometry(geom);

        // Add elevation
        rg.elevation_profile
            .add_elevation(Elevation::linear(0.0, 0.0, 0.05));

        // Add superelevation
        rg.lateral_profile
            .add_superelevation(Superelevation::constant(0.0, 0.02));

        assert_relative_eq!(rg.elevation_at(100.0), 5.0);
        assert_relative_eq!(rg.superelevation_at(50.0), 0.02);
    }
}
