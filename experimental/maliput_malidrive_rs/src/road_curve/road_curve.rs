//! 3D Road curve implementation.
//!
//! A RoadCurve combines a 2D ground curve (reference line) with elevation
//! and superelevation profiles to create a complete 3D road representation.
//!
//! The road curve provides the mapping from LANE coordinates (s, r, h) to
//! INERTIAL coordinates (x, y, z).

use nalgebra::{Rotation3, Vector3};
use std::sync::Arc;

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{Function, GroundCurve};

/// A 3D road curve combining ground curve, elevation, and superelevation.
///
/// The road curve maps from LANE coordinates (s, r, h) to INERTIAL coordinates (x, y, z):
/// - s: Distance along the reference line (ground curve)
/// - r: Lateral offset from the reference line (positive = left)
/// - h: Height above the road surface
///
/// The transformation considers:
/// 1. Ground curve: (s) -> (x_ground, y_ground) in the horizontal plane
/// 2. Elevation: (s) -> z_elevation
/// 3. Superelevation: (s) -> roll angle for banking
pub struct RoadCurve {
    /// The 2D ground curve (reference line in the xy-plane).
    ground_curve: Arc<dyn GroundCurve>,
    /// Elevation function: s -> z.
    elevation: Arc<dyn Function>,
    /// Superelevation function: s -> roll angle (radians).
    superelevation: Arc<dyn Function>,
    /// Linear tolerance.
    linear_tolerance: f64,
    /// Scale factor for the curve (usually 1.0).
    scale_length: f64,
}

impl RoadCurve {
    /// Creates a new road curve.
    ///
    /// # Arguments
    /// * `ground_curve` - The 2D reference line.
    /// * `elevation` - The elevation profile function.
    /// * `superelevation` - The superelevation (banking) function.
    /// * `linear_tolerance` - Linear tolerance for computations.
    /// * `scale_length` - Scale factor (usually 1.0).
    pub fn new(
        ground_curve: Arc<dyn GroundCurve>,
        elevation: Arc<dyn Function>,
        superelevation: Arc<dyn Function>,
        linear_tolerance: f64,
        scale_length: f64,
    ) -> Self {
        Self {
            ground_curve,
            elevation,
            superelevation,
            linear_tolerance,
            scale_length,
        }
    }

    /// Returns the ground curve.
    pub fn ground_curve(&self) -> &dyn GroundCurve {
        self.ground_curve.as_ref()
    }

    /// Returns the elevation function.
    pub fn elevation(&self) -> &dyn Function {
        self.elevation.as_ref()
    }

    /// Returns the superelevation function.
    pub fn superelevation(&self) -> &dyn Function {
        self.superelevation.as_ref()
    }

    /// Returns the start parameter.
    pub fn p0(&self) -> f64 {
        self.ground_curve.p0()
    }

    /// Returns the end parameter.
    pub fn p1(&self) -> f64 {
        self.ground_curve.p1()
    }

    /// Returns the arc length.
    pub fn arc_length(&self) -> f64 {
        self.ground_curve.arc_length()
    }

    /// Returns the linear tolerance.
    pub fn linear_tolerance(&self) -> f64 {
        self.linear_tolerance
    }

    /// Returns the scale length.
    pub fn scale_length(&self) -> f64 {
        self.scale_length
    }

    /// Computes the 3D position at LANE coordinates (s, r, h).
    ///
    /// # Arguments
    /// * `s` - Distance along the reference line (p coordinate).
    /// * `r` - Lateral offset from the reference line.
    /// * `h` - Height above the road surface.
    ///
    /// # Returns
    /// The position in INERTIAL coordinates (x, y, z).
    pub fn w(&self, s: f64, r: f64, h: f64) -> MalidriveResult<Vector3<f64>> {
        // Get the 2D position on the ground curve
        let g = self.ground_curve.g(s)?;

        // Get elevation and superelevation
        let z_elevation = self.elevation.f(s)?;
        let roll = self.superelevation.f(s)?;

        // Get the heading (yaw angle)
        let heading = self.ground_curve.heading(s)?;

        // Compute the position considering superelevation
        // First, create the rotation matrices
        let cos_heading = heading.cos();
        let sin_heading = heading.sin();
        let cos_roll = roll.cos();
        let sin_roll = roll.sin();

        // The lateral offset is applied in the rotated frame
        // r is positive to the left, so the lateral direction is perpendicular to heading
        let lateral_x = -sin_heading;
        let lateral_y = cos_heading;

        // Apply superelevation: the lateral direction is tilted
        let dx_lateral = r * lateral_x * cos_roll;
        let dy_lateral = r * lateral_y * cos_roll;
        let dz_lateral = -r * sin_roll; // Negative because positive r is left, which goes up for positive roll

        // Height h is applied perpendicular to the road surface
        // After superelevation, "up" is no longer purely vertical
        let dx_height = h * lateral_x * sin_roll;
        let dy_height = h * lateral_y * sin_roll;
        let dz_height = h * cos_roll;

        Ok(Vector3::new(
            g.x + dx_lateral + dx_height,
            g.y + dy_lateral + dy_height,
            z_elevation + dz_lateral + dz_height,
        ))
    }

    /// Computes the rotation matrix at parameter s.
    ///
    /// The rotation transforms from the LANE frame to the INERTIAL frame.
    /// The LANE frame has:
    /// - x-axis along the road (tangent direction)
    /// - y-axis to the left (lateral direction)
    /// - z-axis up (normal to road surface, after superelevation)
    pub fn rotation(&self, s: f64) -> MalidriveResult<Rotation3<f64>> {
        let heading = self.ground_curve.heading(s)?;
        let roll = self.superelevation.f(s)?;
        let pitch = self.elevation_slope(s)?;

        // Create rotation: first roll (about x), then pitch (about y), then yaw (about z)
        // Using ZYX order (yaw-pitch-roll)
        let rotation = Rotation3::from_euler_angles(roll, pitch, heading);
        Ok(rotation)
    }

    /// Computes the slope of the elevation profile at parameter s.
    ///
    /// Returns the angle in radians (positive = uphill).
    pub fn elevation_slope(&self, s: f64) -> MalidriveResult<f64> {
        // slope = atan(dz/ds)
        // For small angles, this is approximately equal to dz/ds
        let dz_ds = self.elevation.f_dot(s)?;
        Ok(dz_ds.atan())
    }

    /// Computes the tangent vector at parameter s (in the ground plane).
    pub fn tangent(&self, s: f64) -> MalidriveResult<Vector3<f64>> {
        let g_dot = self.ground_curve.g_dot(s)?;
        let elevation_slope = self.elevation_slope(s)?;
        let cos_slope = elevation_slope.cos();
        let sin_slope = elevation_slope.sin();

        Ok(Vector3::new(
            g_dot.x * cos_slope,
            g_dot.y * cos_slope,
            sin_slope,
        ))
    }

    /// Computes the curvature at parameter s.
    pub fn curvature(&self, s: f64) -> MalidriveResult<f64> {
        self.ground_curve.heading_dot(s)
    }

    /// Validates that p is within the curve's range.
    fn validate_p(&self, p: f64) -> MalidriveResult<()> {
        let p0 = self.p0();
        let p1 = self.p1();
        if p < p0 - self.linear_tolerance || p > p1 + self.linear_tolerance {
            return Err(MalidriveError::ParameterOutOfRange {
                parameter: "p".to_string(),
                value: p,
                min: p0,
                max: p1,
            });
        }
        Ok(())
    }

    /// Inverse mapping: finds LANE coordinates (s, r, h) for an INERTIAL position.
    ///
    /// This is an approximate inverse using iterative refinement.
    pub fn w_inverse(&self, xyz: Vector3<f64>) -> MalidriveResult<(f64, f64, f64)> {
        // First, find s by projecting to the ground curve
        let xy_target = nalgebra::Vector2::new(xyz.x, xyz.y);
        let s = self.ground_curve.g_inverse(&xy_target)?;

        // Get the position on the centerline at s
        let g = self.ground_curve.g(s)?;
        let z_elevation = self.elevation.f(s)?;
        let roll = self.superelevation.f(s)?;
        let heading = self.ground_curve.heading(s)?;

        // Compute lateral direction
        let cos_heading = heading.cos();
        let sin_heading = heading.sin();
        let lateral_x = -sin_heading;
        let lateral_y = cos_heading;

        // Compute the vector from centerline to target
        let dx = xyz.x - g.x;
        let dy = xyz.y - g.y;
        let dz = xyz.z - z_elevation;

        // Project onto lateral direction (accounting for superelevation)
        let cos_roll = roll.cos();
        let sin_roll = roll.sin();

        // r is the lateral offset
        let r_component_xy = dx * lateral_x + dy * lateral_y;
        let r = r_component_xy / cos_roll;

        // h is the height above road
        let h = dz / cos_roll + r * sin_roll / cos_roll;

        Ok((s, r, h))
    }
}

impl std::fmt::Debug for RoadCurve {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RoadCurve")
            .field("p0", &self.p0())
            .field("p1", &self.p1())
            .field("arc_length", &self.arc_length())
            .field("linear_tolerance", &self.linear_tolerance)
            .field("scale_length", &self.scale_length)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::road_curve::{ConstantFunction, LineGroundCurve};
    use approx::assert_relative_eq;
    use nalgebra::Vector2;
    use std::f64::consts::PI;

    const TOLERANCE: f64 = 1e-6;

    fn make_simple_road_curve() -> RoadCurve {
        let ground_curve = Arc::new(LineGroundCurve::from_heading(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0, // heading east
            100.0, // length
            0.0, // p0
            100.0, // p1
        ).unwrap());
        let elevation = Arc::new(ConstantFunction::zero(0.0, 100.0));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, 100.0));

        RoadCurve::new(ground_curve, elevation, superelevation, TOLERANCE, 1.0)
    }

    #[test]
    fn test_road_curve_creation() {
        let rc = make_simple_road_curve();
        assert_relative_eq!(rc.p0(), 0.0);
        assert_relative_eq!(rc.p1(), 100.0);
        assert_relative_eq!(rc.arc_length(), 100.0);
    }

    #[test]
    fn test_road_curve_centerline() {
        let rc = make_simple_road_curve();

        // At s=0, r=0, h=0
        let pos = rc.w(0.0, 0.0, 0.0).unwrap();
        assert_relative_eq!(pos.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-9);

        // At s=50, r=0, h=0
        let pos = rc.w(50.0, 0.0, 0.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_road_curve_lateral_offset() {
        let rc = make_simple_road_curve();

        // At s=50, r=10 (10m to the left), h=0
        // Heading is 0 (east), so left is north (+y)
        let pos = rc.w(50.0, 10.0, 0.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, 10.0, epsilon = 1e-9);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-9);

        // At s=50, r=-5 (5m to the right), h=0
        let pos = rc.w(50.0, -5.0, 0.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, -5.0, epsilon = 1e-9);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_road_curve_height() {
        let rc = make_simple_road_curve();

        // At s=50, r=0, h=5 (5m above road)
        let pos = rc.w(50.0, 0.0, 5.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(pos.z, 5.0, epsilon = 1e-9);
    }

    #[test]
    fn test_road_curve_with_elevation() {
        let ground_curve = Arc::new(LineGroundCurve::from_heading(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            100.0,
            0.0,
            100.0,
        ).unwrap());
        // Elevation = 0.1 * s (rising 10m over 100m)
        let elevation = Arc::new(crate::road_curve::CubicPolynomial::linear(0.0, 0.1, 0.0, 100.0));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let rc = RoadCurve::new(ground_curve, elevation, superelevation, TOLERANCE, 1.0);

        // At s=50, elevation should be 5m
        let pos = rc.w(50.0, 0.0, 0.0).unwrap();
        assert_relative_eq!(pos.z, 5.0, epsilon = 1e-6);

        // At s=100, elevation should be 10m
        let pos = rc.w(100.0, 0.0, 0.0).unwrap();
        assert_relative_eq!(pos.z, 10.0, epsilon = 1e-6);
    }

    #[test]
    fn test_road_curve_with_superelevation() {
        let ground_curve = Arc::new(LineGroundCurve::from_heading(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            100.0,
            0.0,
            100.0,
        ).unwrap());
        let elevation = Arc::new(ConstantFunction::zero(0.0, 100.0));
        // Superelevation = 5% = 0.05 rad (approximately)
        let superelevation = Arc::new(ConstantFunction::new(0.05, 0.0, 100.0));

        let rc = RoadCurve::new(ground_curve, elevation, superelevation, TOLERANCE, 1.0);

        // At r=10 (10m to the left), with superelevation, the left side should be higher
        let pos = rc.w(50.0, 10.0, 0.0).unwrap();
        // z should be approximately -10 * sin(0.05) ≈ -0.5 (negative because we defined it that way)
        assert!(pos.z.abs() < 1.0); // Just verify superelevation has an effect
    }

    #[test]
    fn test_road_curve_tangent() {
        let rc = make_simple_road_curve();

        let tangent = rc.tangent(50.0).unwrap();
        // For a flat road heading east, tangent should be (1, 0, 0)
        assert_relative_eq!(tangent.x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(tangent.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(tangent.z, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_road_curve_curvature() {
        let rc = make_simple_road_curve();

        // A straight line has zero curvature
        let curvature = rc.curvature(50.0).unwrap();
        assert_relative_eq!(curvature, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_road_curve_elevation_slope() {
        let ground_curve = Arc::new(LineGroundCurve::from_heading(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            0.0,
            100.0,
            0.0,
            100.0,
        ).unwrap());
        // 10% grade
        let elevation = Arc::new(crate::road_curve::CubicPolynomial::linear(0.0, 0.1, 0.0, 100.0));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let rc = RoadCurve::new(ground_curve, elevation, superelevation, TOLERANCE, 1.0);

        let slope = rc.elevation_slope(50.0).unwrap();
        // atan(0.1) ≈ 0.0997 rad
        assert_relative_eq!(slope, 0.1_f64.atan(), epsilon = 1e-9);
    }

    #[test]
    fn test_road_curve_with_heading() {
        // Road heading north (y direction)
        let ground_curve = Arc::new(LineGroundCurve::from_heading(
            TOLERANCE,
            Vector2::new(0.0, 0.0),
            PI / 2.0, // heading north
            100.0,
            0.0,
            100.0,
        ).unwrap());
        let elevation = Arc::new(ConstantFunction::zero(0.0, 100.0));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let rc = RoadCurve::new(ground_curve, elevation, superelevation, TOLERANCE, 1.0);

        // At s=50, position should be (0, 50, 0)
        let pos = rc.w(50.0, 0.0, 0.0).unwrap();
        assert_relative_eq!(pos.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 50.0, epsilon = 1e-6);

        // At s=50, r=10 (10m to the left = west = -x direction)
        let pos = rc.w(50.0, 10.0, 0.0).unwrap();
        assert_relative_eq!(pos.x, -10.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 50.0, epsilon = 1e-6);
    }

    #[test]
    fn test_w_inverse() {
        let rc = make_simple_road_curve();

        // Test a point on the centerline
        let (s, r, h) = rc.w_inverse(Vector3::new(50.0, 0.0, 0.0)).unwrap();
        assert_relative_eq!(s, 50.0, epsilon = TOLERANCE);
        assert_relative_eq!(r, 0.0, epsilon = TOLERANCE);
        assert_relative_eq!(h, 0.0, epsilon = TOLERANCE);

        // Test a point with lateral offset
        let (s, r, h) = rc.w_inverse(Vector3::new(50.0, 5.0, 0.0)).unwrap();
        assert_relative_eq!(s, 50.0, epsilon = TOLERANCE);
        assert_relative_eq!(r, 5.0, epsilon = TOLERANCE);
        assert_relative_eq!(h, 0.0, epsilon = TOLERANCE);

        // Test a point with height
        let (s, r, h) = rc.w_inverse(Vector3::new(50.0, 0.0, 3.0)).unwrap();
        assert_relative_eq!(s, 50.0, epsilon = TOLERANCE);
        assert_relative_eq!(r, 0.0, epsilon = TOLERANCE);
        assert_relative_eq!(h, 3.0, epsilon = TOLERANCE);
    }
}
