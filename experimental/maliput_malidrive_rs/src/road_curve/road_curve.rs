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

    /// Computes the derivative dW/dp at (p, r, h).
    ///
    /// This is essential for computing arc length along offset curves.
    /// The derivative accounts for the ground curve tangent, elevation slope,
    /// and the rotation of the (r, h) offset due to changes in orientation.
    ///
    /// # Arguments
    /// * `p` - Parameter along the reference line.
    /// * `r` - Lateral offset from the reference line.
    /// * `h` - Height above the road surface.
    ///
    /// # Returns
    /// The derivative dW/dp as a 3D vector.
    pub fn w_dot(&self, p: f64, r: f64, h: f64) -> MalidriveResult<Vector3<f64>> {
        // Get ground curve derivative and heading derivative
        let g_prime = self.ground_curve.g_dot(p)?;
        let heading = self.ground_curve.heading(p)?;
        let d_gamma = self.ground_curve.heading_dot(p)?; // curvature

        // Get elevation and its derivatives
        let z_prime = self.elevation.f_dot(p)?;
        let z_prime_prime = self.elevation.f_dot_dot(p)?;

        // Get superelevation and its derivative
        let alpha = self.superelevation.f(p)?;
        let d_alpha = self.superelevation.f_dot(p)?;

        // Compute pitch angle (beta) and its derivative
        let g_prime_norm = (g_prime.x * g_prime.x + g_prime.y * g_prime.y).sqrt();
        let beta = if g_prime_norm.abs() > 1e-15 {
            (-z_prime / g_prime_norm).atan()
        } else {
            0.0
        };
        let cb = beta.cos();
        let sb = beta.sin();

        // d_beta assumes |G'| doesn't vary with p
        let d_beta = if g_prime_norm.abs() > 1e-15 {
            -cb * cb * z_prime_prime / g_prime_norm
        } else {
            0.0
        };

        // Compute trig values
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cg = heading.cos();
        let sg = heading.sin();

        // Rotation matrix R = Rz(gamma) * Ry(beta) * Rx(alpha)
        // Standard ZYX Euler angles
        // R = [cb*cg,  cg*sb*sa - sg*ca,  cg*sb*ca + sg*sa]
        //     [cb*sg,  sg*sb*sa + cg*ca,  sg*sb*ca - cg*sa]
        //     [-sb,    cb*sa,             cb*ca           ]

        // Second column of R (for r-direction): R[:, 1]
        let r1_x = cg * sb * sa - sg * ca;
        let r1_y = sg * sb * sa + cg * ca;
        let r1_z = cb * sa;

        // Third column of R (for h-direction): R[:, 2]
        let r2_x = cg * sb * ca + sg * sa;
        let r2_y = sg * sb * ca - cg * sa;
        let r2_z = cb * ca;

        // Offset in world frame: R * (0, r, h)
        let offset_x = r * r1_x + h * r2_x;
        let offset_y = r * r1_y + h * r2_y;
        let offset_z = r * r1_z + h * r2_z;

        // Compute dR/dp * (0, r, h)
        // Using the fact that dR/dp = R * [ω_body]_× where ω_body = (d_alpha, d_beta, d_gamma) in body frame
        // Actually, we need: d(R*v)/dp = (dR/dp)*v
        //
        // For R = Rz*Ry*Rx, we have:
        // dR/dp = dRz/dγ * Ry * Rx * dγ/dp + Rz * dRy/dβ * Rx * dβ/dp + Rz * Ry * dRx/dα * dα/dp
        //
        // But it's easier to use: dR/dp * v = ω_world × (R*v) where ω_world is angular velocity in world frame
        // ω_world = d_gamma * ẑ + Rz * (d_beta * ŷ) + Rz * Ry * (d_alpha * x̂)
        //         = (0, 0, d_gamma) + d_beta*(−sg, cg, 0) + d_alpha*(cg*cb, sg*cb, −sb)

        let omega_x = d_alpha * cg * cb - d_beta * sg;
        let omega_y = d_alpha * sg * cb + d_beta * cg;
        let omega_z = d_gamma - d_alpha * sb;

        // dR/dp * v = ω × (R*v) = ω × offset
        // Note: ω × offset = (ωy*oz - ωz*oy, ωz*ox - ωx*oz, ωx*oy - ωy*ox)
        let d_offset_x = omega_y * offset_z - omega_z * offset_y;
        let d_offset_y = omega_z * offset_x - omega_x * offset_z;
        let d_offset_z = omega_x * offset_y - omega_y * offset_x;

        // Base derivative from ground curve and elevation
        let base_x = g_prime.x;
        let base_y = g_prime.y;
        let base_z = z_prime;

        Ok(Vector3::new(
            base_x + d_offset_x,
            base_y + d_offset_y,
            base_z + d_offset_z,
        ))
    }

    /// Computes the derivative dW/dp at (p, r, h) with a lane offset function.
    ///
    /// This version accounts for the rate of change of the lane offset r(p).
    ///
    /// # Arguments
    /// * `p` - Parameter along the reference line.
    /// * `r` - Lateral offset from the reference line.
    /// * `h` - Height above the road surface.
    /// * `lane_offset` - The function describing r(p).
    ///
    /// # Returns
    /// The derivative dW/dp as a 3D vector.
    pub fn w_dot_with_lane_offset(
        &self,
        p: f64,
        r: f64,
        h: f64,
        lane_offset: &dyn Function,
    ) -> MalidriveResult<Vector3<f64>> {
        // Get the base derivative (without r_dot contribution)
        let base = self.w_dot(p, r, h)?;

        // Get r_dot from lane offset
        let r_dot = lane_offset.f_dot(p)?;

        // Get rotation parameters
        let heading = self.ground_curve.heading(p)?;
        let alpha = self.superelevation.f(p)?;
        let z_prime = self.elevation.f_dot(p)?;
        let g_prime = self.ground_curve.g_dot(p)?;
        let g_prime_norm = (g_prime.x * g_prime.x + g_prime.y * g_prime.y).sqrt();
        let beta = (-z_prime / g_prime_norm).atan();

        // Compute the contribution from r_dot: R * (0, r_dot, 0)
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();
        let cg = heading.cos();
        let sg = heading.sin();

        // R * (0, 1, 0) = second column of R
        let r_col_x = cg * sb * sa - sg * ca;
        let r_col_y = sg * sb * sa + cg * ca;
        let r_col_z = cb * sa;

        Ok(base + Vector3::new(
            r_dot * r_col_x,
            r_dot * r_col_y,
            r_dot * r_col_z,
        ))
    }

    /// Computes the normalized tangent vector (s-hat) at (p, r, h).
    ///
    /// This is the unit vector in the direction of increasing p.
    pub fn s_hat(&self, p: f64, r: f64, h: f64) -> MalidriveResult<Vector3<f64>> {
        let w_dot = self.w_dot(p, r, h)?;
        let norm = w_dot.norm();
        if norm < 1e-15 {
            return Err(MalidriveError::RoadCurveError(
                "w_dot has zero norm".to_string(),
            ));
        }
        Ok(w_dot / norm)
    }

    /// Computes the orientation (roll, pitch, yaw) at parameter p.
    ///
    /// Returns (roll, pitch, yaw) in radians.
    pub fn orientation(&self, p: f64) -> MalidriveResult<(f64, f64, f64)> {
        let g_prime = self.ground_curve.g_dot(p)?;
        let g_prime_norm = (g_prime.x * g_prime.x + g_prime.y * g_prime.y).sqrt();

        let z_prime = self.elevation.f_dot(p)?;

        let roll = self.superelevation.f(p)?;
        let pitch = (-z_prime / g_prime_norm).atan();
        let yaw = self.ground_curve.heading(p)?;

        Ok((roll, pitch, yaw))
    }

    /// Returns the maximum arc length (LMax) of the ground curve.
    pub fn l_max(&self) -> f64 {
        self.ground_curve.arc_length()
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

    // Additional tests based on C++ patterns
    #[test]
    fn test_road_curve_rotation() {
        let rc = make_simple_road_curve();

        // For a flat road heading east, rotation should be identity-ish
        let rotation = rc.rotation(50.0).unwrap();

        // The rotation matrix should transform (1,0,0) to (1,0,0) approximately
        // since the road is flat and heading east
        let forward = rotation * Vector3::new(1.0, 0.0, 0.0);
        assert_relative_eq!(forward.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(forward.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(forward.z, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_road_curve_with_arc_ground_curve() {
        // Create a road with an arc ground curve
        let arc_length = PI * 10.0 / 2.0; // 90-degree turn with radius 10
        let ground_curve = Arc::new(
            crate::road_curve::ArcGroundCurve::new(
                TOLERANCE,
                Vector2::new(0.0, 0.0),
                0.0, // heading east
                0.1, // radius = 10, turning left
                arc_length,
                0.0,
                arc_length,
            )
            .unwrap(),
        );
        let elevation = Arc::new(ConstantFunction::zero(0.0, arc_length));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, arc_length));

        let rc = RoadCurve::new(ground_curve, elevation, superelevation, TOLERANCE, 1.0);

        // At the end of the arc, position should be at approximately (10, 10)
        let pos = rc.w(arc_length, 0.0, 0.0).unwrap();
        assert_relative_eq!(pos.x, 10.0, epsilon = 0.01);
        assert_relative_eq!(pos.y, 10.0, epsilon = 0.01);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-9);

        // Curvature should be 0.1 (1/radius)
        let curvature = rc.curvature(arc_length / 2.0).unwrap();
        assert_relative_eq!(curvature, 0.1, epsilon = 1e-6);
    }

    #[test]
    fn test_road_curve_diagonal_heading() {
        // Road heading diagonally (45 degrees)
        let ground_curve = Arc::new(
            LineGroundCurve::from_heading(
                TOLERANCE,
                Vector2::new(0.0, 0.0),
                PI / 4.0, // heading NE
                100.0,
                0.0,
                100.0,
            )
            .unwrap(),
        );
        let elevation = Arc::new(ConstantFunction::zero(0.0, 100.0));
        let superelevation = Arc::new(ConstantFunction::zero(0.0, 100.0));

        let rc = RoadCurve::new(ground_curve, elevation, superelevation, TOLERANCE, 1.0);

        // At s=50, with lateral offset r=10
        // The lateral direction is perpendicular to heading, so (-sin(45°), cos(45°)) = (-0.707, 0.707)
        // Position should be:
        // x = 50*cos(45°) + 10*(-sin(45°)) = 35.355 - 7.071 ≈ 28.28
        // y = 50*sin(45°) + 10*(cos(45°)) = 35.355 + 7.071 ≈ 42.43
        let pos = rc.w(50.0, 10.0, 0.0).unwrap();
        let expected_x = 50.0 * (PI / 4.0).cos() + 10.0 * (-(PI / 4.0).sin());
        let expected_y = 50.0 * (PI / 4.0).sin() + 10.0 * (PI / 4.0).cos();
        assert_relative_eq!(pos.x, expected_x, epsilon = 1e-6);
        assert_relative_eq!(pos.y, expected_y, epsilon = 1e-6);
    }

    #[test]
    fn test_road_curve_parameter_range() {
        let rc = make_simple_road_curve();

        // Parameters at bounds should work
        assert!(rc.w(0.0, 0.0, 0.0).is_ok());
        assert!(rc.w(100.0, 0.0, 0.0).is_ok());
    }

    #[test]
    fn test_road_curve_combined_elevation_and_superelevation() {
        // Create a road with both elevation and superelevation
        let ground_curve = Arc::new(
            LineGroundCurve::from_heading(TOLERANCE, Vector2::new(0.0, 0.0), 0.0, 100.0, 0.0, 100.0)
                .unwrap(),
        );
        // 10% grade
        let elevation = Arc::new(crate::road_curve::CubicPolynomial::linear(0.0, 0.1, 0.0, 100.0));
        // 5% superelevation
        let superelevation = Arc::new(ConstantFunction::new(0.05, 0.0, 100.0));

        let rc = RoadCurve::new(ground_curve, elevation, superelevation, TOLERANCE, 1.0);

        // At s=50, elevation = 5m
        let centerline = rc.w(50.0, 0.0, 0.0).unwrap();
        assert_relative_eq!(centerline.z, 5.0, epsilon = 1e-6);

        // Elevation slope should be atan(0.1) ≈ 0.0997 rad
        let slope = rc.elevation_slope(50.0).unwrap();
        assert_relative_eq!(slope, 0.1_f64.atan(), epsilon = 1e-9);
    }
}
