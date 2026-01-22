//! Factory for creating road curves from OpenDRIVE geometry data.

use std::sync::Arc;

use nalgebra::Vector2;

use crate::common::{MalidriveError, MalidriveResult};
use crate::road_curve::{
    ArcGroundCurve, ConstantFunction, CubicPolynomial, Function, GroundCurve,
    LineGroundCurve, ParamPoly3GroundCurve, ParamPoly3Range, PiecewiseGroundCurve,
    PiecewiseCubicPolynomial, RoadCurve, SpiralGroundCurve,
};
use crate::xodr::{
    ElevationProfile, Geometry, GeometryType, LateralProfile, RoadHeader,
};

/// Factory for creating road curves from OpenDRIVE data.
pub struct RoadCurveFactory {
    /// Linear tolerance.
    linear_tolerance: f64,
    /// Scale length.
    scale_length: f64,
}

impl RoadCurveFactory {
    /// Creates a new RoadCurveFactory.
    pub fn new(linear_tolerance: f64, scale_length: f64) -> Self {
        Self {
            linear_tolerance,
            scale_length,
        }
    }

    /// Creates a ground curve from an OpenDRIVE geometry element.
    pub fn make_ground_curve(&self, geometry: &Geometry) -> MalidriveResult<Arc<dyn GroundCurve>> {
        let p0 = geometry.s;
        let p1 = geometry.s + geometry.length;
        let xy0 = Vector2::new(geometry.x, geometry.y);
        let heading = geometry.hdg;

        match &geometry.geometry_type {
            GeometryType::Line => {
                Ok(Arc::new(LineGroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    heading,
                    geometry.length,
                    p0,
                    p1,
                )))
            }
            GeometryType::Arc { curvature } => {
                Ok(Arc::new(ArcGroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    heading,
                    *curvature,
                    geometry.length,
                    p0,
                    p1,
                )))
            }
            GeometryType::Spiral { curv_start, curv_end } => {
                Ok(Arc::new(SpiralGroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    heading,
                    *curv_start,
                    *curv_end,
                    geometry.length,
                    p0,
                    p1,
                )))
            }
            GeometryType::Poly3 { a, b, c, d } => {
                // For poly3, we use the same coefficients for u and v=0 (local coords)
                // This is a simplified interpretation - actual poly3 is more complex
                Ok(Arc::new(ParamPoly3GroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    heading,
                    geometry.length,
                    p0,
                    p1,
                    [*a, *b, *c, *d],
                    [0.0, 0.0, 0.0, 0.0],
                    ParamPoly3Range::Normalized,
                )))
            }
            GeometryType::ParamPoly3 {
                a_u, b_u, c_u, d_u,
                a_v, b_v, c_v, d_v,
                p_range,
            } => {
                let range = match p_range.as_deref() {
                    Some("arcLength") => ParamPoly3Range::ArcLength,
                    _ => ParamPoly3Range::Normalized,
                };
                Ok(Arc::new(ParamPoly3GroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    heading,
                    geometry.length,
                    p0,
                    p1,
                    [*a_u, *b_u, *c_u, *d_u],
                    [*a_v, *b_v, *c_v, *d_v],
                    range,
                )))
            }
        }
    }

    /// Creates a piecewise ground curve from multiple geometry elements.
    pub fn make_piecewise_ground_curve(
        &self,
        geometries: &[Geometry],
    ) -> MalidriveResult<Arc<PiecewiseGroundCurve>> {
        let mut segments: Vec<Arc<dyn GroundCurve>> = Vec::new();

        for geom in geometries {
            let curve = self.make_ground_curve(geom)?;
            segments.push(curve);
        }

        let piecewise = PiecewiseGroundCurve::from_segments(self.linear_tolerance, segments)?;
        Ok(Arc::new(piecewise))
    }

    /// Creates an elevation function from OpenDRIVE elevation profile.
    pub fn make_elevation_function(
        &self,
        elevation_profile: &Option<ElevationProfile>,
        p0: f64,
        p1: f64,
    ) -> Arc<dyn Function> {
        match elevation_profile {
            Some(profile) if !profile.elevations.is_empty() => {
                let mut polynomials: Vec<CubicPolynomial> = Vec::new();

                for (i, elev) in profile.elevations.iter().enumerate() {
                    let start = elev.s;
                    let end = if i + 1 < profile.elevations.len() {
                        profile.elevations[i + 1].s
                    } else {
                        p1
                    };

                    polynomials.push(CubicPolynomial::new(
                        elev.a, elev.b, elev.c, elev.d, start, end,
                    ));
                }

                Arc::new(PiecewiseCubicPolynomial::new(polynomials))
            }
            _ => {
                // No elevation - return constant zero
                Arc::new(ConstantFunction::zero(p0, p1))
            }
        }
    }

    /// Creates a superelevation function from OpenDRIVE lateral profile.
    pub fn make_superelevation_function(
        &self,
        lateral_profile: &Option<LateralProfile>,
        p0: f64,
        p1: f64,
    ) -> Arc<dyn Function> {
        match lateral_profile {
            Some(profile) if !profile.superelevations.is_empty() => {
                let mut polynomials: Vec<CubicPolynomial> = Vec::new();

                for (i, superlev) in profile.superelevations.iter().enumerate() {
                    let start = superlev.s;
                    let end = if i + 1 < profile.superelevations.len() {
                        profile.superelevations[i + 1].s
                    } else {
                        p1
                    };

                    polynomials.push(CubicPolynomial::new(
                        superlev.a, superlev.b, superlev.c, superlev.d, start, end,
                    ));
                }

                Arc::new(PiecewiseCubicPolynomial::new(polynomials))
            }
            _ => {
                // No superelevation - return constant zero
                Arc::new(ConstantFunction::zero(p0, p1))
            }
        }
    }

    /// Creates a complete road curve from an OpenDRIVE road header.
    pub fn make_road_curve(&self, road: &RoadHeader) -> MalidriveResult<Arc<RoadCurve>> {
        // Build the ground curve from geometry elements
        let ground_curve = if let Some(ref ref_geom) = road.reference_geometry {
            self.make_piecewise_ground_curve(&ref_geom.geometries)?
        } else {
            return Err(MalidriveError::ParsingError(
                "Road has no reference geometry".to_string(),
            ));
        };

        let p0 = ground_curve.p0();
        let p1 = ground_curve.p1();

        // Build elevation function
        let elevation = self.make_elevation_function(&road.elevation_profile, p0, p1);

        // Build superelevation function
        let superelevation = self.make_superelevation_function(&road.lateral_profile, p0, p1);

        Ok(Arc::new(RoadCurve::new(
            ground_curve,
            elevation,
            superelevation,
            self.linear_tolerance,
            self.scale_length,
        )))
    }

    /// Creates a lane width function from OpenDRIVE lane width polynomials.
    pub fn make_lane_width_function(
        &self,
        widths: &[(f64, f64, f64, f64, f64)], // (s_offset, a, b, c, d)
        s0: f64,
        s1: f64,
    ) -> Arc<dyn Function> {
        if widths.is_empty() {
            // Default width of 3.5m
            return Arc::new(CubicPolynomial::constant(3.5, s0, s1));
        }

        let mut polynomials: Vec<CubicPolynomial> = Vec::new();

        for (i, &(s_offset, a, b, c, d)) in widths.iter().enumerate() {
            let start = s0 + s_offset;
            let end = if i + 1 < widths.len() {
                s0 + widths[i + 1].0
            } else {
                s1
            };

            polynomials.push(CubicPolynomial::new(a, b, c, d, start, end));
        }

        Arc::new(PiecewiseCubicPolynomial::new(polynomials))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const TOLERANCE: f64 = 1e-6;

    #[test]
    fn test_make_line_ground_curve() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let geometry = Geometry {
            s: 0.0,
            x: 0.0,
            y: 0.0,
            hdg: 0.0,
            length: 100.0,
            geometry_type: GeometryType::Line,
        };

        let curve = factory.make_ground_curve(&geometry).unwrap();
        assert_relative_eq!(curve.arc_length(), 100.0);

        let pos = curve.g(50.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-9);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_make_arc_ground_curve() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let geometry = Geometry {
            s: 0.0,
            x: 0.0,
            y: 0.0,
            hdg: 0.0,
            length: 50.0,
            geometry_type: GeometryType::Arc { curvature: 0.01 },
        };

        let curve = factory.make_ground_curve(&geometry).unwrap();
        assert_relative_eq!(curve.arc_length(), 50.0);
    }

    #[test]
    fn test_make_elevation_function_empty() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let elevation = factory.make_elevation_function(&None, 0.0, 100.0);
        assert_relative_eq!(elevation.f(50.0).unwrap(), 0.0);
    }

    #[test]
    fn test_make_elevation_function() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let profile = ElevationProfile {
            elevations: vec![
                crate::xodr::Elevation { s: 0.0, a: 0.0, b: 0.1, c: 0.0, d: 0.0 },
            ],
        };

        let elevation = factory.make_elevation_function(&Some(profile), 0.0, 100.0);
        // At s=50, elevation = 0 + 0.1*50 = 5
        assert_relative_eq!(elevation.f(50.0).unwrap(), 5.0);
    }

    #[test]
    fn test_make_lane_width_function() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let widths = vec![(0.0, 3.5, 0.0, 0.0, 0.0)]; // constant 3.5m width

        let width_fn = factory.make_lane_width_function(&widths, 0.0, 100.0);
        assert_relative_eq!(width_fn.f(50.0).unwrap(), 3.5);
    }
}
