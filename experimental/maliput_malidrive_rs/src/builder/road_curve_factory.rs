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
    ElevationProfile, Geometry, GeometryDescription, LateralProfile, PRange, RoadHeader,
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
        let p0 = geometry.s_0;
        let p1 = geometry.s_0 + geometry.length;
        let xy0 = geometry.start_point;
        let heading = geometry.orientation;

        match &geometry.description {
            GeometryDescription::Line(_) => {
                // Calculate dxy from heading and length
                let dxy = Vector2::new(
                    geometry.length * heading.cos(),
                    geometry.length * heading.sin(),
                );
                Ok(Arc::new(LineGroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    dxy,
                    p0,
                    p1,
                )?))
            }
            GeometryDescription::Arc(arc) => {
                Ok(Arc::new(ArcGroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    heading,
                    arc.curvature,
                    geometry.length,
                    p0,
                    p1,
                )?))
            }
            GeometryDescription::Spiral(spiral) => {
                Ok(Arc::new(SpiralGroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    heading,
                    spiral.curv_start,
                    spiral.curv_end,
                    geometry.length,
                    p0,
                    p1,
                )))
            }
            GeometryDescription::ParamPoly3(poly) => {
                let range = match poly.p_range {
                    PRange::ArcLength => ParamPoly3Range::ArcLength,
                    PRange::Normalized => ParamPoly3Range::Normalized,
                };
                Ok(Arc::new(ParamPoly3GroundCurve::new(
                    self.linear_tolerance,
                    xy0,
                    heading,
                    geometry.length,
                    p0,
                    p1,
                    [poly.a_u, poly.b_u, poly.c_u, poly.d_u],
                    [poly.a_v, poly.b_v, poly.c_v, poly.d_v],
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
        elevation_profile: &ElevationProfile,
        p0: f64,
        p1: f64,
    ) -> Arc<dyn Function> {
        if !elevation_profile.elevations.is_empty() {
            let mut polynomials: Vec<CubicPolynomial> = Vec::new();

            for (i, elev) in elevation_profile.elevations.iter().enumerate() {
                let start = elev.s;
                let end = if i + 1 < elevation_profile.elevations.len() {
                    elevation_profile.elevations[i + 1].s
                } else {
                    p1
                };

                polynomials.push(CubicPolynomial::new(
                    elev.a, elev.b, elev.c, elev.d, start, end,
                ));
            }

            Arc::new(PiecewiseCubicPolynomial::new(polynomials))
        } else {
            // No elevation - return constant zero
            Arc::new(ConstantFunction::zero(p0, p1))
        }
    }

    /// Creates a superelevation function from OpenDRIVE lateral profile.
    pub fn make_superelevation_function(
        &self,
        lateral_profile: &LateralProfile,
        p0: f64,
        p1: f64,
    ) -> Arc<dyn Function> {
        if !lateral_profile.superelevations.is_empty() {
            let mut polynomials: Vec<CubicPolynomial> = Vec::new();

            for (i, superlev) in lateral_profile.superelevations.iter().enumerate() {
                let start = superlev.s;
                let end = if i + 1 < lateral_profile.superelevations.len() {
                    lateral_profile.superelevations[i + 1].s
                } else {
                    p1
                };

                polynomials.push(CubicPolynomial::new(
                    superlev.a, superlev.b, superlev.c, superlev.d, start, end,
                ));
            }

            Arc::new(PiecewiseCubicPolynomial::new(polynomials))
        } else {
            // No superelevation - return constant zero
            Arc::new(ConstantFunction::zero(p0, p1))
        }
    }

    /// Creates a complete road curve from an OpenDRIVE road header.
    pub fn make_road_curve(&self, road: &RoadHeader) -> MalidriveResult<Arc<RoadCurve>> {
        // Build the ground curve from geometry elements
        if road.plan_view.is_empty() {
            return Err(MalidriveError::ParsingError(
                "Road has no reference geometry".to_string(),
            ));
        }

        let ground_curve = self.make_piecewise_ground_curve(&road.plan_view.geometries)?;

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
    use crate::xodr::{load_from_file, Line};
    use approx::assert_relative_eq;
    use std::path::PathBuf;

    const TOLERANCE: f64 = 1e-6;

    fn resources_dir() -> PathBuf {
        let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
        PathBuf::from(manifest_dir).join("../../resources")
    }

    #[test]
    fn test_make_line_ground_curve() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let geometry = Geometry::new_line(
            0.0,                        // s_0
            Vector2::new(0.0, 0.0),     // start_point
            0.0,                        // orientation
            100.0,                      // length
        );

        let curve = factory.make_ground_curve(&geometry).unwrap();
        assert_relative_eq!(curve.arc_length(), 100.0);

        let pos = curve.g(50.0).unwrap();
        assert_relative_eq!(pos.x, 50.0, epsilon = 1e-6);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_make_arc_ground_curve() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let geometry = Geometry::new_arc(
            0.0,                        // s_0
            Vector2::new(0.0, 0.0),     // start_point
            0.0,                        // orientation
            50.0,                       // length
            0.01,                       // curvature
        );

        let curve = factory.make_ground_curve(&geometry).unwrap();
        assert_relative_eq!(curve.arc_length(), 50.0);
    }

    #[test]
    fn test_make_elevation_function_empty() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let profile = ElevationProfile::default();
        let elevation = factory.make_elevation_function(&profile, 0.0, 100.0);
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

        let elevation = factory.make_elevation_function(&profile, 0.0, 100.0);
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

    #[test]
    fn test_make_road_curve_from_xodr() {
        let factory = RoadCurveFactory::new(TOLERANCE, 1.0);

        let xodr_path = resources_dir().join("SingleLane.xodr");
        let doc = load_from_file(xodr_path.to_str().unwrap())
            .expect("Failed to load SingleLane.xodr");

        let road_curve = factory.make_road_curve(&doc.roads[0])
            .expect("Failed to create road curve");

        // SingleLane.xodr has a 100m road
        assert!((road_curve.p1() - road_curve.p0() - 100.0).abs() < 1.0);
    }
}
