//! XODR Geometry types and parsing.
//!
//! This module defines the geometry types used in OpenDRIVE road descriptions.
//! The PlanView of a road is composed of one or more geometry elements that
//! describe the reference line of the road in the XY plane.

use crate::common::{MalidriveError, MalidriveResult};
use nalgebra::Vector2;

/// Types of geometric elements supported in OpenDRIVE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryType {
    /// Straight line segment.
    Line,
    /// Circular arc with constant curvature.
    Arc,
    /// Clothoid/Euler spiral with linearly varying curvature.
    Spiral,
    /// Parametric cubic polynomial curve.
    ParamPoly3,
}

impl std::fmt::Display for GeometryType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GeometryType::Line => write!(f, "line"),
            GeometryType::Arc => write!(f, "arc"),
            GeometryType::Spiral => write!(f, "spiral"),
            GeometryType::ParamPoly3 => write!(f, "paramPoly3"),
        }
    }
}

impl std::str::FromStr for GeometryType {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "line" => Ok(GeometryType::Line),
            "arc" => Ok(GeometryType::Arc),
            "spiral" => Ok(GeometryType::Spiral),
            "parampoly3" => Ok(GeometryType::ParamPoly3),
            _ => Err(MalidriveError::InvalidGeometryType(s.to_string())),
        }
    }
}

/// Line geometry description (no additional parameters needed).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Line;

/// Arc geometry description.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arc {
    /// Arc's curvature (1/radius). Positive = counterclockwise.
    pub curvature: f64,
}

impl Arc {
    /// Creates a new Arc with the given curvature.
    pub fn new(curvature: f64) -> Self {
        Self { curvature }
    }
}

/// Spiral (clothoid/Euler spiral) geometry description.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spiral {
    /// Spiral's curvature at the start.
    pub curv_start: f64,
    /// Spiral's curvature at the end.
    pub curv_end: f64,
}

impl Spiral {
    /// Creates a new Spiral with the given start and end curvatures.
    pub fn new(curv_start: f64, curv_end: f64) -> Self {
        Self { curv_start, curv_end }
    }
}

/// Range of parameter p for ParamPoly3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PRange {
    /// p in [0, length of geometry]
    #[default]
    ArcLength,
    /// p in [0, 1]
    Normalized,
}

impl std::str::FromStr for PRange {
    type Err = MalidriveError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "arclength" => Ok(PRange::ArcLength),
            "normalized" => Ok(PRange::Normalized),
            _ => Err(MalidriveError::InvalidAttribute {
                attribute: "pRange".to_string(),
                value: s.to_string(),
            }),
        }
    }
}

/// Parametric cubic polynomial geometry description.
///
/// Defines a curve in local coordinates using cubic polynomials:
/// - u(p) = aU + bU*p + cU*p² + dU*p³
/// - v(p) = aV + bV*p + cV*p² + dV*p³
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamPoly3 {
    /// Coefficient a for u(p)
    pub a_u: f64,
    /// Coefficient b for u(p)
    pub b_u: f64,
    /// Coefficient c for u(p)
    pub c_u: f64,
    /// Coefficient d for u(p)
    pub d_u: f64,
    /// Coefficient a for v(p)
    pub a_v: f64,
    /// Coefficient b for v(p)
    pub b_v: f64,
    /// Coefficient c for v(p)
    pub c_v: f64,
    /// Coefficient d for v(p)
    pub d_v: f64,
    /// Range type for parameter p
    pub p_range: PRange,
}

impl ParamPoly3 {
    /// Creates a new ParamPoly3 with the given coefficients.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        a_u: f64,
        b_u: f64,
        c_u: f64,
        d_u: f64,
        a_v: f64,
        b_v: f64,
        c_v: f64,
        d_v: f64,
        p_range: PRange,
    ) -> Self {
        Self {
            a_u,
            b_u,
            c_u,
            d_u,
            a_v,
            b_v,
            c_v,
            d_v,
            p_range,
        }
    }
}

impl Default for ParamPoly3 {
    fn default() -> Self {
        Self {
            a_u: 0.0,
            b_u: 1.0, // Identity in u direction
            c_u: 0.0,
            d_u: 0.0,
            a_v: 0.0,
            b_v: 0.0,
            c_v: 0.0,
            d_v: 0.0,
            p_range: PRange::ArcLength,
        }
    }
}

/// Geometry type-specific description.
#[derive(Debug, Clone, PartialEq)]
pub enum GeometryDescription {
    /// Line geometry
    Line(Line),
    /// Arc geometry with curvature
    Arc(Arc),
    /// Spiral geometry with start and end curvature
    Spiral(Spiral),
    /// Parametric cubic polynomial
    ParamPoly3(ParamPoly3),
}

impl GeometryDescription {
    /// Returns the geometry type.
    pub fn geometry_type(&self) -> GeometryType {
        match self {
            GeometryDescription::Line(_) => GeometryType::Line,
            GeometryDescription::Arc(_) => GeometryType::Arc,
            GeometryDescription::Spiral(_) => GeometryType::Spiral,
            GeometryDescription::ParamPoly3(_) => GeometryType::ParamPoly3,
        }
    }
}

/// Holds the values of an XODR geometry element.
///
/// A geometry element describes a section of the road's reference line
/// in the PlanView. Multiple geometry elements are concatenated to form
/// the complete reference line.
#[derive(Debug, Clone, PartialEq)]
pub struct Geometry {
    /// Start position (s-coordinate along the reference line).
    pub s_0: f64,
    /// Start position in inertial coordinates (x, y).
    pub start_point: Vector2<f64>,
    /// Start orientation (heading angle in radians).
    pub orientation: f64,
    /// Length of the geometry element along the reference line.
    pub length: f64,
    /// Type-specific geometry description.
    pub description: GeometryDescription,
}

impl Geometry {
    /// Creates a new line geometry.
    pub fn new_line(s_0: f64, start_point: Vector2<f64>, orientation: f64, length: f64) -> Self {
        Self {
            s_0,
            start_point,
            orientation,
            length,
            description: GeometryDescription::Line(Line),
        }
    }

    /// Creates a new arc geometry.
    pub fn new_arc(
        s_0: f64,
        start_point: Vector2<f64>,
        orientation: f64,
        length: f64,
        curvature: f64,
    ) -> Self {
        Self {
            s_0,
            start_point,
            orientation,
            length,
            description: GeometryDescription::Arc(Arc::new(curvature)),
        }
    }

    /// Creates a new spiral geometry.
    pub fn new_spiral(
        s_0: f64,
        start_point: Vector2<f64>,
        orientation: f64,
        length: f64,
        curv_start: f64,
        curv_end: f64,
    ) -> Self {
        Self {
            s_0,
            start_point,
            orientation,
            length,
            description: GeometryDescription::Spiral(Spiral::new(curv_start, curv_end)),
        }
    }

    /// Creates a new parametric polynomial geometry.
    #[allow(clippy::too_many_arguments)]
    pub fn new_param_poly3(
        s_0: f64,
        start_point: Vector2<f64>,
        orientation: f64,
        length: f64,
        param_poly3: ParamPoly3,
    ) -> Self {
        Self {
            s_0,
            start_point,
            orientation,
            length,
            description: GeometryDescription::ParamPoly3(param_poly3),
        }
    }

    /// Returns the geometry type.
    pub fn geometry_type(&self) -> GeometryType {
        self.description.geometry_type()
    }

    /// Returns the end s-coordinate of this geometry element.
    pub fn s_end(&self) -> f64 {
        self.s_0 + self.length
    }

    /// Validates the geometry parameters.
    pub fn validate(&self) -> MalidriveResult<()> {
        if self.length < 0.0 {
            return Err(MalidriveError::ValidationError(format!(
                "Geometry length must be non-negative, got {}",
                self.length
            )));
        }

        if self.s_0 < 0.0 {
            return Err(MalidriveError::ValidationError(format!(
                "Geometry s_0 must be non-negative, got {}",
                self.s_0
            )));
        }

        Ok(())
    }

    /// Returns true if this geometry is a line.
    pub fn is_line(&self) -> bool {
        matches!(self.description, GeometryDescription::Line(_))
    }

    /// Returns true if this geometry is an arc.
    pub fn is_arc(&self) -> bool {
        matches!(self.description, GeometryDescription::Arc(_))
    }

    /// Returns true if this geometry is a spiral.
    pub fn is_spiral(&self) -> bool {
        matches!(self.description, GeometryDescription::Spiral(_))
    }

    /// Returns true if this geometry is a parametric polynomial.
    pub fn is_param_poly3(&self) -> bool {
        matches!(self.description, GeometryDescription::ParamPoly3(_))
    }

    /// Returns the line description if this is a line geometry.
    pub fn as_line(&self) -> Option<&Line> {
        match &self.description {
            GeometryDescription::Line(line) => Some(line),
            _ => None,
        }
    }

    /// Returns the arc description if this is an arc geometry.
    pub fn as_arc(&self) -> Option<&Arc> {
        match &self.description {
            GeometryDescription::Arc(arc) => Some(arc),
            _ => None,
        }
    }

    /// Returns the spiral description if this is a spiral geometry.
    pub fn as_spiral(&self) -> Option<&Spiral> {
        match &self.description {
            GeometryDescription::Spiral(spiral) => Some(spiral),
            _ => None,
        }
    }

    /// Returns the parametric polynomial description if this is a ParamPoly3 geometry.
    pub fn as_param_poly3(&self) -> Option<&ParamPoly3> {
        match &self.description {
            GeometryDescription::ParamPoly3(pp3) => Some(pp3),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_geometry_type_from_str() {
        assert_eq!("line".parse::<GeometryType>().unwrap(), GeometryType::Line);
        assert_eq!("arc".parse::<GeometryType>().unwrap(), GeometryType::Arc);
        assert_eq!(
            "spiral".parse::<GeometryType>().unwrap(),
            GeometryType::Spiral
        );
        assert_eq!(
            "paramPoly3".parse::<GeometryType>().unwrap(),
            GeometryType::ParamPoly3
        );
        assert!("invalid".parse::<GeometryType>().is_err());
    }

    #[test]
    fn test_geometry_type_display() {
        assert_eq!(format!("{}", GeometryType::Line), "line");
        assert_eq!(format!("{}", GeometryType::Arc), "arc");
        assert_eq!(format!("{}", GeometryType::Spiral), "spiral");
        assert_eq!(format!("{}", GeometryType::ParamPoly3), "paramPoly3");
    }

    #[test]
    fn test_line_geometry() {
        let geom = Geometry::new_line(0.0, Vector2::new(0.0, 0.0), 0.0, 100.0);
        assert_eq!(geom.geometry_type(), GeometryType::Line);
        assert_relative_eq!(geom.s_end(), 100.0);
        assert!(geom.validate().is_ok());
    }

    #[test]
    fn test_arc_geometry() {
        let geom = Geometry::new_arc(0.0, Vector2::new(0.0, 0.0), 0.0, 100.0, 0.01);
        assert_eq!(geom.geometry_type(), GeometryType::Arc);
        if let GeometryDescription::Arc(arc) = &geom.description {
            assert_relative_eq!(arc.curvature, 0.01);
        } else {
            panic!("Expected Arc description");
        }
    }

    #[test]
    fn test_spiral_geometry() {
        let geom = Geometry::new_spiral(0.0, Vector2::new(0.0, 0.0), 0.0, 100.0, 0.0, 0.02);
        assert_eq!(geom.geometry_type(), GeometryType::Spiral);
        if let GeometryDescription::Spiral(spiral) = &geom.description {
            assert_relative_eq!(spiral.curv_start, 0.0);
            assert_relative_eq!(spiral.curv_end, 0.02);
        } else {
            panic!("Expected Spiral description");
        }
    }

    #[test]
    fn test_param_poly3_geometry() {
        let poly = ParamPoly3::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, PRange::ArcLength);
        let geom = Geometry::new_param_poly3(0.0, Vector2::new(0.0, 0.0), 0.0, 100.0, poly);
        assert_eq!(geom.geometry_type(), GeometryType::ParamPoly3);
    }

    #[test]
    fn test_geometry_validation() {
        // Valid geometry
        let geom = Geometry::new_line(0.0, Vector2::new(0.0, 0.0), 0.0, 100.0);
        assert!(geom.validate().is_ok());

        // Invalid: negative length
        let geom = Geometry::new_line(0.0, Vector2::new(0.0, 0.0), 0.0, -100.0);
        assert!(geom.validate().is_err());

        // Invalid: negative s_0
        let geom = Geometry::new_line(-10.0, Vector2::new(0.0, 0.0), 0.0, 100.0);
        assert!(geom.validate().is_err());
    }

    #[test]
    fn test_p_range_from_str() {
        assert_eq!(
            "arcLength".parse::<PRange>().unwrap(),
            PRange::ArcLength
        );
        assert_eq!(
            "normalized".parse::<PRange>().unwrap(),
            PRange::Normalized
        );
        assert!("invalid".parse::<PRange>().is_err());
    }
}
